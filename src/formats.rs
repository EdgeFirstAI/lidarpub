// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Point cloud formatting with architecture-specific SIMD optimizations.
//!
//! This module provides optimized functions for converting point cloud data
//! into the packed binary format required by ROS PointCloud2 messages.
//!
//! # Architectures
//!
//! - **aarch64**: Native NEON intrinsics (stable Rust, ~1.19x speedup)
//! - **x86_64 with `portable_simd`**: std::simd (nightly Rust, ~1.54x speedup)
//! - **Fallback**: Scalar implementation (stable Rust)
//!
//! # Formats
//!
//! ## 13-byte format (xyz + intensity)
//! ```text
//! ┌───────┬───────┬───────┬───────────┐
//! │ x:f32 │ y:f32 │ z:f32 │ intensity │
//! │ 4B    │ 4B    │ 4B    │ 1B        │
//! └───────┴───────┴───────┴───────────┘
//! ```
//!
//! ## 17-byte format (xyz + cluster_id + intensity)
//! ```text
//! ┌───────┬───────┬───────┬────────────┬───────────┐
//! │ x:f32 │ y:f32 │ z:f32 │ cluster:u32│ intensity │
//! │ 4B    │ 4B    │ 4B    │ 4B         │ 1B        │
//! └───────┴───────┴───────┴────────────┴───────────┘
//! ```

use std::borrow::Cow;

#[cfg(all(feature = "portable_simd", not(target_arch = "aarch64")))]
use std::simd::{Simd, ToBytes as _};

#[cfg(target_arch = "aarch64")]
use std::arch::aarch64::*;

use edgefirst_schemas::{
    builtin_interfaces::Time,
    cdr::CdrError,
    geometry_msgs::{Quaternion, Transform, TransformStamped, Vector3},
    sensor_msgs::{Imu, PointCloud2, PointFieldView},
};

/// Point field data types for PointCloud2 messages.
///
/// These values correspond to the ROS sensor_msgs/PointField datatype field.
/// All variants are defined for completeness, even if not all are currently
/// used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(dead_code)]
pub enum PointFieldType {
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8,
}

/// Build the standard XYZ + intensity point fields (13-byte stride).
///
/// Returns PointFieldView definitions for:
/// - x: FLOAT32 at offset 0
/// - y: FLOAT32 at offset 4
/// - z: FLOAT32 at offset 8
/// - reflect: UINT8 at offset 12
pub fn standard_xyz_intensity_fields() -> [PointFieldView<'static>; 4] {
    [
        PointFieldView {
            name: "x",
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointFieldView {
            name: "y",
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointFieldView {
            name: "z",
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointFieldView {
            name: "reflect",
            offset: 12,
            datatype: PointFieldType::UINT8 as u8,
            count: 1,
        },
    ]
}

/// Build the clustered point fields (17-byte stride).
///
/// Returns PointFieldView definitions for:
/// - x: FLOAT32 at offset 0
/// - y: FLOAT32 at offset 4
/// - z: FLOAT32 at offset 8
/// - cluster_id: UINT32 at offset 12
/// - reflect: UINT8 at offset 16
pub fn clustered_xyz_fields() -> [PointFieldView<'static>; 5] {
    [
        PointFieldView {
            name: "x",
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointFieldView {
            name: "y",
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointFieldView {
            name: "z",
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointFieldView {
            name: "cluster_id",
            offset: 12,
            datatype: PointFieldType::UINT32 as u8,
            count: 1,
        },
        PointFieldView {
            name: "reflect",
            offset: 16,
            datatype: PointFieldType::UINT8 as u8,
            count: 1,
        },
    ]
}

/// Optionally negate the first `n_points` values for axis mirroring.
///
/// # Panics
///
/// Panics if `n_points` is greater than `values.len()`.
fn mirrored(values: &[f32], n_points: usize, negate: bool) -> Cow<'_, [f32]> {
    assert!(
        n_points <= values.len(),
        "n_points ({n_points}) exceeds coordinate slice length ({})",
        values.len()
    );
    if negate {
        Cow::Owned(values[..n_points].iter().map(|v| -v).collect())
    } else {
        Cow::Borrowed(&values[..n_points])
    }
}

/// Encode XYZ+intensity points as a CDR-serialized `sensor_msgs/PointCloud2`.
#[allow(clippy::too_many_arguments)]
pub fn encode_xyzr_pointcloud2_cdr(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    intensity: &[u8],
    n_points: usize,
    timestamp: Time,
    frame_id: String,
    mirror_y: bool,
    mirror_z: bool,
) -> Result<Vec<u8>, CdrError> {
    let fields = standard_xyz_intensity_fields();
    let y = mirrored(y, n_points, mirror_y);
    let z = mirrored(z, n_points, mirror_z);

    let data = format_points_13byte(x, &y, &z, intensity, n_points);

    let msg = PointCloud2::builder()
        .stamp(timestamp)
        .frame_id(frame_id)
        .height(1)
        .width(n_points as u32)
        .fields(&fields)
        .is_bigendian(false)
        .point_step(13)
        .row_step(13 * n_points as u32)
        .data(&data)
        .is_dense(true)
        .build()?;

    Ok(msg.into_cdr())
}

/// Encode clustered XYZ+cluster_id+intensity points as CDR `PointCloud2`.
#[allow(clippy::too_many_arguments)]
pub fn encode_clustered_pointcloud2_cdr(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    cluster_ids: &[u32],
    intensity: &[u8],
    n_points: usize,
    timestamp: Time,
    frame_id: String,
    mirror_y: bool,
    mirror_z: bool,
) -> Result<Vec<u8>, CdrError> {
    let fields = clustered_xyz_fields();
    let y = mirrored(y, n_points, mirror_y);
    let z = mirrored(z, n_points, mirror_z);

    let data = format_clustered_17byte(x, &y, &z, cluster_ids, intensity, n_points);

    let msg = PointCloud2::builder()
        .stamp(timestamp)
        .frame_id(frame_id)
        .height(1)
        .width(n_points as u32)
        .fields(&fields)
        .is_bigendian(false)
        .point_step(17)
        .row_step(17 * n_points as u32)
        .data(&data)
        .is_dense(true)
        .build()?;

    Ok(msg.into_cdr())
}

/// Encode an IMU sample as CDR-serialized `sensor_msgs/Imu`.
///
/// Orientation is identity; covariances are left unset (zeros).
pub fn encode_imu_cdr(
    stamp: Time,
    frame_id: &str,
    angular_velocity: Vector3,
    linear_acceleration: Vector3,
) -> Result<Vec<u8>, CdrError> {
    let msg = Imu::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .orientation(Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        })
        .orientation_covariance([0.0; 9])
        .angular_velocity(angular_velocity)
        .angular_velocity_covariance([0.0; 9])
        .linear_acceleration(linear_acceleration)
        .linear_acceleration_covariance([0.0; 9])
        .build()?;

    Ok(msg.into_cdr())
}

/// Encode a static transform as CDR-serialized `geometry_msgs/TransformStamped`.
pub fn encode_transform_stamped_cdr(
    stamp: Time,
    parent_frame: &str,
    child_frame: &str,
    translation: Vector3,
    rotation: Quaternion,
) -> Result<Vec<u8>, CdrError> {
    let msg = TransformStamped::builder()
        .stamp(stamp)
        .frame_id(parent_frame)
        .child_frame_id(child_frame)
        .transform(Transform {
            translation,
            rotation,
        })
        .build()?;

    Ok(msg.into_cdr())
}

// ============================================================================
// 13-byte format: XYZ + intensity
// ============================================================================

/// Format point cloud data into 13-byte packed format (XYZ + intensity).
///
/// Uses architecture-specific SIMD optimizations when available.
///
/// # Arguments
///
/// * `x`, `y`, `z` - Coordinate arrays (must be at least `n_points` long)
/// * `intensity` - Intensity array (must be at least `n_points` long)
/// * `n_points` - Number of points to format
///
/// # Returns
///
/// A vector of bytes in packed 13-byte-per-point format.
#[inline(never)]
pub fn format_points_13byte(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    intensity: &[u8],
    n_points: usize,
) -> Vec<u8> {
    let mut data = vec![0u8; 13 * n_points];
    format_points_13byte_into(x, y, z, intensity, n_points, &mut data);
    data
}

/// Format point cloud data into a pre-allocated buffer (13-byte format).
///
/// # Arguments
///
/// * `x`, `y`, `z` - Coordinate arrays
/// * `intensity` - Intensity array
/// * `n_points` - Number of points to format
/// * `out` - Output buffer (must be at least `13 * n_points` bytes)
///
/// # Panics
///
/// Panics if `out` is too small.
#[cfg(target_arch = "aarch64")]
#[inline(never)]
pub fn format_points_13byte_into(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    intensity: &[u8],
    n_points: usize,
    out: &mut [u8],
) {
    assert!(out.len() >= 13 * n_points);
    let n_simd = n_points - n_points % 4;

    // SAFETY: NEON intrinsics are always available on aarch64.
    // All pointer accesses are bounds-checked by the loop conditions.
    //
    // PERFORMANCE: write_unaligned() used because 13-byte stride means writes
    // are NOT 4-byte aligned after first point. On aarch64, unaligned writes
    // have minimal penalty (~0-5% on modern cores). Benchmarks show ~1.19x
    // faster than scalar code.
    unsafe {
        let out_ptr = out.as_mut_ptr();

        for index in (0..n_simd).step_by(4) {
            // Load 4 x, y, z values using NEON
            let xv = vld1q_f32(x.as_ptr().add(index));
            let yv = vld1q_f32(y.as_ptr().add(index));
            let zv = vld1q_f32(z.as_ptr().add(index));

            // Reinterpret as u32 for lane extraction
            let x_u32 = vreinterpretq_u32_f32(xv);
            let y_u32 = vreinterpretq_u32_f32(yv);
            let z_u32 = vreinterpretq_u32_f32(zv);

            // Extract all lanes
            let x0 = vgetq_lane_u32::<0>(x_u32);
            let x1 = vgetq_lane_u32::<1>(x_u32);
            let x2 = vgetq_lane_u32::<2>(x_u32);
            let x3 = vgetq_lane_u32::<3>(x_u32);

            let y0 = vgetq_lane_u32::<0>(y_u32);
            let y1 = vgetq_lane_u32::<1>(y_u32);
            let y2 = vgetq_lane_u32::<2>(y_u32);
            let y3 = vgetq_lane_u32::<3>(y_u32);

            let z0 = vgetq_lane_u32::<0>(z_u32);
            let z1 = vgetq_lane_u32::<1>(z_u32);
            let z2 = vgetq_lane_u32::<2>(z_u32);
            let z3 = vgetq_lane_u32::<3>(z_u32);

            // Write points with 13-byte stride
            let base = out_ptr.add(index * 13);

            // Point 0
            (base as *mut u32).write_unaligned(x0);
            (base.add(4) as *mut u32).write_unaligned(y0);
            (base.add(8) as *mut u32).write_unaligned(z0);
            *base.add(12) = intensity[index];

            // Point 1
            let p1 = base.add(13);
            (p1 as *mut u32).write_unaligned(x1);
            (p1.add(4) as *mut u32).write_unaligned(y1);
            (p1.add(8) as *mut u32).write_unaligned(z1);
            *p1.add(12) = intensity[index + 1];

            // Point 2
            let p2 = base.add(26);
            (p2 as *mut u32).write_unaligned(x2);
            (p2.add(4) as *mut u32).write_unaligned(y2);
            (p2.add(8) as *mut u32).write_unaligned(z2);
            *p2.add(12) = intensity[index + 2];

            // Point 3
            let p3 = base.add(39);
            (p3 as *mut u32).write_unaligned(x3);
            (p3.add(4) as *mut u32).write_unaligned(y3);
            (p3.add(8) as *mut u32).write_unaligned(z3);
            *p3.add(12) = intensity[index + 3];
        }
    }

    // Handle remainder with scalar code
    for index in n_simd..n_points {
        let offset = index * 13;
        out[offset..offset + 4].copy_from_slice(&x[index].to_le_bytes());
        out[offset + 4..offset + 8].copy_from_slice(&y[index].to_le_bytes());
        out[offset + 8..offset + 12].copy_from_slice(&z[index].to_le_bytes());
        out[offset + 12] = intensity[index];
    }
}

/// Portable SIMD implementation for non-aarch64 targets (requires nightly).
#[cfg(all(feature = "portable_simd", not(target_arch = "aarch64")))]
#[inline(never)]
pub fn format_points_13byte_into(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    intensity: &[u8],
    n_points: usize,
    out: &mut [u8],
) {
    assert!(out.len() >= 13 * n_points);
    const N: usize = 4;
    let n_simd = n_points - n_points % N;

    for index in (0..n_simd).step_by(N) {
        let xv = Simd::<f32, N>::from_slice(&x[index..index + N]);
        let xb = xv.to_le_bytes();
        let yv = Simd::<f32, N>::from_slice(&y[index..index + N]);
        let yb = yv.to_le_bytes();
        let zv = Simd::<f32, N>::from_slice(&z[index..index + N]);
        let zb = zv.to_le_bytes();

        let offset = index * 13;
        out[offset..offset + 4].copy_from_slice(&xb[..4]);
        out[offset + 4..offset + 8].copy_from_slice(&yb[..4]);
        out[offset + 8..offset + 12].copy_from_slice(&zb[..4]);
        out[offset + 12] = intensity[index];

        let offset = (index + 1) * 13;
        out[offset..offset + 4].copy_from_slice(&xb[4..8]);
        out[offset + 4..offset + 8].copy_from_slice(&yb[4..8]);
        out[offset + 8..offset + 12].copy_from_slice(&zb[4..8]);
        out[offset + 12] = intensity[index + 1];

        let offset = (index + 2) * 13;
        out[offset..offset + 4].copy_from_slice(&xb[8..12]);
        out[offset + 4..offset + 8].copy_from_slice(&yb[8..12]);
        out[offset + 8..offset + 12].copy_from_slice(&zb[8..12]);
        out[offset + 12] = intensity[index + 2];

        let offset = (index + 3) * 13;
        out[offset..offset + 4].copy_from_slice(&xb[12..16]);
        out[offset + 4..offset + 8].copy_from_slice(&yb[12..16]);
        out[offset + 8..offset + 12].copy_from_slice(&zb[12..16]);
        out[offset + 12] = intensity[index + 3];
    }

    for index in n_simd..n_points {
        let offset = index * 13;
        out[offset..offset + 4].copy_from_slice(&x[index].to_le_bytes());
        out[offset + 4..offset + 8].copy_from_slice(&y[index].to_le_bytes());
        out[offset + 8..offset + 12].copy_from_slice(&z[index].to_le_bytes());
        out[offset + 12] = intensity[index];
    }
}

/// Scalar fallback for non-aarch64 targets without portable_simd.
#[cfg(all(not(feature = "portable_simd"), not(target_arch = "aarch64")))]
#[inline(never)]
pub fn format_points_13byte_into(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    intensity: &[u8],
    n_points: usize,
    out: &mut [u8],
) {
    assert!(out.len() >= 13 * n_points);

    for index in 0..n_points {
        let offset = index * 13;
        out[offset..offset + 4].copy_from_slice(&x[index].to_le_bytes());
        out[offset + 4..offset + 8].copy_from_slice(&y[index].to_le_bytes());
        out[offset + 8..offset + 12].copy_from_slice(&z[index].to_le_bytes());
        out[offset + 12] = intensity[index];
    }
}

// ============================================================================
// 17-byte format: XYZ + cluster_id + intensity
// ============================================================================

/// Format clustered point cloud data into 17-byte packed format.
///
/// Uses architecture-specific SIMD optimizations when available.
///
/// # Arguments
///
/// * `x`, `y`, `z` - Coordinate arrays
/// * `cluster_ids` - Cluster ID array
/// * `intensity` - Intensity array
/// * `n_points` - Number of points to format
///
/// # Returns
///
/// A vector of bytes in packed 17-byte-per-point format.
#[inline(never)]
pub fn format_clustered_17byte(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    cluster_ids: &[u32],
    intensity: &[u8],
    n_points: usize,
) -> Vec<u8> {
    let mut data = vec![0u8; 17 * n_points];
    format_clustered_17byte_into(x, y, z, cluster_ids, intensity, n_points, &mut data);
    data
}

/// Format clustered point cloud data into a pre-allocated buffer (17-byte
/// format).
///
/// # aarch64 NEON Implementation
///
/// Processes 4 points at a time using NEON vector loads and lane extraction.
#[cfg(target_arch = "aarch64")]
#[inline(never)]
pub fn format_clustered_17byte_into(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    cluster_ids: &[u32],
    intensity: &[u8],
    n_points: usize,
    out: &mut [u8],
) {
    assert!(out.len() >= 17 * n_points);
    let n_simd = n_points - n_points % 4;

    // SAFETY: NEON intrinsics are always available on aarch64.
    unsafe {
        let out_ptr = out.as_mut_ptr();

        for index in (0..n_simd).step_by(4) {
            // Load 4 x, y, z, cluster_id values using NEON
            let xv = vld1q_f32(x.as_ptr().add(index));
            let yv = vld1q_f32(y.as_ptr().add(index));
            let zv = vld1q_f32(z.as_ptr().add(index));
            let idv = vld1q_u32(cluster_ids.as_ptr().add(index));

            // Reinterpret floats as u32 for lane extraction
            let x_u32 = vreinterpretq_u32_f32(xv);
            let y_u32 = vreinterpretq_u32_f32(yv);
            let z_u32 = vreinterpretq_u32_f32(zv);

            // Extract all lanes
            let x0 = vgetq_lane_u32::<0>(x_u32);
            let x1 = vgetq_lane_u32::<1>(x_u32);
            let x2 = vgetq_lane_u32::<2>(x_u32);
            let x3 = vgetq_lane_u32::<3>(x_u32);

            let y0 = vgetq_lane_u32::<0>(y_u32);
            let y1 = vgetq_lane_u32::<1>(y_u32);
            let y2 = vgetq_lane_u32::<2>(y_u32);
            let y3 = vgetq_lane_u32::<3>(y_u32);

            let z0 = vgetq_lane_u32::<0>(z_u32);
            let z1 = vgetq_lane_u32::<1>(z_u32);
            let z2 = vgetq_lane_u32::<2>(z_u32);
            let z3 = vgetq_lane_u32::<3>(z_u32);

            let id0 = vgetq_lane_u32::<0>(idv);
            let id1 = vgetq_lane_u32::<1>(idv);
            let id2 = vgetq_lane_u32::<2>(idv);
            let id3 = vgetq_lane_u32::<3>(idv);

            // Write points with 17-byte stride
            let base = out_ptr.add(index * 17);

            // Point 0: x(4) + y(4) + z(4) + id(4) + intensity(1) = 17 bytes
            (base as *mut u32).write_unaligned(x0);
            (base.add(4) as *mut u32).write_unaligned(y0);
            (base.add(8) as *mut u32).write_unaligned(z0);
            (base.add(12) as *mut u32).write_unaligned(id0);
            *base.add(16) = intensity[index];

            // Point 1
            let p1 = base.add(17);
            (p1 as *mut u32).write_unaligned(x1);
            (p1.add(4) as *mut u32).write_unaligned(y1);
            (p1.add(8) as *mut u32).write_unaligned(z1);
            (p1.add(12) as *mut u32).write_unaligned(id1);
            *p1.add(16) = intensity[index + 1];

            // Point 2
            let p2 = base.add(34);
            (p2 as *mut u32).write_unaligned(x2);
            (p2.add(4) as *mut u32).write_unaligned(y2);
            (p2.add(8) as *mut u32).write_unaligned(z2);
            (p2.add(12) as *mut u32).write_unaligned(id2);
            *p2.add(16) = intensity[index + 2];

            // Point 3
            let p3 = base.add(51);
            (p3 as *mut u32).write_unaligned(x3);
            (p3.add(4) as *mut u32).write_unaligned(y3);
            (p3.add(8) as *mut u32).write_unaligned(z3);
            (p3.add(12) as *mut u32).write_unaligned(id3);
            *p3.add(16) = intensity[index + 3];
        }
    }

    // Handle remainder with scalar code
    for index in n_simd..n_points {
        let offset = index * 17;
        out[offset..offset + 4].copy_from_slice(&x[index].to_le_bytes());
        out[offset + 4..offset + 8].copy_from_slice(&y[index].to_le_bytes());
        out[offset + 8..offset + 12].copy_from_slice(&z[index].to_le_bytes());
        out[offset + 12..offset + 16].copy_from_slice(&cluster_ids[index].to_le_bytes());
        out[offset + 16] = intensity[index];
    }
}

/// Portable SIMD implementation for clustered data.
#[cfg(all(feature = "portable_simd", not(target_arch = "aarch64")))]
#[inline(never)]
pub fn format_clustered_17byte_into(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    cluster_ids: &[u32],
    intensity: &[u8],
    n_points: usize,
    out: &mut [u8],
) {
    assert!(out.len() >= 17 * n_points);
    const N: usize = 4;
    let n_simd = n_points - n_points % N;

    for index in (0..n_simd).step_by(N) {
        let xv = Simd::<f32, N>::from_slice(&x[index..index + N]);
        let xb = xv.to_le_bytes();
        let yv = Simd::<f32, N>::from_slice(&y[index..index + N]);
        let yb = yv.to_le_bytes();
        let zv = Simd::<f32, N>::from_slice(&z[index..index + N]);
        let zb = zv.to_le_bytes();
        let idv = Simd::<u32, N>::from_slice(&cluster_ids[index..index + N]);
        let idb = idv.to_le_bytes();

        let offset = index * 17;
        out[offset..offset + 4].copy_from_slice(&xb[..4]);
        out[offset + 4..offset + 8].copy_from_slice(&yb[..4]);
        out[offset + 8..offset + 12].copy_from_slice(&zb[..4]);
        out[offset + 12..offset + 16].copy_from_slice(&idb[..4]);
        out[offset + 16] = intensity[index];

        let offset = (index + 1) * 17;
        out[offset..offset + 4].copy_from_slice(&xb[4..8]);
        out[offset + 4..offset + 8].copy_from_slice(&yb[4..8]);
        out[offset + 8..offset + 12].copy_from_slice(&zb[4..8]);
        out[offset + 12..offset + 16].copy_from_slice(&idb[4..8]);
        out[offset + 16] = intensity[index + 1];

        let offset = (index + 2) * 17;
        out[offset..offset + 4].copy_from_slice(&xb[8..12]);
        out[offset + 4..offset + 8].copy_from_slice(&yb[8..12]);
        out[offset + 8..offset + 12].copy_from_slice(&zb[8..12]);
        out[offset + 12..offset + 16].copy_from_slice(&idb[8..12]);
        out[offset + 16] = intensity[index + 2];

        let offset = (index + 3) * 17;
        out[offset..offset + 4].copy_from_slice(&xb[12..16]);
        out[offset + 4..offset + 8].copy_from_slice(&yb[12..16]);
        out[offset + 8..offset + 12].copy_from_slice(&zb[12..16]);
        out[offset + 12..offset + 16].copy_from_slice(&idb[12..16]);
        out[offset + 16] = intensity[index + 3];
    }

    for index in n_simd..n_points {
        let offset = index * 17;
        out[offset..offset + 4].copy_from_slice(&x[index].to_le_bytes());
        out[offset + 4..offset + 8].copy_from_slice(&y[index].to_le_bytes());
        out[offset + 8..offset + 12].copy_from_slice(&z[index].to_le_bytes());
        out[offset + 12..offset + 16].copy_from_slice(&cluster_ids[index].to_le_bytes());
        out[offset + 16] = intensity[index];
    }
}

/// Scalar fallback for clustered data.
#[cfg(all(not(feature = "portable_simd"), not(target_arch = "aarch64")))]
#[inline(never)]
pub fn format_clustered_17byte_into(
    x: &[f32],
    y: &[f32],
    z: &[f32],
    cluster_ids: &[u32],
    intensity: &[u8],
    n_points: usize,
    out: &mut [u8],
) {
    assert!(out.len() >= 17 * n_points);

    for index in 0..n_points {
        let offset = index * 17;
        out[offset..offset + 4].copy_from_slice(&x[index].to_le_bytes());
        out[offset + 4..offset + 8].copy_from_slice(&y[index].to_le_bytes());
        out[offset + 8..offset + 12].copy_from_slice(&z[index].to_le_bytes());
        out[offset + 12..offset + 16].copy_from_slice(&cluster_ids[index].to_le_bytes());
        out[offset + 16] = intensity[index];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_points_13byte() {
        let x = vec![1.0f32, 2.0, 3.0, 4.0, 5.0];
        let y = vec![10.0f32, 20.0, 30.0, 40.0, 50.0];
        let z = vec![100.0f32, 200.0, 300.0, 400.0, 500.0];
        let intensity = vec![128u8, 64, 255, 0, 100];

        let data = format_points_13byte(&x, &y, &z, &intensity, 5);
        assert_eq!(data.len(), 13 * 5);

        // Verify first point
        let x0 = f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let y0 = f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let z0 = f32::from_le_bytes([data[8], data[9], data[10], data[11]]);
        let i0 = data[12];

        assert_eq!(x0, 1.0);
        assert_eq!(y0, 10.0);
        assert_eq!(z0, 100.0);
        assert_eq!(i0, 128);

        // Verify last point
        let offset = 4 * 13;
        let x4 = f32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        let y4 = f32::from_le_bytes([
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        let z4 = f32::from_le_bytes([
            data[offset + 8],
            data[offset + 9],
            data[offset + 10],
            data[offset + 11],
        ]);
        let i4 = data[offset + 12];

        assert_eq!(x4, 5.0);
        assert_eq!(y4, 50.0);
        assert_eq!(z4, 500.0);
        assert_eq!(i4, 100);
    }

    #[test]
    fn test_format_clustered_17byte() {
        let x = vec![1.0f32, 2.0, 3.0, 4.0, 5.0];
        let y = vec![10.0f32, 20.0, 30.0, 40.0, 50.0];
        let z = vec![100.0f32, 200.0, 300.0, 400.0, 500.0];
        let cluster_ids = vec![1u32, 1, 2, 2, 0];
        let intensity = vec![128u8, 64, 255, 0, 100];

        let data = format_clustered_17byte(&x, &y, &z, &cluster_ids, &intensity, 5);
        assert_eq!(data.len(), 17 * 5);

        // Verify first point
        let x0 = f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let y0 = f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let z0 = f32::from_le_bytes([data[8], data[9], data[10], data[11]]);
        let id0 = u32::from_le_bytes([data[12], data[13], data[14], data[15]]);
        let i0 = data[16];

        assert_eq!(x0, 1.0);
        assert_eq!(y0, 10.0);
        assert_eq!(z0, 100.0);
        assert_eq!(id0, 1);
        assert_eq!(i0, 128);

        // Verify third point (index 2)
        let offset = 2 * 17;
        let id2 = u32::from_le_bytes([
            data[offset + 12],
            data[offset + 13],
            data[offset + 14],
            data[offset + 15],
        ]);
        assert_eq!(id2, 2);
    }

    #[test]
    fn test_point_field_builders() {
        let fields = standard_xyz_intensity_fields();
        assert_eq!(fields.len(), 4);
        assert_eq!(fields[0].name, "x");
        assert_eq!(fields[0].offset, 0);
        assert_eq!(fields[3].name, "reflect");
        assert_eq!(fields[3].offset, 12);

        let clustered = clustered_xyz_fields();
        assert_eq!(clustered.len(), 5);
        assert_eq!(clustered[3].name, "cluster_id");
        assert_eq!(clustered[3].offset, 12);
        assert_eq!(clustered[4].name, "reflect");
        assert_eq!(clustered[4].offset, 16);
    }

    #[test]
    fn test_format_into_preallocated() {
        let x = vec![1.0f32; 100];
        let y = vec![2.0f32; 100];
        let z = vec![3.0f32; 100];
        let intensity = vec![255u8; 100];

        // Pre-allocate buffer
        let mut buffer = vec![0u8; 13 * 100];
        format_points_13byte_into(&x, &y, &z, &intensity, 100, &mut buffer);

        // Verify last point
        let offset = 99 * 13;
        let x_last = f32::from_le_bytes([
            buffer[offset],
            buffer[offset + 1],
            buffer[offset + 2],
            buffer[offset + 3],
        ]);
        assert_eq!(x_last, 1.0);
    }

    #[test]
    fn test_encode_xyzr_pointcloud2_cdr_roundtrip() {
        let x = [1.0f32, 2.0];
        let y = [10.0f32, 20.0];
        let z = [100.0f32, 200.0];
        let intensity = [128u8, 64];
        let stamp = Time {
            sec: 1,
            nanosec: 500,
        };

        let cdr = encode_xyzr_pointcloud2_cdr(
            &x,
            &y,
            &z,
            &intensity,
            2,
            stamp,
            "lidar".to_string(),
            false,
            false,
        )
        .expect("encode PointCloud2");
        let pc = PointCloud2::from_cdr(cdr).expect("decode PointCloud2");

        assert_eq!(pc.stamp(), stamp);
        assert_eq!(pc.frame_id(), "lidar");
        assert_eq!(pc.height(), 1);
        assert_eq!(pc.width(), 2);
        assert_eq!(pc.point_step(), 13);
        assert_eq!(pc.row_step(), 26);
        assert!(pc.is_dense());
        assert_eq!(pc.data().len(), 26);
        assert_eq!(pc.fields_len(), 4);

        let x0 = f32::from_le_bytes(pc.data()[0..4].try_into().unwrap());
        let y0 = f32::from_le_bytes(pc.data()[4..8].try_into().unwrap());
        let z0 = f32::from_le_bytes(pc.data()[8..12].try_into().unwrap());
        assert_eq!(x0, 1.0);
        assert_eq!(y0, 10.0);
        assert_eq!(z0, 100.0);
        assert_eq!(pc.data()[12], 128);
    }

    #[test]
    fn test_encode_xyzr_pointcloud2_cdr_mirrors_axes() {
        let x = [1.0f32];
        let y = [2.0f32];
        let z = [3.0f32];
        let intensity = [1u8];
        let stamp = Time { sec: 0, nanosec: 0 };

        let cdr = encode_xyzr_pointcloud2_cdr(
            &x,
            &y,
            &z,
            &intensity,
            1,
            stamp,
            "lidar".to_string(),
            true,
            true,
        )
        .expect("encode mirrored PointCloud2");
        let pc = PointCloud2::from_cdr(cdr).expect("decode PointCloud2");
        let y0 = f32::from_le_bytes(pc.data()[4..8].try_into().unwrap());
        let z0 = f32::from_le_bytes(pc.data()[8..12].try_into().unwrap());
        assert_eq!(y0, -2.0);
        assert_eq!(z0, -3.0);
    }

    #[test]
    #[should_panic(expected = "n_points (2) exceeds coordinate slice length (1)")]
    fn test_mirrored_rejects_short_slice() {
        let x = [1.0f32, 2.0];
        let y = [10.0f32];
        let z = [100.0f32, 200.0];
        let intensity = [128u8, 64];
        let stamp = Time { sec: 0, nanosec: 0 };
        let _ = encode_xyzr_pointcloud2_cdr(
            &x,
            &y,
            &z,
            &intensity,
            2,
            stamp,
            "lidar".to_string(),
            true,
            false,
        );
    }

    #[test]
    fn test_encode_clustered_pointcloud2_cdr_roundtrip() {
        let x = [1.0f32, 2.0];
        let y = [10.0f32, 20.0];
        let z = [100.0f32, 200.0];
        let cluster_ids = [7u32, 8];
        let intensity = [128u8, 64];
        let stamp = Time { sec: 2, nanosec: 0 };

        let cdr = encode_clustered_pointcloud2_cdr(
            &x,
            &y,
            &z,
            &cluster_ids,
            &intensity,
            2,
            stamp,
            "cluster".to_string(),
            false,
            true,
        )
        .expect("encode clustered PointCloud2");
        let pc = PointCloud2::from_cdr(cdr).expect("decode clustered PointCloud2");

        assert_eq!(pc.frame_id(), "cluster");
        assert_eq!(pc.width(), 2);
        assert_eq!(pc.point_step(), 17);
        assert_eq!(pc.fields_len(), 5);
        assert_eq!(pc.data().len(), 34);

        let y0 = f32::from_le_bytes(pc.data()[4..8].try_into().unwrap());
        let z0 = f32::from_le_bytes(pc.data()[8..12].try_into().unwrap());
        let id0 = u32::from_le_bytes(pc.data()[12..16].try_into().unwrap());
        assert_eq!(y0, 10.0);
        assert_eq!(z0, -100.0);
        assert_eq!(id0, 7);
        assert_eq!(pc.data()[16], 128);
    }

    #[test]
    fn test_encode_imu_cdr_roundtrip() {
        let stamp = Time { sec: 3, nanosec: 7 };
        let gyro = Vector3 {
            x: 0.1,
            y: 0.2,
            z: 0.3,
        };
        let accel = Vector3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };

        let cdr = encode_imu_cdr(stamp, "imu", gyro, accel).expect("encode Imu");
        let imu = Imu::from_cdr(cdr).expect("decode Imu");

        assert_eq!(imu.stamp(), stamp);
        assert_eq!(imu.frame_id(), "imu");
        assert_eq!(imu.orientation().w, 1.0);
        assert_eq!(imu.angular_velocity(), gyro);
        assert_eq!(imu.linear_acceleration(), accel);
    }

    #[test]
    fn test_encode_transform_stamped_cdr_roundtrip() {
        let stamp = Time { sec: 4, nanosec: 0 };
        let translation = Vector3 {
            x: 0.1,
            y: 0.2,
            z: 0.3,
        };
        let rotation = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };

        let cdr = encode_transform_stamped_cdr(stamp, "base", "lidar", translation, rotation)
            .expect("encode TransformStamped");
        let tf = TransformStamped::from_cdr(cdr).expect("decode TransformStamped");

        assert_eq!(tf.stamp(), stamp);
        assert_eq!(tf.frame_id(), "base");
        assert_eq!(tf.child_frame_id(), "lidar");
        assert_eq!(tf.transform().translation, translation);
        assert_eq!(tf.transform().rotation, rotation);
    }

    type Xyzr = (Vec<f32>, Vec<f32>, Vec<f32>, Vec<u8>);

    fn load_pcd_xyzr(path: &str) -> Option<Xyzr> {
        let data = std::fs::read(path).ok()?;
        let header_end = data.windows(12).position(|w| w == b"DATA binary\n")? + 12;
        let body = &data[header_end..];
        let point_size = 13;
        if body.len() % point_size != 0 {
            return None;
        }
        let n = body.len() / point_size;
        let mut x = Vec::with_capacity(n);
        let mut y = Vec::with_capacity(n);
        let mut z = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        for i in 0..n {
            let off = i * point_size;
            x.push(f32::from_le_bytes(body[off..off + 4].try_into().ok()?));
            y.push(f32::from_le_bytes(body[off + 4..off + 8].try_into().ok()?));
            z.push(f32::from_le_bytes(body[off + 8..off + 12].try_into().ok()?));
            intensity.push(body[off + 12]);
        }
        Some((x, y, z, intensity))
    }

    #[test]
    fn test_encode_e1r_pcd_pointcloud2() {
        let Some((x, y, z, intensity)) = load_pcd_xyzr("testdata/e1r_frame0.pcd") else {
            eprintln!("Skipping: testdata/e1r_frame0.pcd not found");
            return;
        };
        let n = x.len();
        assert!(n > 10_000);
        let stamp = Time { sec: 1, nanosec: 0 };
        let cdr = encode_xyzr_pointcloud2_cdr(
            &x,
            &y,
            &z,
            &intensity,
            n,
            stamp,
            "lidar".to_string(),
            false,
            false,
        )
        .expect("encode E1R PointCloud2");
        let pc = PointCloud2::from_cdr(cdr).expect("decode E1R PointCloud2");
        assert_eq!(pc.width() as usize, n);
        assert_eq!(pc.point_step(), 13);
        assert_eq!(pc.data().len(), n * 13);

        let cluster_ids = vec![1u32; n];
        let clustered = encode_clustered_pointcloud2_cdr(
            &x,
            &y,
            &z,
            &cluster_ids,
            &intensity,
            n,
            stamp,
            "lidar".to_string(),
            true,
            false,
        )
        .expect("encode clustered E1R PointCloud2");
        let pc = PointCloud2::from_cdr(clustered).expect("decode clustered E1R");
        assert_eq!(pc.point_step(), 17);
        assert_eq!(pc.width() as usize, n);
        let y0 = f32::from_le_bytes(pc.data()[4..8].try_into().unwrap());
        assert_eq!(y0, -y[0]);
    }
}
