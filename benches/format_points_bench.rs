// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Benchmark comparing SIMD vs scalar point formatting
//!
//! Run with: cargo bench --bench format_points_bench
//! Or cross-compile and run on target

#![cfg_attr(feature = "portable_simd", feature(portable_simd))]

use std::time::{Duration, Instant};

#[cfg(feature = "portable_simd")]
use std::simd::{Simd, ToBytes as _};

#[cfg(target_arch = "aarch64")]
use std::arch::aarch64::*;

const N_POINTS: usize = 25_000; // Typical E1R frame size
const ITERATIONS: usize = 1000;

/// Points structure matching the main crate
struct Points {
    x: Vec<f32>,
    y: Vec<f32>,
    z: Vec<f32>,
    intensity: Vec<u8>,
}

impl Points {
    fn new(size: usize) -> Self {
        Self {
            x: vec![0.0; size],
            y: vec![0.0; size],
            z: vec![0.0; size],
            intensity: vec![0; size],
        }
    }

    fn randomize(&mut self) {
        for i in 0..self.x.len() {
            self.x[i] = (i as f32 * 0.01).sin() * 10.0;
            self.y[i] = (i as f32 * 0.02).cos() * 10.0;
            self.z[i] = (i as f32 * 0.005) % 5.0;
            self.intensity[i] = (i % 256) as u8;
        }
    }
}

/// Scalar implementation - simple loop
#[inline(never)]
fn format_points_scalar(points: &Points, n_points: usize) -> Vec<u8> {
    let mut data = vec![0u8; 13 * n_points];

    for index in 0..n_points {
        let x = points.x[index].to_le_bytes();
        let y = points.y[index].to_le_bytes();
        let z = points.z[index].to_le_bytes();
        let intensity = points.intensity[index];

        let out = index * 13;
        data[out..out + 4].copy_from_slice(&x);
        data[out + 4..out + 8].copy_from_slice(&y);
        data[out + 8..out + 12].copy_from_slice(&z);
        data[out + 12] = intensity;
    }

    data
}

/// Scalar with manual unrolling (4x)
#[inline(never)]
fn format_points_scalar_unrolled(points: &Points, n_points: usize) -> Vec<u8> {
    let mut data = vec![0u8; 13 * n_points];
    let n_unroll = n_points - n_points % 4;

    for index in (0..n_unroll).step_by(4) {
        // Point 0
        let out = index * 13;
        data[out..out + 4].copy_from_slice(&points.x[index].to_le_bytes());
        data[out + 4..out + 8].copy_from_slice(&points.y[index].to_le_bytes());
        data[out + 8..out + 12].copy_from_slice(&points.z[index].to_le_bytes());
        data[out + 12] = points.intensity[index];

        // Point 1
        let out = (index + 1) * 13;
        data[out..out + 4].copy_from_slice(&points.x[index + 1].to_le_bytes());
        data[out + 4..out + 8].copy_from_slice(&points.y[index + 1].to_le_bytes());
        data[out + 8..out + 12].copy_from_slice(&points.z[index + 1].to_le_bytes());
        data[out + 12] = points.intensity[index + 1];

        // Point 2
        let out = (index + 2) * 13;
        data[out..out + 4].copy_from_slice(&points.x[index + 2].to_le_bytes());
        data[out + 4..out + 8].copy_from_slice(&points.y[index + 2].to_le_bytes());
        data[out + 8..out + 12].copy_from_slice(&points.z[index + 2].to_le_bytes());
        data[out + 12] = points.intensity[index + 2];

        // Point 3
        let out = (index + 3) * 13;
        data[out..out + 4].copy_from_slice(&points.x[index + 3].to_le_bytes());
        data[out + 4..out + 8].copy_from_slice(&points.y[index + 3].to_le_bytes());
        data[out + 8..out + 12].copy_from_slice(&points.z[index + 3].to_le_bytes());
        data[out + 12] = points.intensity[index + 3];
    }

    // Handle remainder
    for index in n_unroll..n_points {
        let out = index * 13;
        data[out..out + 4].copy_from_slice(&points.x[index].to_le_bytes());
        data[out + 4..out + 8].copy_from_slice(&points.y[index].to_le_bytes());
        data[out + 8..out + 12].copy_from_slice(&points.z[index].to_le_bytes());
        data[out + 12] = points.intensity[index];
    }

    data
}

/// Portable SIMD implementation (current approach)
#[cfg(feature = "portable_simd")]
#[inline(never)]
fn format_points_simd(points: &Points, n_points: usize) -> Vec<u8> {
    const N: usize = 4;
    let mut data = vec![0u8; 13 * n_points];

    let n_simd = n_points - n_points % N;
    for index in (0..n_simd).step_by(N) {
        let x = Simd::<f32, N>::from_slice(&points.x[index..index + N]);
        let x = x.to_le_bytes();
        let y = Simd::<f32, N>::from_slice(&points.y[index..index + N]);
        let y = y.to_le_bytes();
        let z = Simd::<f32, N>::from_slice(&points.z[index..index + N]);
        let z = z.to_le_bytes();

        let out = index * 13;
        data[out..out + 4].copy_from_slice(&x[..4]);
        data[out + 4..out + 8].copy_from_slice(&y[..4]);
        data[out + 8..out + 12].copy_from_slice(&z[..4]);
        data[out + 12] = points.intensity[index];

        let out = (index + 1) * 13;
        data[out..out + 4].copy_from_slice(&x[4..8]);
        data[out + 4..out + 8].copy_from_slice(&y[4..8]);
        data[out + 8..out + 12].copy_from_slice(&z[4..8]);
        data[out + 12] = points.intensity[index + 1];

        let out = (index + 2) * 13;
        data[out..out + 4].copy_from_slice(&x[8..12]);
        data[out + 4..out + 8].copy_from_slice(&y[8..12]);
        data[out + 8..out + 12].copy_from_slice(&z[8..12]);
        data[out + 12] = points.intensity[index + 2];

        let out = (index + 3) * 13;
        data[out..out + 4].copy_from_slice(&x[12..16]);
        data[out + 4..out + 8].copy_from_slice(&y[12..16]);
        data[out + 8..out + 12].copy_from_slice(&z[12..16]);
        data[out + 12] = points.intensity[index + 3];
    }

    for index in n_simd..n_points {
        let x = points.x[index].to_le_bytes();
        let y = points.y[index].to_le_bytes();
        let z = points.z[index].to_le_bytes();
        let l = points.intensity[index];

        let out = index * 13;
        data[out..out + 4].copy_from_slice(&x);
        data[out + 4..out + 8].copy_from_slice(&y);
        data[out + 8..out + 12].copy_from_slice(&z);
        data[out + 12] = l;
    }

    data
}

/// Unsafe raw pointer approach (potentially fastest)
#[inline(never)]
fn format_points_unsafe(points: &Points, n_points: usize) -> Vec<u8> {
    let mut data = vec![0u8; 13 * n_points];

    unsafe {
        let ptr = data.as_mut_ptr();
        for index in 0..n_points {
            let out = ptr.add(index * 13);
            std::ptr::copy_nonoverlapping(&points.x[index] as *const f32 as *const u8, out, 4);
            std::ptr::copy_nonoverlapping(
                &points.y[index] as *const f32 as *const u8,
                out.add(4),
                4,
            );
            std::ptr::copy_nonoverlapping(
                &points.z[index] as *const f32 as *const u8,
                out.add(8),
                4,
            );
            *out.add(12) = points.intensity[index];
        }
    }

    data
}

/// NEON SIMD implementation for aarch64
/// Uses native ARM NEON intrinsics for optimal performance
#[cfg(target_arch = "aarch64")]
#[inline(never)]
fn format_points_neon(points: &Points, n_points: usize) -> Vec<u8> {
    let mut data = vec![0u8; 13 * n_points];
    let n_simd = n_points - n_points % 4;

    unsafe {
        let out_ptr = data.as_mut_ptr();

        for index in (0..n_simd).step_by(4) {
            // Load 4 x, y, z values using NEON
            let x = vld1q_f32(points.x.as_ptr().add(index));
            let y = vld1q_f32(points.y.as_ptr().add(index));
            let z = vld1q_f32(points.z.as_ptr().add(index));

            // Reinterpret as u32 for lane extraction
            let x_u32 = vreinterpretq_u32_f32(x);
            let y_u32 = vreinterpretq_u32_f32(y);
            let z_u32 = vreinterpretq_u32_f32(z);

            // Process 4 points - extract lanes and store
            // Point 0
            let out = out_ptr.add(index * 13);
            vst1q_lane_u32::<0>(out as *mut u32, x_u32);
            vst1q_lane_u32::<0>(out.add(4) as *mut u32, y_u32);
            vst1q_lane_u32::<0>(out.add(8) as *mut u32, z_u32);
            *out.add(12) = points.intensity[index];

            // Point 1
            let out = out_ptr.add((index + 1) * 13);
            vst1q_lane_u32::<1>(out as *mut u32, x_u32);
            vst1q_lane_u32::<1>(out.add(4) as *mut u32, y_u32);
            vst1q_lane_u32::<1>(out.add(8) as *mut u32, z_u32);
            *out.add(12) = points.intensity[index + 1];

            // Point 2
            let out = out_ptr.add((index + 2) * 13);
            vst1q_lane_u32::<2>(out as *mut u32, x_u32);
            vst1q_lane_u32::<2>(out.add(4) as *mut u32, y_u32);
            vst1q_lane_u32::<2>(out.add(8) as *mut u32, z_u32);
            *out.add(12) = points.intensity[index + 2];

            // Point 3
            let out = out_ptr.add((index + 3) * 13);
            vst1q_lane_u32::<3>(out as *mut u32, x_u32);
            vst1q_lane_u32::<3>(out.add(4) as *mut u32, y_u32);
            vst1q_lane_u32::<3>(out.add(8) as *mut u32, z_u32);
            *out.add(12) = points.intensity[index + 3];
        }
    }

    // Handle remainder with scalar
    for index in n_simd..n_points {
        let out = index * 13;
        data[out..out + 4].copy_from_slice(&points.x[index].to_le_bytes());
        data[out + 4..out + 8].copy_from_slice(&points.y[index].to_le_bytes());
        data[out + 8..out + 12].copy_from_slice(&points.z[index].to_le_bytes());
        data[out + 12] = points.intensity[index];
    }

    data
}

/// NEON with vget_lane extraction (alternative approach)
#[cfg(target_arch = "aarch64")]
#[inline(never)]
fn format_points_neon_wide(points: &Points, n_points: usize) -> Vec<u8> {
    let mut data = vec![0u8; 13 * n_points];
    let n_simd = n_points - n_points % 4;

    unsafe {
        let out_ptr = data.as_mut_ptr();

        for index in (0..n_simd).step_by(4) {
            // Load xyz vectors
            let x = vld1q_f32(points.x.as_ptr().add(index));
            let y = vld1q_f32(points.y.as_ptr().add(index));
            let z = vld1q_f32(points.z.as_ptr().add(index));

            // Extract lanes as f32 and write directly
            let x_u32 = vreinterpretq_u32_f32(x);
            let y_u32 = vreinterpretq_u32_f32(y);
            let z_u32 = vreinterpretq_u32_f32(z);

            // Extract all lanes at once
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

            // Write using unaligned stores
            let base = out_ptr.add(index * 13);

            // Point 0
            (base as *mut u32).write_unaligned(x0);
            (base.add(4) as *mut u32).write_unaligned(y0);
            (base.add(8) as *mut u32).write_unaligned(z0);
            *base.add(12) = points.intensity[index];

            // Point 1
            let out = base.add(13);
            (out as *mut u32).write_unaligned(x1);
            (out.add(4) as *mut u32).write_unaligned(y1);
            (out.add(8) as *mut u32).write_unaligned(z1);
            *out.add(12) = points.intensity[index + 1];

            // Point 2
            let out = base.add(26);
            (out as *mut u32).write_unaligned(x2);
            (out.add(4) as *mut u32).write_unaligned(y2);
            (out.add(8) as *mut u32).write_unaligned(z2);
            *out.add(12) = points.intensity[index + 2];

            // Point 3
            let out = base.add(39);
            (out as *mut u32).write_unaligned(x3);
            (out.add(4) as *mut u32).write_unaligned(y3);
            (out.add(8) as *mut u32).write_unaligned(z3);
            *out.add(12) = points.intensity[index + 3];
        }
    }

    // Handle remainder
    for index in n_simd..n_points {
        let out = index * 13;
        data[out..out + 4].copy_from_slice(&points.x[index].to_le_bytes());
        data[out + 4..out + 8].copy_from_slice(&points.y[index].to_le_bytes());
        data[out + 8..out + 12].copy_from_slice(&points.z[index].to_le_bytes());
        data[out + 12] = points.intensity[index];
    }

    data
}

fn benchmark<F>(name: &str, f: F, points: &Points, n_points: usize) -> Duration
where
    F: Fn(&Points, usize) -> Vec<u8>,
{
    // Warmup
    for _ in 0..10 {
        std::hint::black_box(f(points, n_points));
    }

    // Measure
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        std::hint::black_box(f(points, n_points));
    }
    let elapsed = start.elapsed();

    let per_iter = elapsed / ITERATIONS as u32;
    let throughput = (n_points as f64 * ITERATIONS as f64) / elapsed.as_secs_f64();

    println!(
        "{:25} {:>10.2} µs/frame  {:>10.1} Mpts/s",
        name,
        per_iter.as_secs_f64() * 1_000_000.0,
        throughput / 1_000_000.0
    );

    elapsed
}

fn main() {
    println!("Point Cloud Formatting Benchmark");
    println!("================================");
    println!("Points per frame: {}", N_POINTS);
    println!("Iterations: {}", ITERATIONS);
    #[cfg(target_arch = "aarch64")]
    println!("Architecture: aarch64 (NEON available)");
    #[cfg(target_arch = "x86_64")]
    println!("Architecture: x86_64");
    println!();

    let mut points = Points::new(N_POINTS);
    points.randomize();

    // Verify all implementations produce the same output
    let scalar_result = format_points_scalar(&points, N_POINTS);
    let unrolled_result = format_points_scalar_unrolled(&points, N_POINTS);
    let unsafe_result = format_points_unsafe(&points, N_POINTS);

    assert_eq!(scalar_result, unrolled_result, "Unrolled mismatch!");
    assert_eq!(scalar_result, unsafe_result, "Unsafe mismatch!");

    #[cfg(feature = "portable_simd")]
    {
        let simd_result = format_points_simd(&points, N_POINTS);
        assert_eq!(scalar_result, simd_result, "Portable SIMD mismatch!");
    }

    #[cfg(target_arch = "aarch64")]
    {
        let neon_result = format_points_neon(&points, N_POINTS);
        assert_eq!(scalar_result, neon_result, "NEON mismatch!");
        let neon_wide_result = format_points_neon_wide(&points, N_POINTS);
        assert_eq!(scalar_result, neon_wide_result, "NEON wide mismatch!");
    }

    println!("All implementations verified to produce identical output.\n");

    // Run benchmarks
    let scalar_time = benchmark("Scalar", format_points_scalar, &points, N_POINTS);
    let unrolled_time = benchmark(
        "Scalar (unrolled 4x)",
        format_points_scalar_unrolled,
        &points,
        N_POINTS,
    );
    let unsafe_time = benchmark("Unsafe pointers", format_points_unsafe, &points, N_POINTS);

    #[cfg(feature = "portable_simd")]
    let simd_time = benchmark("Portable SIMD", format_points_simd, &points, N_POINTS);

    #[cfg(target_arch = "aarch64")]
    let neon_time = benchmark("NEON (4-wide)", format_points_neon, &points, N_POINTS);

    #[cfg(target_arch = "aarch64")]
    let neon_wide_time = benchmark("NEON (extract)", format_points_neon_wide, &points, N_POINTS);

    println!();
    println!("Relative Performance (vs scalar):");
    println!("---------------------------------");
    println!("Scalar:              1.00x");
    println!(
        "Scalar (unrolled):   {:.2}x",
        scalar_time.as_secs_f64() / unrolled_time.as_secs_f64()
    );
    println!(
        "Unsafe pointers:     {:.2}x",
        scalar_time.as_secs_f64() / unsafe_time.as_secs_f64()
    );

    #[cfg(feature = "portable_simd")]
    println!(
        "Portable SIMD:       {:.2}x",
        scalar_time.as_secs_f64() / simd_time.as_secs_f64()
    );

    #[cfg(target_arch = "aarch64")]
    {
        println!(
            "NEON (4-wide):       {:.2}x",
            scalar_time.as_secs_f64() / neon_time.as_secs_f64()
        );
        println!(
            "NEON (extract):      {:.2}x",
            scalar_time.as_secs_f64() / neon_wide_time.as_secs_f64()
        );
    }
}
