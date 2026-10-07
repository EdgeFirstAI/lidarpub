// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.

//! Ouster XYZ projection tests against the reference formula from the Ouster
//! sensor documentation, using real metadata from an OS1-64 (FW 2.5.3, flat
//! metadata) and an OS1-128 Rev7 (FW 3.2, nested metadata).

use edgefirst_lidarpub::{
    OusterLidarFrame,
    lidar::LidarFrame,
    ouster::{BeamIntrinsics, FrameBuilder, LidarDataFormat, Parameters, SensorInfo},
};
use ndarray::Array2;
use serde_json::Value;

const OS1_64_FW25: &str = "testdata/os1_sensor_info.json";
const OS1_128_REV7_FW32: &str = "testdata/os1_128_rev7_metadata.json";

/// Load sensor parameters from either the nested `/api/v1/sensor/metadata`
/// format (FW 3.x) or the flat legacy format (FW 2.x).
pub fn load_params(path: &str) -> Parameters {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let m: Value = serde_json::from_str(&text).expect("metadata JSON");

    if m.get("beam_intrinsics").is_some() {
        return Parameters {
            sensor_info: serde_json::from_value(m["sensor_info"].clone()).unwrap(),
            lidar_data_format: serde_json::from_value(m["lidar_data_format"].clone()).unwrap(),
            beam_intrinsics: serde_json::from_value(m["beam_intrinsics"].clone()).unwrap(),
        };
    }

    let df = &m["data_format"];
    Parameters {
        sensor_info: SensorInfo {
            status: m["status"].as_str().unwrap().to_string(),
            build_rev: m["build_rev"].as_str().unwrap().to_string(),
            prod_sn: m["prod_sn"].as_str().unwrap().to_string(),
            prod_pn: m["prod_pn"].as_str().unwrap().to_string(),
            prod_line: m["prod_line"].as_str().unwrap().to_string(),
        },
        lidar_data_format: LidarDataFormat {
            udp_profile_lidar: df["udp_profile_lidar"].as_str().unwrap().to_string(),
            udp_profile_imu: df["udp_profile_imu"].as_str().unwrap().to_string(),
            columns_per_packet: df["columns_per_packet"].as_u64().unwrap() as usize,
            columns_per_frame: df["columns_per_frame"].as_u64().unwrap() as usize,
            pixels_per_column: df["pixels_per_column"].as_u64().unwrap() as usize,
            column_window: serde_json::from_value(df["column_window"].clone()).unwrap(),
            pixel_shift_by_row: serde_json::from_value(df["pixel_shift_by_row"].clone()).unwrap(),
        },
        beam_intrinsics: BeamIntrinsics {
            beam_azimuth_angles: serde_json::from_value(m["beam_azimuth_angles"].clone()).unwrap(),
            beam_altitude_angles: serde_json::from_value(m["beam_altitude_angles"].clone())
                .unwrap(),
            beam_to_lidar_transform: serde_json::from_value(m["beam_to_lidar_transform"].clone())
                .unwrap(),
        },
    }
}

/// Reference range-to-XYZ projection in the lidar frame, in metres, for a
/// pixel at staggered column `measurement_id` with raw RNG15 range `d`.
fn reference_xyz(params: &Parameters, row: usize, measurement_id: usize, d: u16) -> [f64; 3] {
    use std::f64::consts::PI;
    let b2l = &params.beam_intrinsics.beam_to_lidar_transform;
    let bx = b2l[3] as f64;
    let bz = b2l[11] as f64;
    let n = (bx * bx + bz * bz).sqrt();
    let w = params.lidar_data_format.columns_per_frame as f64;

    let theta_e = 2.0 * PI * (1.0 - measurement_id as f64 / w);
    let theta_a = -2.0 * PI * params.beam_intrinsics.beam_azimuth_angles[row] as f64 / 360.0;
    let phi = 2.0 * PI * params.beam_intrinsics.beam_altitude_angles[row] as f64 / 360.0;
    let r = d as f64 * 8.0 - n;

    [
        (r * (theta_e + theta_a).cos() * phi.cos() + bx * theta_e.cos()) * 1e-3,
        (r * (theta_e + theta_a).sin() * phi.cos() + bx * theta_e.sin()) * 1e-3,
        (r * phi.sin() + bz) * 1e-3,
    ]
}

/// Synthetic depth image: varied ranges up to about 24 m, with a regular
/// pattern of no-return pixels and a few ranges inside the beam offset.
fn synthetic_depth(rows: usize, cols: usize) -> Array2<u16> {
    Array2::from_shape_fn((rows, cols), |(row, col)| {
        if (row * 7 + col) % 11 == 0 {
            0
        } else if (row + col * 3) % 97 == 0 {
            1
        } else {
            100 + ((row * 131 + col * 17) % 3000) as u16
        }
    })
}

fn check_against_reference(path: &str) {
    let params = load_params(path);
    let rows = params.lidar_data_format.pixels_per_column;
    let cols = params.lidar_data_format.columns_per_frame;
    let window_start = params.lidar_data_format.column_window[0];
    let shifts = &params.lidar_data_format.pixel_shift_by_row;

    let depth = synthetic_depth(rows, cols);
    let reflect = Array2::<u8>::from_elem((rows, cols), 7);

    let mut builder = FrameBuilder::new(&params);
    let mut frame = OusterLidarFrame::with_capacity(rows * cols);
    builder.update_fused(&depth, &reflect, &mut frame);

    let crop = builder.crop;
    let b2l = &params.beam_intrinsics.beam_to_lidar_transform;
    let n = ((b2l[3] * b2l[3] + b2l[11] * b2l[11]) as f64).sqrt();

    let mut expected = Vec::new();
    for (row, &shift) in shifts.iter().enumerate() {
        for col in crop.0..crop.1 {
            let measurement_id = (col as isize - shift as isize) as usize + window_start;
            let d = depth[[row, measurement_id]];
            if d == 0 || d as f64 * 8.0 <= n {
                continue;
            }
            expected.push(reference_xyz(&params, row, measurement_id, d));
        }
    }

    assert_eq!(
        frame.len(),
        expected.len(),
        "{path}: projected point count differs from valid pixel count"
    );

    let mut max_err = 0.0f64;
    for (i, e) in expected.iter().enumerate() {
        let dx = frame.x()[i] as f64 - e[0];
        let dy = frame.y()[i] as f64 - e[1];
        let dz = frame.z()[i] as f64 - e[2];
        max_err = max_err.max((dx * dx + dy * dy + dz * dz).sqrt());
    }
    assert!(
        max_err < 0.5e-3,
        "{path}: max XYZ error {:.3} mm exceeds 0.5 mm",
        max_err * 1e3
    );
}

#[test]
fn os1_64_fw25_xyz_matches_reference() {
    check_against_reference(OS1_64_FW25);
}

#[test]
fn os1_128_rev7_fw32_xyz_matches_reference() {
    check_against_reference(OS1_128_REV7_FW32);
}

/// The crop removes only the columns made incomplete by the pixel shift; the
/// inclusive column window must not lose a further column.
#[test]
fn crop_spans_full_inclusive_window() {
    for path in [OS1_64_FW25, OS1_128_REV7_FW32] {
        let params = load_params(path);
        let fmt = &params.lidar_data_format;
        let width = fmt.column_window[1] - fmt.column_window[0] + 1;
        let max_shift = *fmt.pixel_shift_by_row.iter().max().unwrap();
        let min_shift = *fmt.pixel_shift_by_row.iter().min().unwrap();
        let builder = FrameBuilder::new(&params);
        assert_eq!(
            builder.crop.1 - builder.crop.0,
            width - max_shift as usize - min_shift.unsigned_abs() as usize,
            "{path}"
        );
    }
}

/// A frame without returns produces no points instead of a ring at the beam
/// origin.
#[test]
fn no_return_pixels_are_dropped() {
    for path in [OS1_64_FW25, OS1_128_REV7_FW32] {
        let params = load_params(path);
        let rows = params.lidar_data_format.pixels_per_column;
        let cols = params.lidar_data_format.columns_per_frame;
        let mut builder = FrameBuilder::new(&params);
        let mut frame = OusterLidarFrame::with_capacity(rows * cols);
        builder.update_fused(
            &Array2::zeros((rows, cols)),
            &Array2::zeros((rows, cols)),
            &mut frame,
        );
        assert_eq!(frame.len(), 0, "{path}");
    }
}

#[cfg(feature = "pcap")]
mod pcap {
    use super::*;
    use edgefirst_lidarpub::{
        lidar::LidarDriver, ouster::OusterDriver, packet_source::PacketSource,
        pcap_source::PcapSource,
    };

    const OS1_64_FW25_PCAP: &str = "testdata/os1_frames.pcap";
    const OS1_128_REV7_FW32_PCAP: &str = "testdata/os1_128_rev7_frames.pcap";

    /// Decode every complete frame in a capture, calling `check` on each.
    async fn for_each_frame(pcap: &str, metadata: &str, mut check: impl FnMut(&OusterLidarFrame)) {
        let params = load_params(metadata);
        let rows = params.lidar_data_format.pixels_per_column;
        let cols = params.lidar_data_format.columns_per_frame;
        let mut driver = OusterDriver::new(&params).expect("driver");
        let mut frame = OusterLidarFrame::with_capacity(rows * cols);
        let mut source = PcapSource::from_file(pcap, Some(7502)).expect("pcap");
        let mut buf = vec![0u8; 64 * 1024];
        while source.has_more() {
            let len = source.recv(&mut buf).await.expect("packet");
            if let Ok(true) = driver.process(&mut frame, &buf[..len]) {
                check(&frame);
            }
        }
    }

    async fn check_real_frames(pcap: &str, metadata: &str) {
        let mut frames = 0;
        for_each_frame(pcap, metadata, |frame| {
            frames += 1;
            assert!(frame.len() > 0, "{pcap}: empty frame");
            for i in 0..frame.len() {
                let (x, y, z) = (frame.x()[i], frame.y()[i], frame.z()[i]);
                let norm = (x * x + y * y + z * z).sqrt();
                assert!(
                    norm > 0.1,
                    "{pcap}: point {i} at {norm:.4} m sits at the beam origin"
                );
                assert!(frame.range()[i] > 0.0, "{pcap}: point {i} has no range");
            }
        })
        .await;
        assert!(
            frames >= 2,
            "{pcap}: expected at least 2 frames, got {frames}"
        );
    }

    #[tokio::test]
    async fn os1_64_fw25_capture_has_no_origin_points() {
        check_real_frames(OS1_64_FW25_PCAP, OS1_64_FW25).await;
    }

    #[tokio::test]
    async fn os1_128_rev7_fw32_capture_has_no_origin_points() {
        check_real_frames(OS1_128_REV7_FW32_PCAP, OS1_128_REV7_FW32).await;
    }

    /// Regenerate `testdata/os1_frame{0,3}.pcd` from the OS1-64 capture,
    /// numbering frames from the first complete one (the capture starts
    /// mid-rotation, so the first decoded frame is partial).
    /// Run with `cargo test --features pcap --test ouster_xyz_test -- --ignored --nocapture`.
    #[tokio::test]
    #[ignore]
    async fn regenerate_os1_pcd_testdata() {
        let mut decoded = 0;
        for_each_frame(OS1_64_FW25_PCAP, OS1_64_FW25, |frame| {
            decoded += 1;
            let index = decoded - 2;
            if decoded >= 2 && (index == 0 || index == 3) {
                let path = format!("testdata/os1_frame{index}.pcd");
                write_pcd(&path, frame).expect("write PCD");
                println!("{path}: {} points", frame.len());
            }
        })
        .await;
    }

    fn write_pcd(path: &str, frame: &OusterLidarFrame) -> std::io::Result<()> {
        let n = frame.len();
        let mut out = format!(
            "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n\
             SIZE 4 4 4 1\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\n\
             VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
        )
        .into_bytes();
        for i in 0..n {
            out.extend_from_slice(&frame.x()[i].to_le_bytes());
            out.extend_from_slice(&frame.y()[i].to_le_bytes());
            out.extend_from_slice(&frame.z()[i].to_le_bytes());
            out.push(frame.intensity()[i]);
        }
        std::fs::write(path, out)
    }
}
