// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Integration tests for Ouster LiDAR driver using PCAP data.
//!
//! These tests require the `pcap` feature, real sensor captures, and
//! sensor metadata JSON files.
//! See `tests/data/ouster/README.md` for capture instructions.

#![cfg(feature = "pcap")]

use edgefirst_lidarpub::{
    lidar::{LidarDriver, LidarFrame},
    ouster::{
        BeamIntrinsics, LidarDataFormat, OusterDriver, OusterLidarFrame, Parameters, SensorInfo,
    },
    packet_source::PacketSource,
    pcap_source::PcapSource,
};
use std::path::Path;

/// Path to Ouster PCAP test data
const OUSTER_PCAP: &str = "testdata/os1_frames.pcap";

/// Path to Ouster sensor metadata
const SENSOR_INFO: &str = "testdata/os1_sensor_info.json";

/// Ouster lidar data port
const LIDAR_PORT: u16 = 7502;

/// Check if test data is available
fn test_data_available() -> bool {
    Path::new(OUSTER_PCAP).exists() && Path::new(SENSOR_INFO).exists()
}

/// Skip test if data not available
macro_rules! require_test_data {
    () => {
        if !test_data_available() {
            eprintln!(
                "Skipping test: {} or {} not found. See tests/data/ouster/README.md",
                OUSTER_PCAP, SENSOR_INFO
            );
            return;
        }
    };
}

/// Load Ouster parameters from sensor_info.json
fn load_test_params() -> Result<Parameters, Box<dyn std::error::Error>> {
    // For now, we create minimal parameters for testing
    // In a real scenario, this would parse the sensor_info.json file

    let sensor_info_str = std::fs::read_to_string(SENSOR_INFO)?;

    // Try to parse the full sensor_info.json if it contains all fields
    // Otherwise, create minimal test parameters
    #[derive(serde::Deserialize)]
    struct PartialSensorInfo {
        #[serde(default)]
        status: Option<String>,
        #[serde(default)]
        build_rev: Option<String>,
        #[serde(default)]
        prod_sn: Option<String>,
        #[serde(default)]
        prod_pn: Option<String>,
        #[serde(default)]
        prod_line: Option<String>,
        #[serde(default)]
        beam_altitude_angles: Option<Vec<f32>>,
        #[serde(default)]
        beam_azimuth_angles: Option<Vec<f32>>,
        #[serde(default)]
        beam_to_lidar_transform: Option<Vec<f32>>,
        #[serde(default)]
        lidar_mode: Option<String>,
        #[serde(default)]
        lidar_origin_to_beam_origin_mm: Option<f32>,
        #[serde(default)]
        data_format: Option<DataFormatInfo>,
    }

    #[derive(serde::Deserialize)]
    struct DataFormatInfo {
        #[serde(default)]
        columns_per_packet: Option<usize>,
        #[serde(default)]
        columns_per_frame: Option<usize>,
        #[serde(default)]
        pixels_per_column: Option<usize>,
        #[serde(default)]
        column_window: Option<[usize; 2]>,
        #[serde(default)]
        pixel_shift_by_row: Option<Vec<i16>>,
        #[serde(default)]
        udp_profile_lidar: Option<String>,
        #[serde(default)]
        udp_profile_imu: Option<String>,
    }

    let partial: PartialSensorInfo = serde_json::from_str(&sensor_info_str)?;

    // Parse lidar mode to determine dimensions
    let lidar_mode = partial.lidar_mode.as_deref().unwrap_or("1024x10");
    let cols: usize = lidar_mode
        .split('x')
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1024);

    // Determine rows from beam angles or default
    let rows = partial
        .beam_altitude_angles
        .as_ref()
        .map(|v| v.len())
        .unwrap_or(64);

    // Build parameters
    let beam_intrinsics = BeamIntrinsics {
        beam_altitude_angles: partial.beam_altitude_angles.unwrap_or_else(|| {
            // Default beam angles for OS1-64
            (0..rows).map(|i| (i as f32 - 31.5) * 0.7).collect()
        }),
        beam_azimuth_angles: partial
            .beam_azimuth_angles
            .unwrap_or_else(|| vec![0.0; rows]),
        beam_to_lidar_transform: partial.beam_to_lidar_transform.unwrap_or_else(|| {
            // Identity matrix with beam origin offset
            vec![
                1.0,
                0.0,
                0.0,
                partial.lidar_origin_to_beam_origin_mm.unwrap_or(27.67),
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ]
        }),
    };

    let data_format = partial.data_format.unwrap_or(DataFormatInfo {
        columns_per_packet: None,
        columns_per_frame: None,
        pixels_per_column: None,
        column_window: None,
        pixel_shift_by_row: None,
        udp_profile_lidar: None,
        udp_profile_imu: None,
    });

    let lidar_data_format = LidarDataFormat {
        udp_profile_lidar: data_format
            .udp_profile_lidar
            .unwrap_or_else(|| "RNG15_RFL8_NIR8".to_string()),
        udp_profile_imu: data_format
            .udp_profile_imu
            .unwrap_or_else(|| "LEGACY".to_string()),
        columns_per_packet: data_format.columns_per_packet.unwrap_or(16),
        columns_per_frame: data_format.columns_per_frame.unwrap_or(cols),
        pixels_per_column: data_format.pixels_per_column.unwrap_or(rows),
        column_window: data_format.column_window.unwrap_or([0, cols]),
        pixel_shift_by_row: data_format
            .pixel_shift_by_row
            .unwrap_or_else(|| vec![0; rows]),
    };

    let sensor_info = SensorInfo {
        status: partial.status.unwrap_or_else(|| "RUNNING".to_string()),
        build_rev: partial.build_rev.unwrap_or_else(|| "unknown".to_string()),
        prod_sn: partial.prod_sn.unwrap_or_else(|| "unknown".to_string()),
        prod_pn: partial.prod_pn.unwrap_or_else(|| "unknown".to_string()),
        prod_line: partial.prod_line.unwrap_or_else(|| "OS1".to_string()),
    };

    Ok(Parameters {
        sensor_info,
        lidar_data_format,
        beam_intrinsics,
    })
}

#[tokio::test]
async fn test_ouster_frame_assembly() {
    require_test_data!();

    let params = load_test_params().expect("Failed to load sensor parameters");
    let mut driver = OusterDriver::new(&params).expect("Failed to create driver");

    let rows = params.lidar_data_format.pixels_per_column;
    let cols = params.lidar_data_format.columns_per_frame;
    let capacity = rows * cols;

    let mut frame = OusterLidarFrame::with_capacity(capacity);

    let mut source =
        PcapSource::from_file(OUSTER_PCAP, Some(LIDAR_PORT)).expect("Failed to load PCAP file");

    let mut buf = [0u8; 16 * 1024]; // Ouster packets can be larger
    let mut frames_completed = 0;

    while source.has_more() {
        let len = source.recv(&mut buf).await.expect("Failed to read packet");

        match driver.process(&mut frame, &buf[..len]) {
            Ok(true) => {
                frames_completed += 1;

                // Verify frame has reasonable point count
                assert!(frame.len() > 0, "Frame {} is empty", frames_completed);

                // For Ouster, max points = rows * cols
                assert!(
                    frame.len() <= capacity,
                    "Frame {} has too many points: {} > {}",
                    frames_completed,
                    frame.len(),
                    capacity
                );

                // Verify coordinates are within reasonable range (Ouster: ~120m max)
                for (i, &x) in frame.x().iter().enumerate() {
                    assert!(
                        x.abs() < 150.0,
                        "Point {} X coordinate out of range: {}",
                        i,
                        x
                    );
                }

                // Verify range values are reasonable
                for (i, &r) in frame.range().iter().enumerate() {
                    assert!(
                        (0.0..150.0).contains(&r),
                        "Point {} range out of bounds: {}",
                        i,
                        r
                    );
                }

                println!(
                    "Frame {}: {} points ({}x{}), timestamp {}",
                    frames_completed,
                    frame.len(),
                    rows,
                    cols,
                    frame.timestamp()
                );
            }
            Ok(false) => {
                // More packets needed
            }
            Err(e) => {
                // Some packet errors are expected (e.g., IMU packets on wrong port)
                eprintln!("Packet error (may be expected): {:?}", e);
            }
        }
    }

    assert!(
        frames_completed >= 1,
        "Expected at least 1 complete frame, got {}",
        frames_completed
    );
    println!("Total frames completed: {}", frames_completed);
}

#[tokio::test]
async fn test_ouster_double_buffer_pattern() {
    require_test_data!();

    let params = load_test_params().expect("Failed to load sensor parameters");
    let mut driver = OusterDriver::new(&params).expect("Failed to create driver");

    let rows = params.lidar_data_format.pixels_per_column;
    let cols = params.lidar_data_format.columns_per_frame;
    let capacity = rows * cols;

    // Two frames for buffer swapping pattern
    let mut frame_a = OusterLidarFrame::with_capacity(capacity);
    let mut frame_b = OusterLidarFrame::with_capacity(capacity);
    let mut using_a = true;

    let mut source =
        PcapSource::from_file(OUSTER_PCAP, Some(LIDAR_PORT)).expect("Failed to load PCAP file");

    let mut buf = [0u8; 16 * 1024];
    let mut total_points = 0;
    let mut frames_completed = 0;

    while source.has_more() {
        let len = source.recv(&mut buf).await.expect("Failed to read packet");

        let frame = if using_a { &mut frame_a } else { &mut frame_b };

        match driver.process(frame, &buf[..len]) {
            Ok(true) => {
                frames_completed += 1;
                total_points += frame.len();

                // Swap to other buffer for next frame
                using_a = !using_a;

                // The completed frame is still accessible
                let completed = if using_a { &frame_b } else { &frame_a };
                assert!(!completed.is_empty());
            }
            Ok(false) => {}
            Err(_) => {
                // Ignore packet errors in this test
            }
        }
    }

    println!(
        "Double buffer test: {} frames, {} total points",
        frames_completed, total_points
    );

    assert!(frames_completed >= 1);
    assert!(total_points > 0);
}

#[tokio::test]
async fn test_ouster_pcap_port_filtering() {
    require_test_data!();

    // Load without port filter
    let source_all = PcapSource::from_file(OUSTER_PCAP, None).expect("Failed to load PCAP file");

    // Load with lidar port filter
    let source_lidar =
        PcapSource::from_file(OUSTER_PCAP, Some(LIDAR_PORT)).expect("Failed to load PCAP file");

    println!(
        "Total packets: {}, Lidar packets: {}",
        source_all.len(),
        source_lidar.len()
    );

    // Lidar-filtered should have fewer or equal packets
    assert!(source_lidar.len() <= source_all.len());

    // Should have at least some lidar packets
    assert!(
        !source_lidar.is_empty(),
        "No lidar packets found on port {}",
        LIDAR_PORT
    );
}
