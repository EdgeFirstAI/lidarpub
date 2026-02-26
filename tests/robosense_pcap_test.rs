// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Integration tests for Robosense E1R driver using PCAP data.
//!
//! These tests require the `pcap` feature and real sensor captures.
//! See `tests/data/robosense/README.md` for capture instructions.

#![cfg(feature = "pcap")]

use edgefirst_lidarpub::{
    cluster::{ClusterData, VoxelClusterData, cluster_, voxel_cluster},
    lidar::{LidarDriver, LidarFrame},
    packet_source::PacketSource,
    pcap_source::PcapSource,
    robosense::{RobosenseDriver, RobosenseLidarFrame},
};
use std::path::Path;

/// Path to Robosense E1R PCAP test data
const E1R_PCAP: &str = "testdata/e1r_frames.pcap";

/// Robosense E1R MSOP port
const MSOP_PORT: u16 = 6699;

/// Check if test data is available
fn test_data_available() -> bool {
    Path::new(E1R_PCAP).exists()
}

/// Skip test if data not available
macro_rules! require_test_data {
    () => {
        if !test_data_available() {
            eprintln!(
                "Skipping test: {} not found. See tests/data/robosense/README.md",
                E1R_PCAP
            );
            return;
        }
    };
}

#[tokio::test]
async fn test_e1r_frame_assembly() {
    require_test_data!();

    let mut source =
        PcapSource::from_file(E1R_PCAP, Some(MSOP_PORT)).expect("Failed to load PCAP file");

    let mut driver = RobosenseDriver::new();
    let mut frame = RobosenseLidarFrame::new();
    let mut buf = [0u8; 2048];

    let mut frames_completed = 0;

    while source.has_more() {
        let len = source.recv(&mut buf).await.expect("Failed to read packet");

        match driver.process(&mut frame, &buf[..len]) {
            Ok(true) => {
                frames_completed += 1;

                // Verify frame has reasonable point count for E1R (~26k points)
                assert!(
                    frame.len() > 10_000,
                    "Frame {} has too few points: {}",
                    frames_completed,
                    frame.len()
                );
                assert!(
                    frame.len() < 35_000,
                    "Frame {} has too many points: {}",
                    frames_completed,
                    frame.len()
                );

                // Verify coordinates are within E1R's 30m range
                for (i, &x) in frame.x().iter().enumerate() {
                    assert!(
                        x.abs() < 35.0,
                        "Point {} X coordinate out of range: {}",
                        i,
                        x
                    );
                }
                for (i, &y) in frame.y().iter().enumerate() {
                    assert!(
                        y.abs() < 35.0,
                        "Point {} Y coordinate out of range: {}",
                        i,
                        y
                    );
                }
                for (i, &z) in frame.z().iter().enumerate() {
                    assert!(
                        z.abs() < 35.0,
                        "Point {} Z coordinate out of range: {}",
                        i,
                        z
                    );
                }

                // Verify range values are reasonable (0 to 30m for E1R)
                for (i, &r) in frame.range().iter().enumerate() {
                    assert!(
                        (0.0..35.0).contains(&r),
                        "Point {} range out of bounds: {}",
                        i,
                        r
                    );
                }

                println!(
                    "Frame {}: {} points, timestamp {}",
                    frames_completed,
                    frame.len(),
                    frame.timestamp()
                );
            }
            Ok(false) => {
                // More packets needed
            }
            Err(e) => {
                panic!("Packet processing error: {:?}", e);
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
async fn test_e1r_double_buffer_pattern() {
    require_test_data!();

    let mut source =
        PcapSource::from_file(E1R_PCAP, Some(MSOP_PORT)).expect("Failed to load PCAP file");

    let mut driver = RobosenseDriver::new();

    // Two frames for buffer swapping pattern
    let mut frame_a = RobosenseLidarFrame::new();
    let mut frame_b = RobosenseLidarFrame::new();
    let mut using_a = true;

    let mut buf = [0u8; 2048];
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
            Err(e) => panic!("Error: {:?}", e),
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
async fn test_e1r_pcap_source_reset() {
    require_test_data!();

    let mut source =
        PcapSource::from_file(E1R_PCAP, Some(MSOP_PORT)).expect("Failed to load PCAP file");

    let initial_count = source.len();
    assert!(initial_count > 0, "PCAP should contain packets");

    // Consume some packets
    let mut buf = [0u8; 2048];
    for _ in 0..5.min(initial_count) {
        source.recv(&mut buf).await.expect("Failed to read packet");
    }

    // Reset and verify
    source.reset();
    assert_eq!(source.current_index(), 0);
    assert_eq!(source.remaining(), initial_count);
}

#[tokio::test]
async fn test_e1r_frame_timestamps() {
    require_test_data!();

    let mut source =
        PcapSource::from_file(E1R_PCAP, Some(MSOP_PORT)).expect("Failed to load PCAP file");

    let mut driver = RobosenseDriver::new();
    let mut frame = RobosenseLidarFrame::new();
    let mut buf = [0u8; 2048];

    let mut frames_with_timestamp = 0;

    while source.has_more() {
        let len = source.recv(&mut buf).await.expect("Failed to read packet");

        if let Ok(true) = driver.process(&mut frame, &buf[..len]) {
            // Verify timestamp is set (non-zero for real sensor data)
            if frame.timestamp() > 0 {
                frames_with_timestamp += 1;
            }
        }
    }

    assert!(
        frames_with_timestamp > 0,
        "Expected frames with valid timestamps"
    );
}

/// Test voxel and DBSCAN clustering on real E1R point cloud data.
///
/// Verifies that both algorithms handle realistic point distributions
/// (>20k points, wide spatial spread) without hanging or panicking,
/// and that they can be called repeatedly with state reuse.
#[tokio::test]
async fn test_e1r_clustering_real_data() {
    require_test_data!();

    let mut source =
        PcapSource::from_file(E1R_PCAP, Some(MSOP_PORT)).expect("Failed to load PCAP file");

    let mut driver = RobosenseDriver::new();
    let mut frame = RobosenseLidarFrame::new();
    let mut buf = [0u8; 2048];

    // Collect all complete frames
    let mut frames: Vec<(Vec<f32>, Vec<f32>, Vec<f32>)> = Vec::new();

    while source.has_more() {
        let len = source.recv(&mut buf).await.expect("Failed to read packet");

        if let Ok(true) = driver.process(&mut frame, &buf[..len]) {
            frames.push((
                frame.x().to_vec(),
                frame.y().to_vec(),
                frame.z().to_vec(),
            ));
        }
    }

    assert!(!frames.is_empty(), "Expected at least one frame from pcap");

    let eps_m = 0.256; // 256mm default
    let min_pts = 4;

    // Test voxel clustering on all frames with state reuse
    let mut voxel_data = VoxelClusterData::new(eps_m, min_pts);
    for (i, (x, y, z)) in frames.iter().enumerate() {
        let clusters = voxel_cluster(&mut voxel_data, x, y, z);
        assert_eq!(clusters.len(), x.len());

        let n_clustered = clusters.iter().filter(|&&c| c > 0).count();
        let max_id = clusters.iter().max().copied().unwrap_or(0);
        println!(
            "Voxel frame {}: {} points, {} clustered, {} clusters",
            i,
            x.len(),
            n_clustered,
            max_id
        );
        // Sanity: real data should produce at least some clusters
        assert!(max_id > 0, "Frame {} should have at least 1 cluster", i);
    }

    // Test DBSCAN (flat) clustering on all frames with state reuse
    let mut dbscan_data = ClusterData::new_flat(eps_m, min_pts);
    for (i, (x, y, z)) in frames.iter().enumerate() {
        let clusters = cluster_(&mut dbscan_data, x, y, z);
        assert_eq!(clusters.len(), x.len());

        let max_id = clusters.iter().max().copied().unwrap_or(0);
        println!(
            "DBSCAN frame {}: {} points, {} clusters",
            i,
            x.len(),
            max_id
        );
        assert!(max_id > 0, "Frame {} should have at least 1 cluster", i);
    }
}
