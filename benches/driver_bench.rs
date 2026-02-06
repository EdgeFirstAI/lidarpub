// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Benchmarks for LiDAR driver packet parsing and point cloud generation.
//!
//! Measures:
//! - Protocol parsing performance (packet decoding)
//! - Frame assembly and point cloud generation
//!
//! Run with: cargo bench --bench driver_bench --features pcap
//!
//! For on-target profiling, cross-compile and run:
//!   cargo bench --bench driver_bench --features pcap --target
//! aarch64-unknown-linux-gnu

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use std::path::Path;

// Re-use the library's types
use edgefirst_lidarpub::{
    LidarDriver, LidarFrame, LidarFrameWriter,
    ouster::{OusterDriver, OusterLidarFrame, Parameters},
    packet_source::PacketSource,
    pcap_source::PcapSource,
    robosense::{RobosenseDriver, RobosenseLidarFrame},
};

/// Robosense E1R test data paths
const E1R_PCAP: &str = "testdata/e1r_frames.pcap";
const ROBOSENSE_MSOP_PORT: u16 = 6699;

/// Ouster OS1 test data paths
const OS1_PCAP: &str = "testdata/os1_frames.pcap";
const OS1_SENSOR_INFO: &str = "testdata/os1_sensor_info.json";
const OUSTER_LIDAR_PORT: u16 = 7502;

/// Load all packets from a PCAP file into memory for benchmarking.
fn load_packets(path: &str, port: Option<u16>) -> Vec<Vec<u8>> {
    let source = PcapSource::from_file(path, port).expect("Failed to load PCAP");
    let mut packets = Vec::with_capacity(source.len());

    // Create a runtime for the async recv
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();

    let mut source = source;
    while source.has_more() {
        let mut buf = vec![0u8; 16 * 1024];
        let len = rt
            .block_on(async { source.recv(&mut buf).await })
            .expect("Failed to read packet");
        buf.truncate(len);
        packets.push(buf);
    }

    packets
}

/// Load Ouster parameters from sensor_info.json
fn load_ouster_params() -> Parameters {
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

    let sensor_info_str =
        std::fs::read_to_string(OS1_SENSOR_INFO).expect("Failed to read sensor_info.json");
    let partial: PartialSensorInfo =
        serde_json::from_str(&sensor_info_str).expect("Failed to parse sensor_info.json");

    let lidar_mode = partial.lidar_mode.as_deref().unwrap_or("1024x10");
    let cols: usize = lidar_mode
        .split('x')
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1024);

    let rows = partial
        .beam_altitude_angles
        .as_ref()
        .map(|v| v.len())
        .unwrap_or(64);

    use edgefirst_lidarpub::ouster::{BeamIntrinsics, LidarDataFormat, SensorInfo};

    let beam_intrinsics = BeamIntrinsics {
        beam_altitude_angles: partial
            .beam_altitude_angles
            .unwrap_or_else(|| (0..rows).map(|i| (i as f32 - 31.5) * 0.7).collect()),
        beam_azimuth_angles: partial
            .beam_azimuth_angles
            .unwrap_or_else(|| vec![0.0; rows]),
        beam_to_lidar_transform: partial.beam_to_lidar_transform.unwrap_or_else(|| {
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

    Parameters {
        sensor_info,
        lidar_data_format,
        beam_intrinsics,
    }
}

/// Benchmark Robosense E1R driver
fn bench_robosense(c: &mut Criterion) {
    if !Path::new(E1R_PCAP).exists() {
        eprintln!("Skipping Robosense benchmarks: {} not found", E1R_PCAP);
        return;
    }

    let packets = load_packets(E1R_PCAP, Some(ROBOSENSE_MSOP_PORT));
    let total_bytes: usize = packets.iter().map(|p| p.len()).sum();

    println!(
        "Robosense: {} packets, {} bytes total",
        packets.len(),
        total_bytes
    );

    let mut group = c.benchmark_group("robosense_e1r");

    // Set throughput based on packet data
    group.throughput(Throughput::Bytes(total_bytes as u64));

    // Benchmark: Process all packets (full frame assembly + point generation)
    group.bench_function("full_pipeline", |b| {
        b.iter_with_setup(
            || {
                let driver = RobosenseDriver::new();
                let frame = RobosenseLidarFrame::new();
                (driver, frame, packets.clone())
            },
            |(mut driver, mut frame, packets)| {
                let mut frames_completed = 0;
                for packet in &packets {
                    if let Ok(true) = driver.process(&mut frame, packet) {
                        frames_completed += 1;
                        std::hint::black_box(frame.len());
                    }
                }
                frames_completed
            },
        );
    });

    // Benchmark: Single packet processing (amortized)
    group.throughput(Throughput::Elements(packets.len() as u64));
    group.bench_function("per_packet", |b| {
        let mut driver = RobosenseDriver::new();
        let mut frame = RobosenseLidarFrame::new();

        b.iter(|| {
            for packet in &packets {
                let _ = driver.process(&mut frame, packet);
            }
        });
    });

    group.finish();
}

/// Benchmark Ouster OS1 driver
fn bench_ouster(c: &mut Criterion) {
    if !Path::new(OS1_PCAP).exists() || !Path::new(OS1_SENSOR_INFO).exists() {
        eprintln!(
            "Skipping Ouster benchmarks: {} or {} not found",
            OS1_PCAP, OS1_SENSOR_INFO
        );
        return;
    }

    let packets = load_packets(OS1_PCAP, Some(OUSTER_LIDAR_PORT));
    let total_bytes: usize = packets.iter().map(|p| p.len()).sum();
    let params = load_ouster_params();

    let rows = params.lidar_data_format.pixels_per_column;
    let cols = params.lidar_data_format.columns_per_frame;
    let capacity = rows * cols;

    println!(
        "Ouster: {} packets, {} bytes total, {}x{} frame",
        packets.len(),
        total_bytes,
        cols,
        rows
    );

    let mut group = c.benchmark_group("ouster_os1");

    // Set throughput based on packet data
    group.throughput(Throughput::Bytes(total_bytes as u64));

    // Benchmark: Process all packets (full frame assembly + point generation)
    group.bench_function("full_pipeline", |b| {
        b.iter_with_setup(
            || {
                let driver = OusterDriver::new(&params).expect("Failed to create driver");
                let frame = OusterLidarFrame::with_capacity(capacity);
                (driver, frame, packets.clone())
            },
            |(mut driver, mut frame, packets)| {
                let mut frames_completed = 0;
                for packet in &packets {
                    if let Ok(true) = driver.process(&mut frame, packet) {
                        frames_completed += 1;
                        std::hint::black_box(frame.len());
                    }
                }
                frames_completed
            },
        );
    });

    // Benchmark: Single packet processing (amortized)
    group.throughput(Throughput::Elements(packets.len() as u64));
    group.bench_function("per_packet", |b| {
        let mut driver = OusterDriver::new(&params).expect("Failed to create driver");
        let mut frame = OusterLidarFrame::with_capacity(capacity);

        b.iter(|| {
            for packet in &packets {
                let _ = driver.process(&mut frame, packet);
            }
        });
    });

    group.finish();
}

/// Benchmark point cloud size scaling
fn bench_frame_sizes(c: &mut Criterion) {
    let mut group = c.benchmark_group("frame_scaling");

    // Test different frame sizes to understand scaling behavior
    for size in [10_000, 25_000, 50_000, 100_000, 131_072] {
        group.throughput(Throughput::Elements(size as u64));

        // Robosense frame operations
        group.bench_with_input(
            BenchmarkId::new("robosense_frame_reset", size),
            &size,
            |b, &size| {
                let mut frame = RobosenseLidarFrame::with_capacity(size);
                // Fill frame
                for i in 0..size {
                    frame.push(i as f32, i as f32, i as f32, (i % 256) as u8, i as f32);
                }

                b.iter(|| {
                    frame.reset();
                    std::hint::black_box(frame.len());
                });
            },
        );

        // Ouster frame operations
        group.bench_with_input(
            BenchmarkId::new("ouster_frame_reset", size),
            &size,
            |b, &size| {
                let mut frame = OusterLidarFrame::with_capacity(size);
                // Fill frame
                for i in 0..size {
                    frame.push(i as f32, i as f32, i as f32, (i % 256) as u8, i as f32);
                }

                b.iter(|| {
                    frame.reset();
                    std::hint::black_box(frame.len());
                });
            },
        );

        // Frame push performance
        group.bench_with_input(
            BenchmarkId::new("robosense_frame_push", size),
            &size,
            |b, &size| {
                let mut frame = RobosenseLidarFrame::with_capacity(size);

                b.iter(|| {
                    frame.reset();
                    for i in 0..size {
                        frame.push(
                            i as f32 * 0.01,
                            i as f32 * 0.02,
                            i as f32 * 0.005,
                            (i % 256) as u8,
                            ((i as f32 * 0.01).powi(2) + (i as f32 * 0.02).powi(2)).sqrt(),
                        );
                    }
                    std::hint::black_box(frame.len());
                });
            },
        );
    }

    group.finish();
}

/// Compare drivers head-to-head with similar workloads
fn bench_comparison(c: &mut Criterion) {
    if !Path::new(E1R_PCAP).exists() || !Path::new(OS1_PCAP).exists() {
        eprintln!("Skipping comparison benchmarks: test data not found");
        return;
    }

    let e1r_packets = load_packets(E1R_PCAP, Some(ROBOSENSE_MSOP_PORT));
    let os1_packets = load_packets(OS1_PCAP, Some(OUSTER_LIDAR_PORT));
    let ouster_params = load_ouster_params();

    let mut group = c.benchmark_group("driver_comparison");

    // Normalize by points per second
    // Count frames and points
    let (e1r_frames, e1r_points) = {
        let mut driver = RobosenseDriver::new();
        let mut frame = RobosenseLidarFrame::new();
        let mut frames = 0;
        let mut points = 0;
        for packet in &e1r_packets {
            if let Ok(true) = driver.process(&mut frame, packet) {
                frames += 1;
                points += frame.len();
            }
        }
        (frames, points)
    };

    let (os1_frames, os1_points) = {
        let rows = ouster_params.lidar_data_format.pixels_per_column;
        let cols = ouster_params.lidar_data_format.columns_per_frame;
        let mut driver = OusterDriver::new(&ouster_params).unwrap();
        let mut frame = OusterLidarFrame::with_capacity(rows * cols);
        let mut frames = 0;
        let mut points = 0;
        for packet in &os1_packets {
            if let Ok(true) = driver.process(&mut frame, packet) {
                frames += 1;
                points += frame.len();
            }
        }
        (frames, points)
    };

    println!(
        "Robosense E1R: {} frames, {} total points",
        e1r_frames, e1r_points
    );
    println!(
        "Ouster OS1: {} frames, {} total points",
        os1_frames, os1_points
    );

    // Benchmark throughput in points per second
    group.throughput(Throughput::Elements(e1r_points as u64));
    group.bench_function("robosense_points_throughput", |b| {
        let mut driver = RobosenseDriver::new();
        let mut frame = RobosenseLidarFrame::new();

        b.iter(|| {
            for packet in &e1r_packets {
                let _ = driver.process(&mut frame, packet);
            }
        });
    });

    group.throughput(Throughput::Elements(os1_points as u64));
    group.bench_function("ouster_points_throughput", |b| {
        let rows = ouster_params.lidar_data_format.pixels_per_column;
        let cols = ouster_params.lidar_data_format.columns_per_frame;
        let mut driver = OusterDriver::new(&ouster_params).unwrap();
        let mut frame = OusterLidarFrame::with_capacity(rows * cols);

        b.iter(|| {
            for packet in &os1_packets {
                let _ = driver.process(&mut frame, packet);
            }
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_robosense,
    bench_ouster,
    bench_frame_sizes,
    bench_comparison,
);
criterion_main!(benches);
