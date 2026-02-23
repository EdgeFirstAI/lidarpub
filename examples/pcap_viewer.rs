// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! PCAP Replay Viewer - Replay captured LiDAR data through Rerun
//!
//! This example replays PCAP files and visualizes point clouds using Rerun.
//! Supports both Ouster and Robosense sensors.
//!
//! # Usage
//!
//! ```bash
//! # Robosense E1R - save to RRD file
//! cargo run --example pcap_viewer --features rerun -- \
//!     --sensor-type robosense \
//!     --pcap testdata/e1r_frames.pcap \
//!     --save robosense.rrd
//!
//! # Ouster OS1 - save to RRD file
//! cargo run --example pcap_viewer --features rerun -- \
//!     --sensor-type ouster \
//!     --pcap testdata/os1_frames.pcap \
//!     --sensor-info testdata/os1_sensor_info.json \
//!     --save ouster.rrd
//!
//! # View saved files
//! rerun robosense.rrd
//! rerun ouster.rrd
//! ```

use clap::Parser;
use edgefirst_lidarpub::{
    lidar::{LidarDriver, LidarFrame, SensorType},
    ouster::{OusterDriver, OusterLidarFrame, Parameters},
    packet_source::PacketSource,
    pcap_source::PcapSource,
    robosense::{RobosenseDriver, RobosenseLidarFrame},
};

#[derive(Parser, Debug)]
#[command(author, version, about = "PCAP replay viewer for LiDAR point clouds")]
struct Args {
    /// Sensor type
    #[arg(long, value_enum)]
    sensor_type: SensorType,

    /// Path to PCAP file
    #[arg(long)]
    pcap: String,

    /// Path to Ouster sensor_info.json (required for Ouster)
    #[arg(long)]
    sensor_info: Option<String>,

    /// UDP port filter (default: 6699 for Robosense, 7502 for Ouster)
    #[arg(long)]
    port: Option<u16>,

    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Initialize Rerun
    let (rec, _guard) = args.rerun.init("pcap_viewer")?;

    match args.sensor_type {
        SensorType::Robosense => run_robosense(&args, rec),
        SensorType::Ouster => run_ouster(&args, rec),
    }
}

fn run_robosense(
    args: &Args,
    rec: rerun::RecordingStream,
) -> Result<(), Box<dyn std::error::Error>> {
    let port = args.port.unwrap_or(6699);
    let mut source = PcapSource::from_file(&args.pcap, Some(port))?;
    let mut driver = RobosenseDriver::new();
    let mut frame = RobosenseLidarFrame::new();

    println!("Replaying {} ({} packets)", args.pcap, source.len());

    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();

    let mut buf = vec![0u8; 16 * 1024];
    let mut frame_count = 0u64;
    let mut total_points = 0usize;

    while source.has_more() {
        let len = rt.block_on(async { source.recv(&mut buf).await })?;

        match driver.process(&mut frame, &buf[..len]) {
            Ok(true) => {
                frame_count += 1;
                total_points += frame.len();

                let points = frame_to_rerun(&frame);
                let colors = intensities_to_colors(&frame);

                rec.log(
                    "lidar/points",
                    &rerun::Points3D::new(points).with_colors(colors),
                )?;

                println!(
                    "Frame {}: {} points (total: {})",
                    frame_count,
                    frame.len(),
                    total_points
                );
            }
            Ok(false) => {}
            Err(e) => {
                eprintln!("Packet error: {:?}", e);
            }
        }
    }

    println!(
        "Complete: {} frames, {} total points",
        frame_count, total_points
    );
    Ok(())
}

fn run_ouster(args: &Args, rec: rerun::RecordingStream) -> Result<(), Box<dyn std::error::Error>> {
    let sensor_info_path = args
        .sensor_info
        .as_ref()
        .ok_or("--sensor-info required for Ouster")?;

    let params = load_ouster_params(sensor_info_path)?;
    let rows = params.lidar_data_format.pixels_per_column;
    let cols = params.lidar_data_format.columns_per_frame;

    let port = args.port.unwrap_or(7502);
    let mut source = PcapSource::from_file(&args.pcap, Some(port))?;
    let mut driver = OusterDriver::new(&params)?;
    let mut frame = OusterLidarFrame::with_capacity(rows * cols);

    println!(
        "Replaying {} ({} packets, {}x{} frame)",
        args.pcap,
        source.len(),
        cols,
        rows
    );

    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();

    let mut buf = vec![0u8; 16 * 1024];
    let mut frame_count = 0u64;
    let mut total_points = 0usize;

    while source.has_more() {
        let len = rt.block_on(async { source.recv(&mut buf).await })?;

        match driver.process(&mut frame, &buf[..len]) {
            Ok(true) => {
                frame_count += 1;
                total_points += frame.len();

                let points = frame_to_rerun(&frame);
                let colors = intensities_to_colors(&frame);

                rec.log(
                    "lidar/points",
                    &rerun::Points3D::new(points).with_colors(colors),
                )?;

                println!(
                    "Frame {}: {} points (total: {})",
                    frame_count,
                    frame.len(),
                    total_points
                );
            }
            Ok(false) => {}
            Err(e) => {
                eprintln!("Packet error: {:?}", e);
            }
        }
    }

    println!(
        "Complete: {} frames, {} total points",
        frame_count, total_points
    );
    Ok(())
}

fn load_ouster_params(path: &str) -> Result<Parameters, Box<dyn std::error::Error>> {
    use edgefirst_lidarpub::ouster::{BeamIntrinsics, LidarDataFormat, SensorInfo};
    use serde::Deserialize;

    #[derive(Deserialize)]
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

    #[derive(Deserialize)]
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

    let sensor_info_str = std::fs::read_to_string(path)?;
    let partial: PartialSensorInfo = serde_json::from_str(&sensor_info_str)?;

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

    Ok(Parameters {
        sensor_info,
        lidar_data_format,
        beam_intrinsics,
    })
}

/// Convert LidarFrame to Rerun point format
fn frame_to_rerun<F: LidarFrame>(frame: &F) -> Vec<[f32; 3]> {
    let x = frame.x();
    let y = frame.y();
    let z = frame.z();
    (0..frame.len()).map(|i| [x[i], y[i], z[i]]).collect()
}

/// Convert intensities to Rerun colors (grayscale based on intensity)
fn intensities_to_colors<F: LidarFrame>(frame: &F) -> Vec<rerun::Color> {
    frame
        .intensity()
        .iter()
        .map(|&intensity| rerun::Color::from_rgb(intensity, intensity, intensity))
        .collect()
}
