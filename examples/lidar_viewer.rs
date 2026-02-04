// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Standalone LiDAR Viewer - Direct UDP connection (no Zenoh)
//!
//! This example connects directly to a LiDAR sensor and visualizes the point
//! cloud using Rerun. Unlike the main binary, this doesn't use Zenoh and
//! connects directly to the sensor's UDP stream.
//!
//! # Usage
//!
//! ```bash
//! # Robosense E1R
//! cargo run --example lidar_viewer --features rerun -- \
//!     --sensor-type robosense --msop-port 6699
//!
//! # Ouster (requires HTTP configuration first - use main binary)
//! # Note: Ouster requires HTTP API configuration which this example doesn't support
//! ```

use clap::Parser;
use edgefirst_lidarpub::{
    lidar::{LidarDriver, Points, SensorType},
    robosense::RobosenseDriver,
};
use std::net::UdpSocket;

#[derive(Parser, Debug)]
#[command(author, version, about = "Standalone LiDAR point cloud viewer")]
struct Args {
    /// Sensor type to use
    #[arg(long, value_enum, default_value = "robosense")]
    sensor_type: SensorType,

    /// MSOP port for Robosense sensors
    #[arg(long, default_value = "6699")]
    msop_port: u16,

    /// DIFOP port for Robosense sensors (device info)
    #[arg(long, default_value = "7788")]
    difop_port: u16,

    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Initialize Rerun using the clap integration
    let (rec, _guard) = args.rerun.init("lidar_viewer")?;

    match args.sensor_type {
        SensorType::Robosense => run_robosense(&args, rec),
        SensorType::Ouster => {
            eprintln!("Ouster sensors require HTTP API configuration.");
            eprintln!("Use the main edgefirst-lidarpub binary instead.");
            std::process::exit(1);
        }
    }
}

fn run_robosense(
    args: &Args,
    rec: rerun::RecordingStream,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = RobosenseDriver::new();

    let bind_addr = format!("0.0.0.0:{}", args.msop_port);
    println!("Listening for MSOP packets on {}", bind_addr);
    let sock = UdpSocket::bind(&bind_addr)?;

    // Set receive buffer size
    let _ = sock.set_read_timeout(None);

    let mut buf = [0u8; 2048];
    let mut frame_count = 0u64;

    loop {
        let len = sock.recv(&mut buf)?;

        match driver.process_packet(&buf[..len]) {
            Ok(Some(frame)) => {
                frame_count += 1;

                // Convert points to Rerun format
                let points = points_to_rerun(&frame.points, frame.n_points);
                let colors = intensities_to_colors(&frame.points, frame.n_points);

                rec.log(
                    "lidar/points",
                    &rerun::Points3D::new(points).with_colors(colors),
                )?;

                if frame_count.is_multiple_of(10) {
                    println!(
                        "Frame {}: {} points, timestamp {}",
                        frame_count, frame.n_points, frame.timestamp
                    );
                }
            }
            Ok(None) => {
                // More packets needed
            }
            Err(e) => {
                eprintln!("Packet error: {:?}", e);
            }
        }
    }
}

/// Convert Points to Rerun point format
fn points_to_rerun(points: &Points, n_points: usize) -> Vec<[f32; 3]> {
    (0..n_points)
        .map(|i| [points.x[i], points.y[i], points.z[i]])
        .collect()
}

/// Convert intensities to Rerun colors (grayscale based on intensity)
fn intensities_to_colors(points: &Points, n_points: usize) -> Vec<rerun::Color> {
    (0..n_points)
        .map(|i| {
            let intensity = points.intensity[i];
            rerun::Color::from_rgb(intensity, intensity, intensity)
        })
        .collect()
}
