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
    lidar::{LidarDriver, LidarFrame, SensorType},
    robosense::{RobosenseDriver, RobosenseLidarFrame},
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
    // Client-owned frame with appropriate capacity
    let mut frame = RobosenseLidarFrame::new();

    let bind_addr = format!("0.0.0.0:{}", args.msop_port);
    println!("Listening for MSOP packets on {}", bind_addr);
    let sock = UdpSocket::bind(&bind_addr)?;

    // Set receive buffer size
    let _ = sock.set_read_timeout(None);

    let mut buf = [0u8; 2048];
    let mut frame_count = 0u64;

    loop {
        // NOTE: Blocking recv - this is intentional for this simple example.
        // For production, consider async I/O or a separate receiver thread.
        let len = sock.recv(&mut buf)?;

        match driver.process(&mut frame, &buf[..len]) {
            Ok(true) => {
                // Frame complete - process it
                frame_count += 1;

                // Convert points to Rerun format using LidarFrame trait methods
                let points = frame_to_rerun(&frame);
                let colors = intensities_to_colors(&frame);

                rec.log(
                    "lidar/points",
                    &rerun::Points3D::new(points).with_colors(colors),
                )?;

                if frame_count.is_multiple_of(10) {
                    println!(
                        "Frame {}: {} points, timestamp {}",
                        frame_count,
                        frame.len(),
                        frame.timestamp()
                    );
                }
            }
            Ok(false) => {
                // More packets needed
            }
            Err(e) => {
                eprintln!("Packet error: {:?}", e);
            }
        }
    }
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
