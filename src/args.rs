// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use clap::{Parser, builder::PossibleValuesParser};
use serde_json::json;
use tracing::level_filters::LevelFilter;
use zenoh::config::{Config, WhatAmI};

use crate::{
    common::TimeSync,
    lidar::{SensorType, TimeSource},
};

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
pub struct Args {
    /// Sensor type (ouster or robosense)
    #[arg(long, env, default_value = "ouster", value_enum)]
    pub sensor_type: SensorType,

    /// Connect to target device or pcap file.  If target is a valid pcap file,
    /// it will be used otherwise it will be tried as a hostname or IP address.
    /// Required for Ouster sensors. For Robosense, optional: when provided,
    /// filters packets to only accept from this source IP.
    #[arg(env)]
    pub target: Option<String>,

    // --- Ouster-specific options ---
    /// Azimuth field of view start and stop angles in degrees.
    /// The 0 degree point is the rear connector of the LiDAR.
    /// (Ouster only)
    #[arg(long, env, num_args = 2, value_names = ["START", "STOP"], value_delimiter=' ', default_value = "0 360")]
    pub azimuth: Vec<u32>,

    /// LiDAR column and refresh rate configuration.  The format is "COLxHZ".
    /// (Ouster only)
    #[arg(long, env, default_value = "1024x10",
          value_parser = PossibleValuesParser::new(["512x10", "1024x10", "2048x10", "512x20", "1024x20",]))]
    pub lidar_mode: String,

    /// LiDAR time synchronization mode. Configures which clock the Ouster
    /// sensor hardware uses. "internal" uses the internal oscillator,
    /// "ptp-1588" synchronizes to a PTP master (e.g. ptp4l on Maivin).
    /// (Ouster only)
    #[arg(
        long,
        env = "TIME_SYNC",
        default_value = "internal",
        alias = "timestamp-mode"
    )]
    pub time_sync: TimeSync,

    /// Timestamp source for point cloud and cluster messages.
    /// "host" uses the host wall clock (CLOCK_REALTIME) at frame start.
    /// "sensor" uses the sensor's packet timestamp with sanity validation,
    /// falling back to host time if validation fails.
    #[arg(long, env, default_value = "host")]
    pub time_source: TimeSource,

    /// Timestamp offset in nanoseconds, subtracted from each frame timestamp
    /// to compensate for estimated sensor-to-host latency. A positive value
    /// shifts timestamps earlier; negative shifts forward. 0 = disabled.
    #[arg(long, env, default_value = "0")]
    pub timestamp_offset: i64,

    // --- Robosense-specific options ---
    /// MSOP (Main data Stream Output Protocol) port for Robosense sensors.
    /// (Robosense only)
    #[arg(long, env, default_value = "6699")]
    pub msop_port: u16,

    /// DIFOP (Device Information Output Protocol) port for Robosense sensors.
    /// (Robosense only)
    #[arg(long, env, default_value = "7788")]
    pub difop_port: u16,

    /// Include noisy points (PointAttribute == 2) in the point cloud.
    /// By default, noisy points are filtered out.
    /// (Robosense only)
    #[arg(long, env, default_value = "false")]
    pub include_noisy: bool,

    /// Discover Robosense sensors on the network and exit.
    /// Listens for DIFOP packets and prints device information.
    /// (Robosense only)
    #[arg(long, env)]
    pub discover: bool,

    /// Frame transformation vector from the base_link
    #[arg(
        long,
        env,
        default_value = "0 0 0",
        value_delimiter = ' ',
        num_args = 3
    )]
    pub tf_vec: Vec<f64>,

    /// Frame transformation quaternion from the base_link
    #[arg(
        long,
        env,
        default_value = "0 0 0 1",
        value_delimiter = ' ',
        num_args = 4
    )]
    pub tf_quat: Vec<f64>,

    /// The name of the base frame
    #[arg(long, env, default_value = "base_link")]
    pub base_frame_id: String,

    /// The name of the lidar frame
    #[arg(long, env, default_value = "lidar")]
    pub frame_id: String,

    /// lidar base topic
    #[arg(long, env, default_value = "rt/lidar")]
    pub lidar_topic: String,

    /// Application log level
    #[arg(long, env, default_value = "info")]
    pub rust_log: LevelFilter,

    /// Enable Tracy profiler broadcast
    #[arg(long, env)]
    pub tracy: bool,

    /// Clustering algorithm: "" (disabled), "dbscan", "voxel"
    #[arg(long, env, default_value = "",
          value_parser = PossibleValuesParser::new(["", "dbscan", "voxel"]))]
    pub clustering: String,

    /// 3D Euclidean distance threshold for clustering, in millimeters
    #[arg(long, env, default_value = "200")]
    pub clustering_eps: u16,

    /// Minimum number of points to form a cluster
    #[arg(long, env, default_value = "4")]
    pub clustering_minpts: usize,

    /// Minimum neighbors for a point to propagate during cluster expansion.
    /// Higher values prevent thin structures (ropes, wires) from merging
    /// separate objects. 0 = same as clustering_minpts (standard DBSCAN).
    #[arg(long, env, default_value = "0")]
    pub clustering_bridge: usize,

    /// Enable IMU-guided ground plane removal before clustering.
    /// Requires IMU data (Robosense built-in or external).
    #[arg(long, env, default_value = "false")]
    pub ground_filter: bool,

    /// Ground slab thickness for ground plane removal, in millimeters.
    /// Points within this distance above the detected ground plane are removed.
    #[arg(long, env, default_value = "150")]
    pub ground_thickness: u16,

    /// Known sensor height above ground in millimeters. When set, skips
    /// automatic ground detection and uses this fixed height instead.
    #[arg(long, env)]
    pub sensor_height: Option<u16>,

    /// Mirror the point cloud output. Useful when the sensor's coordinate
    /// frame doesn't match the expected orientation.
    ///   ""           - No mirroring (default)
    ///   "horizontal" - Flip left-right (negate Y)
    ///   "vertical"   - Flip up-down (negate Z)
    ///   "both"       - Flip both axes
    #[arg(long, env, default_value = "",
          value_parser = PossibleValuesParser::new(["", "horizontal", "vertical", "both"]))]
    pub mirror: String,

    /// zenoh connection mode
    #[arg(long, env, default_value = "peer")]
    mode: WhatAmI,

    /// connect to zenoh endpoints
    #[arg(long, env)]
    connect: Vec<String>,

    /// listen to zenoh endpoints
    #[arg(long, env)]
    listen: Vec<String>,

    /// disable zenoh multicast scouting
    #[arg(long, env)]
    no_multicast_scouting: bool,
}

impl Args {
    pub fn clustering_enabled(&self) -> bool {
        !self.clustering.is_empty()
    }

    pub fn mirror_y(&self) -> bool {
        self.mirror == "horizontal" || self.mirror == "both"
    }

    pub fn mirror_z(&self) -> bool {
        self.mirror == "vertical" || self.mirror == "both"
    }
}

impl From<Args> for Config {
    fn from(args: Args) -> Self {
        let mut config = Config::default();

        config
            .insert_json5("mode", &json!(args.mode).to_string())
            .unwrap();

        if !args.connect.is_empty() {
            config
                .insert_json5("connect/endpoints", &json!(args.connect).to_string())
                .unwrap();
        }

        if !args.listen.is_empty() {
            config
                .insert_json5("listen/endpoints", &json!(args.listen).to_string())
                .unwrap();
        }

        if args.no_multicast_scouting {
            config
                .insert_json5("scouting/multicast/enabled", &json!(false).to_string())
                .unwrap();
        }

        config
            .insert_json5("scouting/multicast/interface", &json!("lo").to_string())
            .unwrap();

        config
    }
}
