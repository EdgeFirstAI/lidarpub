// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use clap::{Parser, builder::PossibleValuesParser};
use serde_json::json;
use tracing::level_filters::LevelFilter;
use zenoh::config::{Config, WhatAmI};

use crate::common::TimestampMode;

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
pub struct Args {
    /// Connect to target device or pcap file.  If target is a valid pcap file,
    /// it will be used otherwise it will be tried as a hostname or IP address.
    #[arg(env)]
    pub target: String,

    /// Azimuth field of view start and stop angles in degrees.  
    /// The 0 degree point is the rear connector of the LiDAR.
    #[arg(long, env, num_args = 2, value_names = ["START", "STOP"], value_delimiter=' ', default_value = "0 360")]
    pub azimuth: Vec<u32>,

    /// LiDAR column and refresh rate configuration.  The format is "COLxHZ".
    #[arg(long, env, default_value = "1024x10", 
          value_parser = PossibleValuesParser::new(["512x10", "1024x10", "2048x10", "512x20", "1024x20",]))]
    pub lidar_mode: String,

    /// LiDAR timestamp mode.  If using the PTP1588 timestamp mode the LiDAR
    /// must be connected to a PTP1588 enabled network, the Maivin can provide
    /// this time through the ptp4l service.
    #[arg(long, env, default_value = "internal")]
    pub timestamp_mode: TimestampMode,

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

    /// Enable lidar clustering task.
    #[arg(long, env, default_value = "false")]
    pub clustering: bool,

    /// the distancing metric for clustering, in millimeters
    #[arg(long, env, default_value = "256")]
    pub clustering_eps: u16,

    /// the number of points needed per clustering, in millimeters
    #[arg(long, env, default_value = "4")]
    pub clustering_minpts: usize,

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
