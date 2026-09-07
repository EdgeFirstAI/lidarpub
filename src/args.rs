// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use clap::{CommandFactory, Parser, builder::PossibleValuesParser};
use serde_json::json;
use tracing::level_filters::LevelFilter;
use zenoh::config::{Config, WhatAmI};

use crate::{common::TimestampMode, lidar::SensorType};

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

    /// LiDAR timestamp mode.  If using the PTP1588 timestamp mode the LiDAR
    /// must be connected to a PTP1588 enabled network, the Maivin can provide
    /// this time through the ptp4l service.
    /// (Ouster only)
    #[arg(long, env, default_value = "internal")]
    pub timestamp_mode: TimestampMode,

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
    #[arg(long, env, default_value = "lidar")]
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

/// Environment variables where an empty value is meaningful and must be
/// preserved (i.e. the argument has a non-empty default but "" is a documented
/// "disable" sentinel).
///
/// None for lidarpub: the only arguments where "" is a documented sentinel
/// (`CLUSTERING`, `MIRROR`) already default to "", so scrubbing them is a
/// no-op.
pub const KEEP: &[&str] = &[];

/// Treat an empty environment variable as unset, so clap's declared
/// `default_value` applies instead of failing to parse.
///
/// Only variables bound to this program's own arguments are considered;
/// unrelated process environment is left alone. `keep` names variables
/// where an empty value is meaningful and must be preserved.
///
/// # Safety
/// Must be called before any thread is spawned — that is, before the tokio
/// runtime is built. Mutating the process environment is not thread-safe.
pub unsafe fn scrub_empty_env<C: CommandFactory>(keep: &[&str]) {
    for arg in C::command().get_arguments() {
        let Some(env) = arg.get_env() else { continue };
        let name = env.to_string_lossy().into_owned();
        if keep.contains(&name.as_str()) {
            continue;
        }
        if matches!(std::env::var(&name), Ok(v) if v.is_empty()) {
            unsafe { std::env::remove_var(&name) };
        }
    }
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

/// System hostname used as the Zenoh session namespace.
///
/// Empty or `/`-containing hostnames would create unintended sub-keys, so we
/// fall back to `"localhost"` and warn. Two devices both falling back would
/// silently share a namespace; that is a deployment defect.
fn zenoh_namespace() -> String {
    zenoh_namespace_from(&gethostname::gethostname().to_string_lossy())
}

fn zenoh_namespace_from(raw: &str) -> String {
    if raw.is_empty() || raw.contains('/') {
        tracing::warn!(
            hostname = %raw,
            "system hostname is empty or contains '/' — falling back to \"localhost\""
        );
        "localhost".into()
    } else {
        raw.to_owned()
    }
}

impl From<Args> for Config {
    fn from(args: Args) -> Self {
        let mut config = Config::default();

        // Session namespace = hostname: application keys are bare (`lidar`)
        // and the wire form is `{hostname}/lidar/...`.
        config
            .insert_json5("namespace", &json!(zenoh_namespace()).to_string())
            .unwrap();

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

/// Serialises tests that parse [`Args`] against tests that mutate the process
/// environment. Every `Args::parse_from` in the test binary consults the
/// environment, so a concurrent `set_var` would otherwise race with it.
#[cfg(test)]
pub(crate) fn env_lock() -> std::sync::MutexGuard<'static, ()> {
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    /// Env-bound arguments with a non-empty default where we have consciously
    /// decided that an empty value is NOT meaningful (so scrubbing to the
    /// default is correct).
    const SCRUB_REVIEWED: &[&str] = &[
        "SENSOR_TYPE",
        "AZIMUTH",
        "LIDAR_MODE",
        "TIMESTAMP_MODE",
        "MSOP_PORT",
        "DIFOP_PORT",
        "INCLUDE_NOISY",
        "DISCOVER",
        "TF_VEC",
        "TF_QUAT",
        "BASE_FRAME_ID",
        "FRAME_ID",
        "LIDAR_TOPIC",
        "RUST_LOG",
        "TRACY",
        "CLUSTERING_EPS",
        "CLUSTERING_MINPTS",
        "CLUSTERING_BRIDGE",
        "GROUND_FILTER",
        "GROUND_THICKNESS",
        "MODE",
        "NO_MULTICAST_SCOUTING",
    ];

    fn parse_cli() -> Args {
        Args::parse_from(["edgefirst-lidarpub", "--rust-log", "info"])
    }

    fn with_cleared_lidar_topic<T>(f: impl FnOnce() -> T) -> T {
        let _env = env_lock();
        let saved = std::env::var("LIDAR_TOPIC").ok();
        unsafe { std::env::remove_var("LIDAR_TOPIC") };
        let result = f();
        match saved {
            Some(v) => unsafe { std::env::set_var("LIDAR_TOPIC", v) },
            None => unsafe { std::env::remove_var("LIDAR_TOPIC") },
        }
        result
    }

    #[test]
    fn zenoh_namespace_from_valid_hostname() {
        assert_eq!(zenoh_namespace_from("verdin-imx8mp"), "verdin-imx8mp");
    }

    #[test]
    fn zenoh_namespace_from_empty_falls_back() {
        assert_eq!(zenoh_namespace_from(""), "localhost");
    }

    #[test]
    fn zenoh_namespace_from_slash_falls_back() {
        assert_eq!(zenoh_namespace_from("bad/name"), "localhost");
    }

    #[test]
    fn every_env_arg_is_either_scrubbable_or_explicitly_kept() {
        for arg in Args::command().get_arguments() {
            let Some(env) = arg.get_env() else { continue };
            let name = env.to_string_lossy().into_owned();
            let has_nonempty_default = arg
                .get_default_values()
                .first()
                .is_some_and(|d| !d.is_empty());
            if has_nonempty_default && !KEEP.contains(&name.as_str()) {
                assert!(
                    SCRUB_REVIEWED.contains(&name.as_str()),
                    "{name} has a non-empty default; decide whether empty is meaningful \
                     and add it to KEEP or SCRUB_REVIEWED"
                );
            }
        }
    }

    #[test]
    fn empty_env_vars_fall_back_to_defaults_after_scrub() {
        let _env = env_lock();
        // Numeric, boolean, bare-flag and two-value (num_args = 2) arguments,
        // all written as KEY="" in /etc/default/lidarpub.
        const VARS: [&str; 4] = ["CLUSTERING_EPS", "GROUND_FILTER", "AZIMUTH", "DISCOVER"];
        let saved: Vec<Option<String>> = VARS.iter().map(|v| std::env::var(v).ok()).collect();
        let restore = || {
            for (name, value) in VARS.iter().zip(&saved) {
                match value {
                    Some(v) => unsafe { std::env::set_var(name, v) },
                    None => unsafe { std::env::remove_var(name) },
                }
            }
        };

        for name in VARS {
            unsafe { std::env::set_var(name, "") };
        }
        // Without scrubbing, clap sees "" as a present value and fails.
        let before = Args::try_parse_from(["edgefirst-lidarpub"]);
        // A variable listed in `keep` must survive the scrub untouched.
        unsafe { scrub_empty_env::<Args>(&["CLUSTERING_EPS"]) };
        let kept = std::env::var("CLUSTERING_EPS");
        // The service configuration (KEEP is empty) scrubs everything.
        unsafe { scrub_empty_env::<Args>(KEEP) };
        let remaining: Vec<&str> = VARS
            .into_iter()
            .filter(|v| std::env::var(v).is_ok())
            .collect();
        let after = Args::try_parse_from(["edgefirst-lidarpub"]);
        restore();

        assert!(before.is_err(), "empty env vars should fail without scrub");
        assert_eq!(kept.as_deref(), Ok(""), "KEEP entry must not be removed");
        assert!(remaining.is_empty(), "still set after scrub: {remaining:?}");
        let args = after.expect("empty env vars must fall back to defaults");
        assert_eq!(args.clustering_eps, 200);
        assert!(!args.ground_filter);
        assert_eq!(args.azimuth, [0, 360]);
        assert!(!args.discover);
    }

    #[test]
    fn zenoh_config_sets_namespace() {
        let _env = env_lock();
        let ns = zenoh_namespace();
        assert!(!ns.is_empty(), "namespace should be non-empty");
        assert!(!ns.contains('/'), "namespace must not contain '/'");
        let rendered = Config::from(parse_cli()).to_string();
        assert!(
            rendered.contains(&ns),
            "config should include namespace {ns}: {rendered}"
        );
    }

    #[test]
    fn cli_default_lidar_topic_has_no_rt_prefix() {
        let topic = with_cleared_lidar_topic(|| parse_cli().lidar_topic);
        assert_eq!(topic, "lidar");
    }

    #[test]
    fn clustering_enabled_and_mirror_helpers() {
        let _env = env_lock();
        let disabled = parse_cli();
        assert!(!disabled.clustering_enabled());
        assert!(!disabled.mirror_y());
        assert!(!disabled.mirror_z());

        let voxel = Args::parse_from(["edgefirst-lidarpub", "--clustering", "voxel"]);
        assert!(voxel.clustering_enabled());

        let horizontal = Args::parse_from(["edgefirst-lidarpub", "--mirror", "horizontal"]);
        assert!(horizontal.mirror_y());
        assert!(!horizontal.mirror_z());

        let vertical = Args::parse_from(["edgefirst-lidarpub", "--mirror", "vertical"]);
        assert!(!vertical.mirror_y());
        assert!(vertical.mirror_z());

        let both = Args::parse_from(["edgefirst-lidarpub", "--mirror", "both"]);
        assert!(both.mirror_y());
        assert!(both.mirror_z());
    }

    #[test]
    fn zenoh_config_optional_endpoints_and_scouting() {
        let _env = env_lock();
        let args = Args::parse_from([
            "edgefirst-lidarpub",
            "--connect",
            "tcp/127.0.0.1:7447",
            "--listen",
            "tcp/127.0.0.1:7448",
            "--no-multicast-scouting",
        ]);
        let rendered = Config::from(args).to_string();
        assert!(rendered.contains("127.0.0.1:7447"), "{rendered}");
        assert!(rendered.contains("127.0.0.1:7448"), "{rendered}");
        assert!(
            rendered.contains("false") || rendered.contains("multicast"),
            "{rendered}"
        );
    }
}
