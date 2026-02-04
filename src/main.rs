// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Point cloud formatting uses architecture-specific SIMD optimizations:
//! - aarch64: Native NEON intrinsics (stable Rust)
//! - x86_64/other with `portable_simd` feature: std::simd (nightly Rust)
//! - Fallback: Scalar implementation (stable Rust)

#![cfg_attr(
    all(feature = "portable_simd", not(target_arch = "aarch64")),
    feature(portable_simd)
)]

mod args;
mod buffer;
mod cluster;
mod common;
mod formats;
mod lidar;
mod ouster;
mod robosense;

use args::Args;
use clap::Parser as _;
use cluster::cluster_thread;
use edgefirst_schemas::{
    builtin_interfaces::Time,
    geometry_msgs::{Quaternion, Transform, TransformStamped, Vector3},
    sensor_msgs::PointCloud2,
    serde_cdr,
    std_msgs::Header,
};
use formats::{format_points_13byte, standard_xyz_intensity_fields};
use lidar::{LidarDriver, LidarFrame, Points, SensorType};
use ouster::{BeamIntrinsics, Config, LidarDataFormat, Parameters, SensorInfo};
use robosense::RobosenseDriver;
use std::{
    io::{IsTerminal as _, Write as _},
    net::TcpStream,
    sync::{Arc, Mutex},
    thread::sleep,
    time::{Duration, Instant, SystemTime},
};

use tokio::net::UdpSocket;
use tracing::{debug, error, info, trace};
use tracing_subscriber::{Layer as _, Registry, layer::SubscriberExt as _};
use tracy_client::frame_mark;
use zenoh::{
    Session,
    bytes::{Encoding, ZBytes},
    qos::{CongestionControl, Priority},
};

#[cfg(feature = "profiling")]
#[global_allocator]
static GLOBAL: tracy_client::ProfiledAllocator<std::alloc::System> =
    tracy_client::ProfiledAllocator::new(std::alloc::System, 100);

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    args.tracy.then(tracy_client::Client::start);

    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(args.rust_log);

    let journald = match tracing_journald::layer() {
        Ok(journald) => Some(journald.with_filter(args.rust_log)),
        Err(_) => None,
    };

    let tracy = match args.tracy {
        true => Some(tracing_tracy::TracyLayer::default().with_filter(args.rust_log)),
        false => None,
    };

    let subscriber = Registry::default()
        .with(stdout_log)
        .with(journald)
        .with(tracy);
    tracing::subscriber::set_global_default(subscriber).expect("setting default subscriber failed");
    tracing_log::LogTracer::init()?;

    let session = zenoh::open(args.clone()).await.unwrap();

    tokio::spawn(tf_static_loop(session.clone(), args.clone()));

    match args.sensor_type {
        SensorType::Ouster => run_ouster(session, args).await,
        SensorType::Robosense => run_robosense(session, args).await,
    }
}

/// Run the Ouster LiDAR sensor
async fn run_ouster(session: Session, args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let local = {
        let stream = TcpStream::connect(format!("{}:80", &args.target))?;
        stream.local_addr()?.ip()
    };

    let api = format!("http://{}//api/v1/sensor", &args.target);

    let config = Config {
        udp_dest: local.to_string(),
        lidar_mode: args.lidar_mode.clone(),
        timestamp_mode: args.timestamp_mode.to_string(),
        azimuth_window: args
            .azimuth
            .iter()
            .map(|x| x * 1000)
            .collect::<Vec<_>>()
            .try_into()
            .unwrap(),
        ..Default::default()
    };

    ureq::post(&format!("{}/config", api)).send_json(&config)?;
    let config = ureq::get(&format!("{}/config", api))
        .call()?
        .body_mut()
        .read_json::<Config>()?;
    info!("{:?}", config);

    // Get sensor_info continuously until it is running with the updated config.
    let sensor_info = {
        if std::io::stdout().is_terminal() {
            print!("Waiting for LiDAR to initialize");
            std::io::stdout().flush()?;
        }

        loop {
            let sensor_info = ureq::get(&format!("{}/metadata/sensor_info", api))
                .call()?
                .body_mut()
                .read_json::<SensorInfo>()?;
            if sensor_info.status == "RUNNING" {
                if std::io::stdout().is_terminal() {
                    println!("done.");
                } else {
                    info!("LiDAR initialization complete");
                }
                break sensor_info;
            }

            if std::io::stdout().is_terminal() {
                print!(".");
                std::io::stdout().flush()?;
            }

            sleep(Duration::from_secs(1));
        }
    };

    let lidar_data_format = ureq::get(&format!("{}/metadata/lidar_data_format", api))
        .call()?
        .body_mut()
        .read_json::<LidarDataFormat>()?;
    let beam_intrinsics = ureq::get(&format!("{}/metadata/beam_intrinsics", api))
        .call()?
        .body_mut()
        .read_json::<BeamIntrinsics>()?;

    let params = Parameters {
        sensor_info,
        lidar_data_format,
        beam_intrinsics,
    };

    debug!("{:?}", params);

    // Create OusterDriver
    let driver = ouster::OusterDriver::new(&params)?;
    let rows = driver.rows();
    let cols = driver.cols();

    // On Linux [::] will bind to IPv4 and IPv6 but not on Windows so we bind
    // according to the local address IP version.
    let bind_addr = match local.is_ipv4() {
        true => format!("0.0.0.0:{}", config.udp_port_lidar),
        false => format!("[::]:{}", config.udp_port_lidar),
    };

    run_lidar_loop(session, args, driver, &bind_addr, rows, cols).await
}

/// Run the Robosense E1R LiDAR sensor
async fn run_robosense(session: Session, args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let driver = Arc::new(Mutex::new(RobosenseDriver::new()));

    // Start DIFOP listener for device information
    let difop_driver = driver.clone();
    let difop_port = args.difop_port;
    tokio::spawn(async move {
        let bind_addr = format!("0.0.0.0:{}", difop_port);
        let sock = match UdpSocket::bind(&bind_addr).await {
            Ok(s) => s,
            Err(e) => {
                error!("Failed to bind DIFOP socket on {}: {:?}", bind_addr, e);
                return;
            }
        };
        info!("Listening for DIFOP packets on port {}", difop_port);

        let mut buf = [0u8; 512];
        loop {
            match sock.recv(&mut buf).await {
                Ok(len) => {
                    if let Ok(mut driver) = difop_driver.lock() {
                        if let Err(e) = driver.process_difop(&buf[..len]) {
                            debug!("DIFOP parse error: {:?}", e);
                        } else {
                            let info = driver.device_info();
                            trace!(
                                serial = %info.serial_string(),
                                version = %info.version_string(),
                                "DIFOP received"
                            );
                        }
                    }
                }
                Err(e) => {
                    error!("DIFOP recv error: {:?}", e);
                }
            }
        }
    });

    // E1R doesn't have a fixed row/col structure like Ouster
    // Use approximate values for clustering (not typically used with E1R)
    let rows = 1;
    let cols = 30_000; // ~26k points per frame

    let bind_addr = format!("0.0.0.0:{}", args.msop_port);
    info!("Listening for MSOP packets on port {}", args.msop_port);

    // Wrap the driver in a struct that can be used with run_lidar_loop
    let driver = RobosenseDriverWrapper(driver);

    run_lidar_loop(session, args, driver, &bind_addr, rows, cols).await
}

/// Wrapper to allow shared RobosenseDriver with DIFOP thread
struct RobosenseDriverWrapper(Arc<Mutex<RobosenseDriver>>);

impl LidarDriver for RobosenseDriverWrapper {
    fn process_packet(&mut self, data: &[u8]) -> Result<Option<LidarFrame>, lidar::Error> {
        self.0.lock().unwrap().process_packet(data)
    }
}

/// Generic lidar processing loop
async fn run_lidar_loop<D: LidarDriver>(
    session: Session,
    args: Args,
    mut driver: D,
    bind_addr: &str,
    rows: usize,
    cols: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    let points_publisher = session
        .declare_publisher(format!("{}/points", args.lidar_topic))
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    let cluster_publisher = session
        .declare_publisher(format!("{}/clusters", args.lidar_topic))
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    // Set up clustering if enabled
    let (tx_cluster, rx_cluster) = kanal::bounded(8);
    if args.clustering {
        let args_ = args.clone();
        match std::thread::Builder::new()
            .name("cluster".to_string())
            .spawn(move || {
                tokio::runtime::Builder::new_multi_thread()
                    .enable_all()
                    .build()
                    .unwrap()
                    .block_on(cluster_thread(
                        rx_cluster,
                        cluster_publisher,
                        rows,
                        cols,
                        args_,
                    ));
            }) {
            Ok(_) => info!("Clustering thread started"),
            Err(e) => error!("Could not start clustering thread: {:?}", e),
        };
    }

    common::set_process_priority();
    let sock = UdpSocket::bind(bind_addr).await?;
    let sock = common::set_socket_bufsize(sock.into_std()?, 16 * 1024 * 1024);
    let sock = UdpSocket::from_std(sock)?;

    let mut buf = [0u8; 16 * 1024];

    info!("Starting LiDAR processing loop");

    loop {
        let len = match sock.recv(&mut buf).await {
            Ok(len) => len,
            Err(e) => {
                error!("UDP recv error: {:?}", e);
                continue;
            }
        };

        match driver.process_packet(&buf[..len]) {
            Ok(Some(frame)) => {
                trace!(
                    timestamp = frame.timestamp,
                    frame_id = frame.frame_id,
                    n_points = frame.n_points,
                    "publishing frame"
                );

                let timestamp = Time::from_nanos(frame.timestamp);

                // Send to clustering if enabled
                if args.clustering {
                    // For clustering we need range data - compute from XYZ
                    let ranges: Vec<f32> = (0..frame.n_points)
                        .map(|i| {
                            let x = frame.points.x[i];
                            let y = frame.points.y[i];
                            let z = frame.points.z[i];
                            (x * x + y * y + z * z).sqrt()
                        })
                        .collect();
                    let _ = tx_cluster.send((ranges, frame.points.clone(), timestamp.clone()));
                }

                // Format and publish point cloud
                let (msg, enc) = format_points(
                    &frame.points,
                    frame.n_points,
                    timestamp,
                    args.frame_id.clone(),
                )?;

                if let Err(e) = points_publisher.put(msg).encoding(enc).await {
                    error!("publish points error: {:?}", e);
                }

                args.tracy.then(frame_mark);
            }
            Ok(None) => {
                // More packets needed to complete frame
            }
            Err(e) => {
                debug!("Packet processing error: {:?}", e);
            }
        }
    }
}

/// Format point cloud data into a PointCloud2 message.
///
/// Uses the shared SIMD formatters from the formats module.
#[inline(never)]
fn format_points(
    points: &Points,
    n_points: usize,
    timestamp: Time,
    frame_id: String,
) -> Result<(ZBytes, Encoding), serde_cdr::Error> {
    let fields = standard_xyz_intensity_fields();

    // Use the shared SIMD formatter
    let data = format_points_13byte(&points.x, &points.y, &points.z, &points.intensity, n_points);

    let msg = PointCloud2 {
        header: Header {
            stamp: timestamp,
            frame_id,
        },
        height: 1,
        width: n_points as u32,
        fields,
        is_bigendian: false,
        point_step: 13,
        row_step: 13 * n_points as u32,
        data,
        is_dense: true,
    };

    let msg = ZBytes::from(serde_cdr::serialize(&msg)?);
    let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/PointCloud2");

    Ok((msg, enc))
}

async fn tf_static_loop(session: Session, args: Args) {
    let publisher = session
        .declare_publisher("rt/tf_static".to_string())
        .priority(Priority::Background)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    let timestamp = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap();
    let timestamp = Time::from_nanos(timestamp.as_nanos() as u64);
    let msg = TransformStamped {
        header: Header {
            frame_id: args.base_frame_id.clone(),
            stamp: timestamp,
        },
        child_frame_id: args.frame_id.clone(),
        transform: Transform {
            translation: Vector3 {
                x: args.tf_vec[0],
                y: args.tf_vec[1],
                z: args.tf_vec[2],
            },
            rotation: Quaternion {
                x: args.tf_quat[0],
                y: args.tf_quat[1],
                z: args.tf_quat[2],
                w: args.tf_quat[3],
            },
        },
    };

    let msg = ZBytes::from(serde_cdr::serialize(&msg).unwrap());
    let enc = Encoding::APPLICATION_CDR.with_schema("geometry_msgs/msg/TransformStamped");

    let interval = Duration::from_secs(1);
    let mut target_time = Instant::now() + interval;

    loop {
        publisher
            .put(msg.clone())
            .encoding(enc.clone())
            .await
            .unwrap();
        trace!("lidarpub publishing rt/tf");
        sleep(target_time.duration_since(Instant::now()));
        target_time += interval
    }
}
