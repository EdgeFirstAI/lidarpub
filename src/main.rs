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
mod cluster_thread;
mod common;
mod formats;
mod lidar;
mod ouster;
mod robosense;

use args::Args;
use clap::Parser as _;
use cluster_thread::cluster_thread;
use edgefirst_schemas::{
    builtin_interfaces::Time,
    cdr::CdrError,
    geometry_msgs::{Quaternion, Vector3},
};
use formats::{encode_imu_cdr, encode_transform_stamped_cdr, encode_xyzr_pointcloud2_cdr};
use lidar::{LidarDriver, LidarFrame, SensorType};
use ouster::{BeamIntrinsics, Config, LidarDataFormat, OusterLidarFrame, Parameters, SensorInfo};
use robosense::{RobosenseDriver, RobosenseLidarFrame};
use std::{
    collections::HashMap,
    io::{IsTerminal as _, Write as _},
    net::TcpStream,
    sync::{Arc, Mutex},
    thread::sleep,
    time::{Duration, Instant},
};

use tokio::net::UdpSocket;
use tracing::{debug, error, info, trace, warn};
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

    if args.discover {
        return run_discover(&args).await;
    }

    match args.sensor_type {
        SensorType::Ouster => run_ouster(session, args).await,
        SensorType::Robosense => run_robosense(session, args).await,
    }
}

/// Run the Ouster LiDAR sensor
async fn run_ouster(session: Session, args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let target = args.target.as_deref().ok_or(
        "Ouster sensor requires a target hostname or IP address. Usage: edgefirst-lidarpub <TARGET>",
    )?;

    let local = {
        let stream = TcpStream::connect(format!("{}:80", target))?;
        stream.local_addr()?.ip()
    };

    let api = format!("http://{}//api/v1/sensor", target);

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

    // Create client-owned frame with appropriate capacity
    let capacity = rows * cols;
    let frame = OusterLidarFrame::with_capacity(capacity);

    // On Linux [::] will bind to IPv4 and IPv6 but not on Windows so we bind
    // according to the local address IP version.
    let bind_addr = match local.is_ipv4() {
        true => format!("0.0.0.0:{}", config.udp_port_lidar),
        false => format!("[::]:{}", config.udp_port_lidar),
    };

    // Ouster has no built-in IMU plumbing — no ground filter IMU source
    let no_imu: Arc<Mutex<Option<(f32, f32, f32)>>> = Arc::new(Mutex::new(None));
    run_lidar_loop(session, args, driver, frame, &bind_addr, None, no_imu).await
}

/// Run the Robosense E1R LiDAR sensor
async fn run_robosense(session: Session, args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let mut robosense_driver = RobosenseDriver::new();
    robosense_driver.set_filter_noisy(!args.include_noisy);
    let driver = Arc::new(Mutex::new(robosense_driver));

    // Start DIFOP listener for device information and IMU publishing
    let difop_driver = driver.clone();
    let difop_port = args.difop_port;
    let imu_topic = format!("{}/imu", args.lidar_topic);
    let imu_frame_id = args.frame_id.clone();

    let imu_publisher = session
        .declare_publisher(imu_topic)
        .priority(Priority::Data)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    // Shared latest IMU reading for ground plane filtering
    let latest_imu: Arc<Mutex<Option<(f32, f32, f32)>>> = Arc::new(Mutex::new(None));
    let imu_writer = latest_imu.clone();

    // Use oneshot channel to confirm DIFOP startup (PR #7 fix)
    let (startup_tx, startup_rx) = tokio::sync::oneshot::channel();

    tokio::spawn(async move {
        let bind_addr = format!("0.0.0.0:{}", difop_port);
        let sock = match UdpSocket::bind(&bind_addr).await {
            Ok(s) => {
                let _ = startup_tx.send(Ok(()));
                s
            }
            Err(e) => {
                let _ = startup_tx.send(Err(e));
                return;
            }
        };
        info!("Listening for DIFOP packets on port {}", difop_port);

        let mut logged_imu_raw = false;
        let mut buf = [0u8; 512];
        let mut logged_device_info = false;
        let mut last_device_info: Option<robosense::DeviceInfo> = None;

        loop {
            match sock.recv(&mut buf).await {
                Ok(len) => {
                    // Extract device info under lock, then release before await
                    let info = {
                        let Ok(mut driver) = difop_driver.lock() else {
                            continue;
                        };
                        if let Err(e) = driver.process_difop(&buf[..len]) {
                            debug!("DIFOP parse error: {:?}", e);
                            continue;
                        }
                        driver.device_info().clone()
                    };

                    // First DIFOP: log at INFO level
                    if !logged_device_info {
                        info!(
                            serial = %info.serial_string(),
                            firmware = %info.version_string(),
                            timesync_mode = ?info.timesync_mode,
                            timesync_status = ?info.timesync_status,
                            "Robosense E1R device info"
                        );
                        logged_device_info = true;
                    } else if last_device_info.as_ref() != Some(&info) {
                        trace!(
                            serial = %info.serial_string(),
                            firmware = %info.version_string(),
                            "DIFOP device info updated"
                        );
                    }

                    // Store latest IMU accel for ground plane filtering.
                    // The E1R IMU axes (from DIFOP) vs LiDAR point cloud frame:
                    //   IMU +Y = gravity (down)  → LiDAR -Z
                    //   IMU +X = LiDAR left      → LiDAR -X (or similar horizontal)
                    //   IMU +Z = LiDAR forward   → LiDAR +Y? or +X?
                    // Previous remap (imu_x, imu_z, -imu_y) put tilt in LiDAR Y
                    // but visual test showed 90° CW error → swap X/Y:
                    //   LiDAR = (imu_z, -imu_x, -imu_y)
                    if let Some(imu) = &info.imu {
                        if !logged_imu_raw {
                            info!(
                                "IMU raw sensor: accel=({:.3}, {:.3}, {:.3}) gyro=({:.3}, {:.3}, {:.3})",
                                imu.accel_x,
                                imu.accel_y,
                                imu.accel_z,
                                imu.gyro_x,
                                imu.gyro_y,
                                imu.gyro_z
                            );
                            logged_imu_raw = true;
                        }
                        if let Ok(mut lock) = imu_writer.lock() {
                            *lock = Some((imu.accel_z, -imu.accel_x, -imu.accel_y));
                        }
                    }

                    // Publish IMU data if present and wall-clock stamp available
                    if let Some(imu) = &info.imu
                        && let Some(stamp) = get_stamp()
                    {
                        match encode_imu_cdr(
                            stamp,
                            imu_frame_id.as_str(),
                            Vector3 {
                                x: imu.gyro_x as f64,
                                y: imu.gyro_y as f64,
                                z: imu.gyro_z as f64,
                            },
                            Vector3 {
                                x: imu.accel_x as f64,
                                y: imu.accel_y as f64,
                                z: imu.accel_z as f64,
                            },
                        ) {
                            Ok(cdr) => {
                                let zbytes = ZBytes::from(cdr);
                                let enc =
                                    Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/Imu");
                                if let Err(e) = imu_publisher.put(zbytes).encoding(enc).await {
                                    debug!("IMU publish error: {:?}", e);
                                }
                            }
                            Err(e) => debug!("IMU encode error: {:?}", e),
                        }
                    }

                    last_device_info = Some(info);
                }
                Err(e) => {
                    error!("DIFOP recv error: {:?}", e);
                }
            }
        }
    });

    // Wait for DIFOP startup confirmation (PR #7 fix)
    match startup_rx.await {
        Ok(Ok(())) => info!("DIFOP listener started on port {}", difop_port),
        Ok(Err(e)) => warn!("DIFOP bind failed: {} (continuing without DIFOP)", e),
        Err(_) => warn!("DIFOP startup channel dropped"),
    }

    let bind_addr = format!("0.0.0.0:{}", args.msop_port);
    info!("Listening for MSOP packets on port {}", args.msop_port);

    // Parse target as source IP filter for Robosense
    let source_filter: Option<std::net::IpAddr> = args
        .target
        .as_deref()
        .filter(|t| !t.is_empty())
        .map(|t| t.parse())
        .transpose()
        .map_err(|e| format!("Invalid target IP address: {}", e))?;

    // Create client-owned frame
    let frame = RobosenseLidarFrame::new();

    // Wrap the driver in a struct that can be used with run_lidar_loop
    let driver = RobosenseDriverWrapper {
        inner: driver,
        logged_return_mode: false,
    };

    run_lidar_loop(
        session,
        args,
        driver,
        frame,
        &bind_addr,
        source_filter,
        latest_imu,
    )
    .await
}

/// Discover LiDAR sensors on the network.
///
/// Runs all discovery methods in parallel and stops on Ctrl-C.
/// - Robosense: Passive UDP listening for DIFOP packets on the configured port
/// - Ouster: mDNS browsing for `_roger._tcp` services (requires `discovery` feature)
async fn run_discover(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    use std::sync::atomic::{AtomicUsize, Ordering};

    let found = Arc::new(AtomicUsize::new(0));

    println!("Discovering LiDAR sensors... Press Ctrl-C to stop.\n");

    let ouster_found = found.clone();
    tokio::spawn(async move {
        discover_ouster(ouster_found).await;
    });

    let robosense_found = found.clone();
    let difop_port = args.difop_port;
    tokio::spawn(async move {
        if let Err(e) = discover_robosense(difop_port, robosense_found).await {
            error!("Robosense discovery error: {}", e);
        }
    });

    // Wait for Ctrl-C
    tokio::signal::ctrl_c().await.ok();
    println!();

    let total = found.load(Ordering::Relaxed);
    println!("Discovery complete: {} sensor(s) found", total);

    // The mDNS daemon spawns background threads that can't be cancelled
    // from here, so force a clean exit.
    std::process::exit(0);
}

/// Discover Ouster sensors via mDNS browsing for `_roger._tcp`.
#[cfg(feature = "discovery")]
async fn discover_ouster(found: Arc<std::sync::atomic::AtomicUsize>) {
    use mdns_sd::{ServiceDaemon, ServiceEvent};
    use std::sync::atomic::Ordering;

    let mdns = match ServiceDaemon::new() {
        Ok(d) => d,
        Err(e) => {
            warn!("Failed to start mDNS daemon: {}", e);
            return;
        }
    };

    let service_type = "_roger._tcp.local.";
    let receiver = match mdns.browse(service_type) {
        Ok(r) => r,
        Err(e) => {
            warn!("Failed to browse mDNS: {}", e);
            let _ = mdns.shutdown();
            return;
        }
    };

    // mDNS recv_timeout is blocking, run on a blocking thread
    tokio::task::spawn_blocking(move || {
        loop {
            match receiver.recv_timeout(Duration::from_secs(1)) {
                Ok(ServiceEvent::ServiceResolved(info)) => {
                    let hostname = info.get_hostname();
                    let port = info.get_port();
                    let properties = info.get_properties();

                    let sn = properties
                        .get("sn")
                        .map(|v| v.val_str().to_string())
                        .unwrap_or_default();
                    let pn = properties
                        .get("pn")
                        .map(|v| v.val_str().to_string())
                        .unwrap_or_default();
                    let fw = properties
                        .get("fw")
                        .map(|v| v.val_str().to_string())
                        .unwrap_or_default();

                    let addrs_v4 = info.get_addresses_v4();
                    let ip_str = addrs_v4
                        .iter()
                        .next()
                        .map(|a| a.to_string())
                        .unwrap_or_else(|| "unknown".to_string());

                    println!("Found Ouster at {}:", ip_str);
                    println!("  Hostname:  {}", hostname);
                    println!("  Part No:   {}", pn);
                    println!("  Serial:    {}", sn);
                    println!("  Firmware:  {}", fw);
                    println!("  API Port:  {}", port);

                    // Try to query HTTP API for more details
                    if let Some(addr) = addrs_v4.iter().next() {
                        match query_ouster_api(std::net::IpAddr::V4(*addr), port) {
                            Ok(sensor_info) => {
                                println!("  Product:   {}", sensor_info.prod_line);
                                println!("  Status:    {}", sensor_info.status);
                                println!("  Build:     {}", sensor_info.build_rev);
                            }
                            Err(e) => {
                                log::debug!("Could not query Ouster API: {}", e);
                            }
                        }
                    }

                    println!();
                    found.fetch_add(1, Ordering::Relaxed);
                }
                Ok(_) => {}
                Err(_) => {}
            }
        }
    })
    .await
    .ok();
}

/// Query Ouster HTTP API for sensor information.
#[cfg(feature = "discovery")]
fn query_ouster_api(
    addr: std::net::IpAddr,
    port: u16,
) -> Result<SensorInfo, Box<dyn std::error::Error>> {
    let api = format!(
        "http://{}:{}/api/v1/sensor/metadata/sensor_info",
        addr, port
    );
    let sensor_info = ureq::get(&api)
        .call()?
        .body_mut()
        .read_json::<SensorInfo>()?;
    Ok(sensor_info)
}

/// Stub when discovery feature is not enabled.
#[cfg(not(feature = "discovery"))]
async fn discover_ouster(_found: Arc<std::sync::atomic::AtomicUsize>) {
    println!("Ouster discovery requires the 'discovery' feature.");
    println!("  Rebuild with: cargo build --features discovery\n");
}

/// Discover Robosense sensors via passive DIFOP UDP listening.
async fn discover_robosense(
    difop_port: u16,
    found: Arc<std::sync::atomic::AtomicUsize>,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::sync::atomic::Ordering;

    let bind_addr = format!("0.0.0.0:{}", difop_port);
    let sock = UdpSocket::bind(&bind_addr).await?;

    let mut buf = [0u8; 512];
    let mut seen: HashMap<std::net::IpAddr, robosense::DeviceInfo> = HashMap::new();

    loop {
        let (len, addr) = sock.recv_from(&mut buf).await?;

        let src_ip = addr.ip();
        if seen.contains_key(&src_ip) {
            continue;
        }

        let mut driver = RobosenseDriver::new();
        if driver.process_difop(&buf[..len]).is_ok() {
            let info = driver.device_info().clone();
            let local_ip = format!(
                "{}.{}.{}.{}",
                info.local_ip[0], info.local_ip[1], info.local_ip[2], info.local_ip[3]
            );

            println!("Found Robosense E1R at {}:", src_ip);
            println!("  Serial:    {}", info.serial_string());
            println!("  Firmware:  {}", info.version_string());
            println!(
                "  Time Sync: {:?} ({:?})",
                info.timesync_mode, info.timesync_status
            );
            println!("  Local IP:  {}", local_ip);
            println!("  MSOP Port: {}", info.msop_port);
            println!("  DIFOP Port: {}", info.difop_port);

            if let Some(imu) = &info.imu {
                println!(
                    "  IMU:       accel=({:.3}, {:.3}, {:.3}) gyro=({:.3}, {:.3}, {:.3})",
                    imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x, imu.gyro_y, imu.gyro_z
                );
            }
            println!();

            found.fetch_add(1, Ordering::Relaxed);
            seen.insert(src_ip, info);
        }
    }
}

/// Wrapper to allow shared RobosenseDriver with DIFOP thread
struct RobosenseDriverWrapper {
    inner: Arc<Mutex<RobosenseDriver>>,
    logged_return_mode: bool,
}

impl LidarDriver for RobosenseDriverWrapper {
    fn process<F: lidar::LidarFrameWriter>(
        &mut self,
        frame: &mut F,
        data: &[u8],
    ) -> Result<bool, lidar::Error> {
        // Handle mutex poison gracefully - the DIFOP thread may have panicked
        // but the driver state is likely still valid for packet processing
        match self.inner.lock() {
            Ok(mut driver) => {
                let result = driver.process(frame, data);
                if !self.logged_return_mode
                    && let Ok(true) = &result
                {
                    info!(return_mode = %driver.return_mode(), "First frame received");
                    self.logged_return_mode = true;
                }
                result
            }
            Err(poisoned) => {
                warn!(
                    "Driver mutex poisoned (DIFOP thread may have panicked), \
                     recovering with potentially stale device info"
                );
                poisoned.into_inner().process(frame, data)
            }
        }
    }
}

/// Generic lidar processing loop
async fn run_lidar_loop<D: LidarDriver, F: lidar::LidarFrameWriter + LidarFrame>(
    session: Session,
    args: Args,
    mut driver: D,
    mut frame: F,
    bind_addr: &str,
    source_filter: Option<std::net::IpAddr>,
    latest_imu: Arc<Mutex<Option<(f32, f32, f32)>>>,
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
    if args.clustering_enabled() {
        let args_ = args.clone();
        match std::thread::Builder::new()
            .name("cluster".to_string())
            .spawn(move || {
                tokio::runtime::Builder::new_multi_thread()
                    .enable_all()
                    .build()
                    .expect("Failed to create clustering runtime")
                    .block_on(cluster_thread(rx_cluster, cluster_publisher, args_));
            }) {
            Ok(_) => info!("Clustering thread started"),
            Err(e) => {
                error!("Could not start clustering thread: {:?}", e);
                std::process::exit(1);
            }
        };
    }

    common::set_process_priority();
    let sock = UdpSocket::bind(bind_addr).await?;
    let sock = common::set_socket_bufsize(sock.into_std()?, 16 * 1024 * 1024);
    let sock = UdpSocket::from_std(sock)?;

    let mut buf = [0u8; 16 * 1024];

    if let Some(filter_ip) = source_filter {
        info!("Filtering packets from source IP: {}", filter_ip);
    }
    info!("Starting LiDAR processing loop");

    loop {
        let (len, src_addr) = match sock.recv_from(&mut buf).await {
            Ok((len, addr)) => (len, addr),
            Err(e) => {
                error!("UDP recv error: {:?}", e);
                continue;
            }
        };

        // Filter by source IP if configured
        if let Some(filter_ip) = source_filter
            && src_addr.ip() != filter_ip
        {
            continue;
        }

        // Process packet into client-owned frame
        match driver.process(&mut frame, &buf[..len]) {
            Ok(true) => {
                // Frame is complete - process it
                let n_points = frame.len();
                let timestamp_ns = frame.timestamp();
                let frame_id = frame.frame_id();

                trace!(
                    timestamp = timestamp_ns,
                    frame_id = frame_id,
                    n_points = n_points,
                    "publishing frame"
                );

                let timestamp = Time::from_nanos(timestamp_ns);

                // Send to clustering if enabled
                if args.clustering_enabled() {
                    // Use pre-computed range directly - NO sqrt needed! (PR #2 fix)
                    let ranges: Vec<f32> = frame.range().to_vec();
                    let points = lidar::Points {
                        x: frame.x().to_vec(),
                        y: frame.y().to_vec(),
                        z: frame.z().to_vec(),
                        intensity: frame.intensity().to_vec(),
                    };
                    let imu_accel = if args.ground_filter {
                        latest_imu.lock().ok().and_then(|lock| *lock)
                    } else {
                        None
                    };
                    let _ = tx_cluster.send((ranges, points, timestamp, imu_accel));
                }

                // Format and publish point cloud
                let (msg, enc) = format_points(
                    &frame,
                    timestamp,
                    args.frame_id.clone(),
                    args.mirror_y(),
                    args.mirror_z(),
                )?;

                if let Err(e) = points_publisher.put(msg).encoding(enc).await {
                    error!("publish points error: {:?}", e);
                }

                args.tracy.then(frame_mark);
            }
            Ok(false) => {
                // More packets needed to complete frame
            }
            Err(e) => {
                debug!("Packet processing error: {:?}", e);
            }
        }
    }
}

/// Gets the current wall-clock timestamp for message headers.
///
/// On Y2038 overflow, logs a warning and returns a saturated timestamp so
/// data continues publishing. Returns `None` only if the system clock is
/// before the Unix epoch (unrecoverable).
fn get_stamp() -> Option<Time> {
    match lidar::timestamp() {
        Ok(ns) => Some(Time::from_nanos(ns)),
        Err(lidar::Error::TimestampOverflow) => {
            warn!("Timestamp overflow: system clock exceeds i32 range (Y2038), saturating");
            Some(Time {
                sec: i32::MAX,
                nanosec: 999_999_999,
            })
        }
        Err(e) => {
            warn!("Failed to get timestamp: {}", e);
            None
        }
    }
}

/// Uses the shared SIMD formatters from the formats module.
#[inline(never)]
fn format_points<F: LidarFrame>(
    frame: &F,
    timestamp: Time,
    frame_id: String,
    mirror_y: bool,
    mirror_z: bool,
) -> Result<(ZBytes, Encoding), CdrError> {
    let n_points = frame.len();
    let cdr = encode_xyzr_pointcloud2_cdr(
        frame.x(),
        frame.y(),
        frame.z(),
        frame.intensity(),
        n_points,
        timestamp,
        frame_id,
        mirror_y,
        mirror_z,
    )?;
    let zbytes = ZBytes::from(cdr);
    let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/PointCloud2");
    Ok((zbytes, enc))
}

async fn tf_static_loop(session: Session, args: Args) {
    let publisher = session
        .declare_publisher("tf_static".to_string())
        .priority(Priority::Background)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    let stamp = get_stamp().unwrap_or_else(|| {
        warn!("tf_static: system clock unavailable, using epoch-zero timestamp");
        Time { sec: 0, nanosec: 0 }
    });
    let cdr = match encode_transform_stamped_cdr(
        stamp,
        args.base_frame_id.as_str(),
        args.frame_id.as_str(),
        Vector3 {
            x: args.tf_vec[0],
            y: args.tf_vec[1],
            z: args.tf_vec[2],
        },
        Quaternion {
            x: args.tf_quat[0],
            y: args.tf_quat[1],
            z: args.tf_quat[2],
            w: args.tf_quat[3],
        },
    ) {
        Ok(cdr) => cdr,
        Err(e) => {
            error!("TransformStamped encode failed: {e}");
            return;
        }
    };

    let msg = ZBytes::from(cdr);
    let enc = Encoding::APPLICATION_CDR.with_schema("geometry_msgs/msg/TransformStamped");

    let interval = Duration::from_secs(1);
    let mut target_time = Instant::now() + interval;

    loop {
        publisher
            .put(msg.clone())
            .encoding(enc.clone())
            .await
            .unwrap();
        trace!("lidarpub publishing tf_static");
        sleep(target_time.duration_since(Instant::now()));
        target_time += interval
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use edgefirst_schemas::sensor_msgs::PointCloud2;
    use lidar::LidarFrameWriter;

    fn sample_frame() -> RobosenseLidarFrame {
        let mut frame = RobosenseLidarFrame::with_capacity(4);
        assert!(frame.push(1.0, 2.0, 3.0, 128, 3.74));
        assert!(frame.push(4.0, 5.0, 6.0, 64, 8.77));
        frame
    }

    #[test]
    fn format_points_encodes_pointcloud2() {
        let frame = sample_frame();
        let stamp = Time { sec: 1, nanosec: 2 };
        let (zbytes, enc) =
            format_points(&frame, stamp, "lidar".to_string(), false, false).unwrap();
        assert!(enc.to_string().contains("PointCloud2"));

        let cdr = zbytes.to_bytes().into_owned();
        let pc = PointCloud2::from_cdr(cdr).unwrap();
        assert_eq!(pc.frame_id(), "lidar");
        assert_eq!(pc.width(), 2);
        assert_eq!(pc.point_step(), 13);
        let y0 = f32::from_le_bytes(pc.data()[4..8].try_into().unwrap());
        assert_eq!(y0, 2.0);
    }

    #[test]
    fn format_points_mirrors_y() {
        let frame = sample_frame();
        let stamp = Time { sec: 0, nanosec: 0 };
        let (zbytes, _) = format_points(&frame, stamp, "lidar".to_string(), true, false).unwrap();
        let cdr = zbytes.to_bytes().into_owned();
        let pc = PointCloud2::from_cdr(cdr).unwrap();
        let y0 = f32::from_le_bytes(pc.data()[4..8].try_into().unwrap());
        assert_eq!(y0, -2.0);
    }
}
