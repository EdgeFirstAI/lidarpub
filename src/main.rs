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
#[cfg(target_os = "linux")]
mod clock;
mod cluster_thread;
mod common;
mod formats;
mod lidar;
mod net;
mod ouster;
mod robosense;
mod stamp;

use args::{Args, KEEP, scrub_empty_env};
use clap::Parser as _;
use cluster_thread::cluster_thread;
use common::TimestampMode;
use edgefirst_schemas::{
    builtin_interfaces::Time,
    cdr::CdrError,
    geometry_msgs::{Quaternion, Vector3},
};
use formats::{encode_imu_cdr, encode_transform_stamped_cdr, encode_xyzr_pointcloud2_cdr};
use lidar::{LidarDriver, LidarFrame, SensorType};
use ouster::{BeamIntrinsics, Config, LidarDataFormat, OusterLidarFrame, Parameters, SensorInfo};
use robosense::{RobosenseDriver, RobosenseLidarFrame};
use stamp::system_time_ns;
use std::{
    collections::HashMap,
    io::{IsTerminal as _, Write as _},
    net::TcpStream,
    os::fd::AsRawFd as _,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    thread::sleep,
    time::{Duration, Instant, SystemTime},
};

use tokio::{io::Interest, net::UdpSocket};
use tracing::{debug, error, info, trace, warn};
use tracing_subscriber::{Layer as _, Registry, layer::SubscriberExt as _};
use tracy_client::frame_mark;
use zenoh::{
    Session,
    bytes::{Encoding, ZBytes},
    qos::{CongestionControl, Priority},
    time::TimestampId,
};

#[cfg(feature = "profiling")]
#[global_allocator]
static GLOBAL: tracy_client::ProfiledAllocator<std::alloc::System> =
    tracy_client::ProfiledAllocator::new(std::alloc::System, 100);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // SAFETY: single-threaded here; runs before the runtime is built below.
    unsafe { scrub_empty_env::<Args>(KEEP) };

    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?
        .block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
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

    if args.timestamp_mode == TimestampMode::Ptp1588 {
        info!(
            "Ouster timestamp mode ptp1588: frames use the sensor clock while it is PTP-synchronized"
        );
        let target = target.to_owned();
        let shared = Arc::new(OusterPtpShared {
            synced: driver.sensor_synced(),
            restarts: AtomicU64::new(0),
        });
        spawn_named("ouster-ptp", {
            let target = target.clone();
            let shared = shared.clone();
            move || ouster_ptp_monitor(target, shared)
        });
        spawn_named("clock-step", move || {
            clock_step_monitor(Some((target, shared)))
        });
    } else {
        info!(
            "Ouster timestamp mode {}: frames use the host receive time",
            args.timestamp_mode
        );
        spawn_named("clock-step", || clock_step_monitor(None));
    }

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

    // Parse target as source IP filter for Robosense MSOP and DIFOP packets
    let source_filter: Option<std::net::IpAddr> = args
        .target
        .as_deref()
        .filter(|t| !t.is_empty())
        .map(|t| t.parse())
        .transpose()
        .map_err(|e| format!("Invalid target IP address: {}", e))?;

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

    // Confirm the DIFOP socket is bound before starting the MSOP loop
    let (startup_tx, startup_rx) = tokio::sync::oneshot::channel();
    let session_difop = session.clone();

    tokio::spawn(async move {
        let bind_addr = format!("0.0.0.0:{}", difop_port);
        let sock = match bind_udp(&bind_addr, None) {
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

        let ts_id = common::timestamp_id(&session_difop);
        let mut receiver = net::Receiver::new(4, 512);
        let mut rx_monitor = RxMonitor::new("DIFOP");
        let mut logged_imu_raw = false;
        let mut logged_device_info = false;
        let mut last_device_info: Option<robosense::DeviceInfo> = None;

        loop {
            match sock
                .async_io(Interest::READABLE, || receiver.recv(sock.as_raw_fd()))
                .await
            {
                Ok(_) => rx_monitor.recovered(),
                Err(e) => {
                    if let Some(pause) = rx_monitor.error(&e) {
                        tokio::time::sleep(pause).await;
                    }
                    continue;
                }
            }

            for datagram in receiver.datagrams() {
                // Another sensor's DIFOP must not change this sensor's
                // synchronization state or publish its IMU.
                if source_filter.is_some_and(|ip| datagram.source != Some(ip)) {
                    continue;
                }
                if !rx_monitor.accept(&datagram) {
                    continue;
                }

                // Extract device info under lock, then release before await
                let info = {
                    let Ok(mut driver) = difop_driver.lock() else {
                        continue;
                    };
                    if let Err(e) = driver.process_difop(datagram.data) {
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
                } else if let Some(last) = &last_device_info
                    && (last.timesync_mode, last.timesync_status)
                        != (info.timesync_mode, info.timesync_status)
                {
                    info!(
                        timesync_mode = ?info.timesync_mode,
                        timesync_status = ?info.timesync_status,
                        "Robosense E1R time synchronization changed"
                    );
                } else if last_device_info.as_ref() != Some(&info) {
                    trace!(
                        serial = %info.serial_string(),
                        firmware = %info.version_string(),
                        "DIFOP device info updated"
                    );
                }

                // Store latest IMU accel for ground plane filtering, mapped
                // to the LiDAR frame as (imu_z, -imu_x, -imu_y); IMU +Y is
                // gravity (LiDAR -Z).
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

                // Publish IMU data stamped with the DIFOP receive time
                if let Some(imu) = &info.imu {
                    let stamp = stamp_from_ns(system_time_ns(datagram.rx_time));
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
                            let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/Imu");
                            if let Err(e) =
                                publish_cdr(&imu_publisher, zbytes, enc, ts_id, &stamp).await
                            {
                                debug!("IMU publish error: {:?}", e);
                            }
                        }
                        Err(e) => debug!("IMU encode error: {:?}", e),
                    }
                }

                last_device_info = Some(info);
            }
        }
    });

    // Wait for DIFOP startup confirmation
    match startup_rx.await {
        Ok(Ok(())) => info!("DIFOP listener started on port {}", difop_port),
        Ok(Err(e)) => warn!("DIFOP bind failed: {} (continuing without DIFOP)", e),
        Err(_) => warn!("DIFOP startup channel dropped"),
    }

    let bind_addr = format!("0.0.0.0:{}", args.msop_port);
    info!("Listening for MSOP packets on port {}", args.msop_port);

    // The E1R follows grandmaster steps on its own; host steps are only logged.
    spawn_named("clock-step", || clock_step_monitor(None));

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
    fn process_at<F: lidar::LidarFrameWriter>(
        &mut self,
        frame: &mut F,
        data: &[u8],
        rx_time: SystemTime,
    ) -> Result<bool, lidar::Error> {
        // Handle mutex poison gracefully - the DIFOP thread may have panicked
        // but the driver state is likely still valid for packet processing
        match self.inner.lock() {
            Ok(mut driver) => {
                let result = driver.process_at(frame, data, rx_time);
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
                poisoned.into_inner().process_at(frame, data, rx_time)
            }
        }
    }

    fn set_host_latency(&mut self, latency: Duration) {
        match self.inner.lock() {
            Ok(mut driver) => driver.set_host_latency(latency),
            Err(poisoned) => poisoned.into_inner().set_host_latency(latency),
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
        let session_cluster = session.clone();
        match std::thread::Builder::new()
            .name("cluster".to_string())
            .spawn(move || {
                tokio::runtime::Builder::new_multi_thread()
                    .enable_all()
                    .build()
                    .expect("Failed to create clustering runtime")
                    .block_on(cluster_thread(
                        rx_cluster,
                        cluster_publisher,
                        session_cluster,
                        args_,
                    ));
            }) {
            Ok(_) => info!("Clustering thread started"),
            Err(e) => {
                error!("Could not start clustering thread: {:?}", e);
                std::process::exit(1);
            }
        };
    }

    common::set_process_priority();
    let sock = bind_udp(bind_addr, Some(LIDAR_RECV_BUFFER))?;
    let mut receiver = net::Receiver::new(LIDAR_RECV_BATCH, net::MAX_DATAGRAM);
    let ts_id = common::timestamp_id(&session);
    let mut rx_monitor = RxMonitor::new("LiDAR");

    driver.set_host_latency(Duration::from_nanos(args.lidar_latency));

    if let Some(filter_ip) = source_filter {
        info!("Filtering packets from source IP: {}", filter_ip);
    }
    info!("Starting LiDAR processing loop");

    loop {
        match sock
            .async_io(Interest::READABLE, || receiver.recv(sock.as_raw_fd()))
            .await
        {
            Ok(_) => rx_monitor.recovered(),
            Err(e) => {
                if let Some(pause) = rx_monitor.error(&e) {
                    tokio::time::sleep(pause).await;
                }
                continue;
            }
        }

        for datagram in receiver.datagrams() {
            // Filter by source IP if configured
            if let Some(filter_ip) = source_filter
                && datagram.source != Some(filter_ip)
            {
                continue;
            }

            if !rx_monitor.accept(&datagram) {
                continue;
            }

            // Process packet into client-owned frame
            match driver.process_at(&mut frame, datagram.data, datagram.rx_time) {
                Ok(true) => {
                    publish_frame(
                        &frame,
                        &args,
                        &points_publisher,
                        ts_id,
                        &tx_cluster,
                        &latest_imu,
                    )
                    .await?;
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
}

/// Frame data sent to the clustering thread: ranges, points, stamp and the
/// latest IMU acceleration when the ground filter is enabled.
type ClusterInput = (Vec<f32>, lidar::Points, Time, Option<(f32, f32, f32)>);

/// Publishes a completed frame as a point cloud and hands it to the
/// clustering thread when clustering is enabled.
async fn publish_frame<F: LidarFrame>(
    frame: &F,
    args: &Args,
    points_publisher: &zenoh::pubsub::Publisher<'_>,
    ts_id: TimestampId,
    tx_cluster: &kanal::Sender<ClusterInput>,
    latest_imu: &Mutex<Option<(f32, f32, f32)>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let timestamp_ns = frame.timestamp();
    trace!(
        timestamp = timestamp_ns,
        frame_id = frame.frame_id(),
        n_points = frame.len(),
        "publishing frame"
    );

    let timestamp = stamp_from_ns(timestamp_ns);

    if args.clustering_enabled() {
        // Range comes from the driver, no sqrt needed
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

    let (msg, enc) = format_points(
        frame,
        timestamp,
        args.frame_id.clone(),
        args.mirror_y(),
        args.mirror_z(),
    )?;

    if let Err(e) = publish_cdr(points_publisher, msg, enc, ts_id, &timestamp).await {
        error!("publish points error: {:?}", e);
    }

    args.tracy.then(frame_mark);
    Ok(())
}

/// Requested receive buffer for the LiDAR data socket, about 3 s of E1R
/// data or 1.5 s of Ouster 1024x20 data.
const LIDAR_RECV_BUFFER: usize = 16 * 1024 * 1024;

/// Maximum datagrams read per receive call on the LiDAR data socket.
const LIDAR_RECV_BATCH: usize = 32;

/// Time after binding during which datagrams without a kernel receive
/// timestamp are expected, while the kernel enables timestamping.
const RX_STAMP_GRACE: Duration = Duration::from_secs(1);

/// Pause after a receive error before the next attempt.
const RX_ERROR_PAUSE: Duration = Duration::from_millis(10);

/// Receive-path conditions that are logged once or rate-limited rather than
/// per datagram.
struct RxMonitor {
    name: &'static str,
    started: Instant,
    logged_unstamped: bool,
    logged_truncated: bool,
    error_streak: u64,
}

impl RxMonitor {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            started: Instant::now(),
            logged_unstamped: false,
            logged_truncated: false,
            error_streak: 0,
        }
    }

    /// Returns whether a datagram should be processed. Truncated datagrams
    /// are dropped.
    fn accept(&mut self, datagram: &net::Datagram) -> bool {
        if datagram.truncated {
            if !self.logged_truncated {
                warn!(
                    "{}: dropping UDP datagrams larger than {} bytes",
                    self.name,
                    datagram.data.len()
                );
                self.logged_truncated = true;
            }
            return false;
        }
        if !datagram.kernel_stamped
            && !self.logged_unstamped
            && self.started.elapsed() > RX_STAMP_GRACE
        {
            warn!(
                "{}: datagram without a kernel receive timestamp, stamped when read",
                self.name
            );
            self.logged_unstamped = true;
        }
        true
    }

    /// Logs a receive error (the first of a run, then every 1000th) and
    /// returns how long to pause before the next attempt.
    fn error(&mut self, err: &std::io::Error) -> Option<Duration> {
        if err.kind() == std::io::ErrorKind::Interrupted {
            return None;
        }
        self.error_streak += 1;
        if self.error_streak == 1 || self.error_streak.is_multiple_of(1000) {
            error!(
                "{}: UDP receive error ({} in a row): {err}",
                self.name, self.error_streak
            );
        }
        Some(RX_ERROR_PAUSE)
    }

    /// Records a successful receive, logging the end of an error run.
    fn recovered(&mut self) {
        if self.error_streak > 0 {
            info!(
                "{}: UDP receive recovered after {} errors",
                self.name, self.error_streak
            );
            self.error_streak = 0;
        }
    }
}

/// Binds a non-blocking UDP socket with kernel receive timestamps and, when
/// `recv_buffer` is given, a receive buffer of that size.
///
/// Missing receive timestamps or a smaller buffer are logged, not errors.
fn bind_udp(addr: &str, recv_buffer: Option<usize>) -> std::io::Result<UdpSocket> {
    let sock = std::net::UdpSocket::bind(addr)?;
    sock.set_nonblocking(true)?;
    let fd = sock.as_raw_fd();

    if let Some(size) = recv_buffer {
        match net::set_recv_buffer(fd, size) {
            Ok(granted) if granted < size => warn!(
                "{addr}: UDP receive buffer is {granted} bytes, requested {size}; \
                 run with CAP_NET_ADMIN or raise net.core.rmem_max"
            ),
            Ok(granted) => debug!("{addr}: UDP receive buffer is {granted} bytes"),
            Err(e) => warn!("{addr}: cannot set the UDP receive buffer: {e}"),
        }
    }

    if let Err(e) = net::enable_rx_timestamps(fd) {
        warn!(
            "{addr}: kernel receive timestamps unavailable, using the clock after each receive: {e}"
        );
    }

    UdpSocket::from_std(sock)
}

/// Converts nanoseconds since the Unix epoch to a message stamp, saturating
/// past the `i32` seconds range (Y2038).
fn stamp_from_ns(ns: u64) -> Time {
    let sec = ns / 1_000_000_000;
    match i32::try_from(sec) {
        Ok(sec) => Time {
            sec,
            nanosec: (ns % 1_000_000_000) as u32,
        },
        Err(_) => {
            static WARNED: AtomicBool = AtomicBool::new(false);
            if !WARNED.swap(true, Ordering::Relaxed) {
                warn!("Timestamp overflow: stamp exceeds i32 range (Y2038), saturating");
            }
            Time {
                sec: i32::MAX,
                nanosec: 999_999_999,
            }
        }
    }
}

/// Ouster PTP offset from master below which the sensor clock can become
/// the stamp source.
const OUSTER_PTP_ENTER_OFFSET_NS: f64 = 1_000_000.0;

/// Ouster PTP offset from master above which the sensor clock stops being the
/// stamp source.
const OUSTER_PTP_EXIT_OFFSET_NS: f64 = 2_000_000.0;

/// Consecutive polls within [`OUSTER_PTP_ENTER_OFFSET_NS`] required before
/// the sensor clock becomes the stamp source, so a converging PTP servo does
/// not switch the source back and forth.
const OUSTER_PTP_ENTER_POLLS: u32 = 5;

/// Consecutive failed polls after which a synchronized Ouster clock stops
/// being the stamp source; fewer failures keep the current state.
const OUSTER_PTP_MAX_FAILED_POLLS: u32 = 3;

/// Longest wait between attempts to restart the Ouster PTP client.
#[cfg(target_os = "linux")]
const OUSTER_PTP_RESTART_MAX_BACKOFF: Duration = Duration::from_secs(30);

/// Interval between Ouster PTP status polls.
const OUSTER_PTP_POLL: Duration = Duration::from_secs(1);

/// Host clock steps at least this large restart the Ouster PTP client. The
/// Ouster steps its clock only when its PTP client starts and slews any later
/// offset at well under 1 ms/s, while a restart converges in about 25 s.
#[cfg(target_os = "linux")]
const OUSTER_PTP_RESTART_STEP_NS: i128 = 10_000_000;

/// Host clock steps at least this large are logged at INFO level.
#[cfg(target_os = "linux")]
const CLOCK_STEP_LOG_NS: i128 = 1_000_000;

/// Spawns a named background thread, logging when it cannot be started.
fn spawn_named(name: &str, f: impl FnOnce() + Send + 'static) {
    if let Err(e) = std::thread::Builder::new().name(name.to_owned()).spawn(f) {
        error!("could not start the {name} thread: {e}");
    }
}

/// HTTP agent for the Ouster API with a bounded request time.
fn ouster_agent() -> ureq::Agent {
    ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(2)))
        .build()
        .into()
}

/// PTP state reported by an Ouster sensor at `/api/v1/time/ptp`.
#[derive(Debug, PartialEq)]
struct OusterPtpStatus {
    profile: String,
    port_state: String,
    offset_ns: f64,
}

impl OusterPtpStatus {
    fn from_json(value: &serde_json::Value) -> Option<Self> {
        Some(Self {
            profile: value["profile"].as_str().unwrap_or_default().to_owned(),
            port_state: value["port_data_set"]["port_state"].as_str()?.to_owned(),
            offset_ns: value["current_data_set"]["offset_from_master"].as_f64()?,
        })
    }

    fn locked_within(&self, offset_ns: f64) -> bool {
        self.port_state == "SLAVE" && self.offset_ns.abs() < offset_ns
    }
}

/// Decides from successive PTP polls whether the Ouster clock is
/// synchronized, with hysteresis between entering and leaving.
#[derive(Debug, Default)]
struct OusterPtpTracker {
    synced: bool,
    streak: u32,
    failures: u32,
}

impl OusterPtpTracker {
    /// Updates the state with a poll result (`None` when the poll failed) and
    /// returns whether the sensor clock is synchronized.
    fn update(&mut self, status: Option<&OusterPtpStatus>) -> bool {
        if status.is_none() {
            self.failures += 1;
            if self.synced && self.failures < OUSTER_PTP_MAX_FAILED_POLLS {
                return true;
            }
        } else {
            self.failures = 0;
        }
        let within_enter = status.is_some_and(|s| s.locked_within(OUSTER_PTP_ENTER_OFFSET_NS));
        let within_exit = status.is_some_and(|s| s.locked_within(OUSTER_PTP_EXIT_OFFSET_NS));
        self.streak = if within_enter { self.streak + 1 } else { 0 };
        self.synced = if self.synced {
            within_exit
        } else {
            self.streak >= OUSTER_PTP_ENTER_POLLS
        };
        self.synced
    }

    /// Forgets the synchronized state, for example after the sensor's PTP
    /// client was restarted.
    fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Shared state between the Ouster PTP monitor, the clock step monitor and
/// the driver.
struct OusterPtpShared {
    /// Set while frames may be stamped from the sensor clock.
    synced: Arc<AtomicBool>,
    /// Incremented by the clock step monitor when it restarts the sensor's
    /// PTP client, so the PTP monitor discards polls that straddle a restart.
    restarts: AtomicU64,
}

fn ouster_ptp_status(
    agent: &ureq::Agent,
    target: &str,
) -> Result<OusterPtpStatus, Box<dyn std::error::Error>> {
    let value = agent
        .get(&format!("http://{target}/api/v1/time/ptp"))
        .call()?
        .body_mut()
        .read_json::<serde_json::Value>()?;
    OusterPtpStatus::from_json(&value).ok_or_else(|| "unexpected PTP status format".into())
}

/// Polls the Ouster PTP state and sets `synced` while the sensor clock is
/// synchronized to its grandmaster.
fn ouster_ptp_monitor(target: String, shared: Arc<OusterPtpShared>) {
    let agent = ouster_agent();
    let mut tracker = OusterPtpTracker::default();
    let mut seen_restarts = 0;
    let mut last: Option<(Option<String>, bool)> = None;
    loop {
        let before = shared.restarts.load(Ordering::SeqCst);
        let status = ouster_ptp_status(&agent, &target);
        let after = shared.restarts.load(Ordering::SeqCst);
        if after != seen_restarts {
            tracker.reset();
            seen_restarts = after;
        }
        // A poll that straddles a restart may describe the sensor before the
        // host step and is not trusted.
        let now_synced = before == after && tracker.update(status.as_ref().ok());
        shared.synced.store(now_synced, Ordering::SeqCst);
        // A restart after the check above must win over this store.
        if shared.restarts.load(Ordering::SeqCst) != after {
            shared.synced.store(false, Ordering::SeqCst);
        }

        let state = (
            status.as_ref().ok().map(|s| s.port_state.clone()),
            now_synced,
        );
        if last.as_ref() != Some(&state) {
            match &status {
                Ok(status) => info!(
                    profile = %status.profile,
                    port_state = %status.port_state,
                    offset_ms = status.offset_ns / 1e6,
                    synced = now_synced,
                    "Ouster PTP state"
                ),
                Err(e) => warn!("Ouster PTP state unavailable: {e}"),
            }
            last = Some(state);
        }

        sleep(OUSTER_PTP_POLL);
    }
}

/// Restarts the Ouster PTP client by re-applying its current profile, so it
/// steps its clock to the grandmaster again. Returns the profile.
fn restart_ouster_ptp(
    agent: &ureq::Agent,
    target: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    let url = format!("http://{target}/api/v1/time/ptp/profile");
    let profile = agent.get(&url).call()?.body_mut().read_json::<String>()?;
    agent.put(&url).send_json(&profile)?;
    Ok(profile)
}

/// Logs host clock steps and, for an Ouster in PTP mode, restarts its PTP
/// client after a step so the sensor clock follows the host again.
#[cfg(target_os = "linux")]
fn clock_step_monitor(ouster_ptp: Option<(String, Arc<OusterPtpShared>)>) {
    let mut watcher = match clock::ClockStepWatcher::new() {
        Ok(watcher) => watcher,
        Err(e) => {
            error!(
                "cannot watch for host clock steps, so they are not logged{}: {e}",
                restart_note(&ouster_ptp)
            );
            return;
        }
    };
    let agent = ouster_agent();

    loop {
        let step_ns = match watcher.wait() {
            Ok(step_ns) => step_ns,
            Err(e) => {
                error!(
                    "stopped watching for host clock steps, so they are no longer logged{}: {e}",
                    restart_note(&ouster_ptp)
                );
                return;
            }
        };

        if step_ns.abs() >= CLOCK_STEP_LOG_NS {
            info!(step_s = step_ns as f64 / 1e9, "host clock stepped");
        } else {
            debug!(step_ns = step_ns as i64, "host clock set");
        }

        if let Some((target, shared)) = &ouster_ptp
            && step_ns.abs() >= OUSTER_PTP_RESTART_STEP_NS
        {
            shared.restarts.fetch_add(1, Ordering::SeqCst);
            shared.synced.store(false, Ordering::SeqCst);
            restart_ouster_ptp_until_done(&agent, target);
        }
    }
}

/// Restarts the Ouster PTP client, retrying with backoff until it succeeds.
#[cfg(target_os = "linux")]
fn restart_ouster_ptp_until_done(agent: &ureq::Agent, target: &str) {
    let mut backoff = Duration::from_secs(1);
    let mut failures = 0u32;
    loop {
        match restart_ouster_ptp(agent, target) {
            Ok(profile) => {
                info!(
                    profile = %profile,
                    failed_attempts = failures,
                    "restarted the Ouster PTP client to follow the host clock step"
                );
                return;
            }
            Err(e) => {
                if failures == 0 {
                    warn!("could not restart the Ouster PTP client, retrying: {e}");
                }
                failures += 1;
            }
        }
        sleep(backoff);
        backoff = (backoff * 2).min(OUSTER_PTP_RESTART_MAX_BACKOFF);
    }
}

/// Consequence of losing clock step handling, for the log.
#[cfg(target_os = "linux")]
fn restart_note(ouster_ptp: &Option<(String, Arc<OusterPtpShared>)>) -> &'static str {
    match ouster_ptp {
        Some(_) => " and the Ouster PTP client is not restarted after them",
        None => "",
    }
}

#[cfg(not(target_os = "linux"))]
fn clock_step_monitor(_ouster_ptp: Option<(String, Arc<OusterPtpShared>)>) {}

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

/// Put a CDR payload whose Zenoh sample timestamp equals its `stamp`.
async fn publish_cdr(
    publisher: &zenoh::pubsub::Publisher<'_>,
    payload: impl Into<ZBytes>,
    encoding: Encoding,
    ts_id: TimestampId,
    stamp: &Time,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    publisher
        .put(payload)
        .encoding(encoding)
        .timestamp(common::zenoh_timestamp(ts_id, stamp))
        .await
}

async fn tf_static_loop(session: Session, args: Args) {
    let publisher = session
        .declare_publisher("tf_static".to_string())
        .priority(Priority::Background)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    let ts_id = common::timestamp_id(&session);
    let enc = Encoding::APPLICATION_CDR.with_schema("geometry_msgs/msg/TransformStamped");
    let interval = Duration::from_secs(1);
    let mut target_time = Instant::now() + interval;
    let mut warned_epoch = false;
    let mut publish_failing = false;

    loop {
        // Re-stamp at each republish so the stamp follows host clock steps.
        let stamp = get_stamp().unwrap_or_else(|| {
            if !warned_epoch {
                warn!("tf_static: system clock unavailable, using epoch-zero timestamp");
                warned_epoch = true;
            }
            Time { sec: 0, nanosec: 0 }
        });
        match encode_transform_stamped_cdr(
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
            Ok(cdr) => {
                match publish_cdr(&publisher, ZBytes::from(cdr), enc.clone(), ts_id, &stamp).await {
                    Ok(()) if publish_failing => {
                        info!("tf_static publishing recovered");
                        publish_failing = false;
                    }
                    Ok(()) => {}
                    Err(e) if !publish_failing => {
                        warn!("tf_static publish error: {e:?}");
                        publish_failing = true;
                    }
                    Err(_) => {}
                }
                trace!("lidarpub publishing tf_static");
            }
            Err(e) => error!("TransformStamped encode failed: {e}"),
        }
        tokio::time::sleep(target_time.saturating_duration_since(Instant::now())).await;
        target_time += interval;
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

    fn test_zenoh_config() -> zenoh::Config {
        let mut config = zenoh::Config::default();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        config
            .insert_json5("listen/endpoints", r#"["tcp/127.0.0.1:0"]"#)
            .unwrap();
        config
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn publish_cdr_timestamp_equals_stamp() {
        let session = zenoh::open(test_zenoh_config()).await.unwrap();
        let key = format!("lidarpub/test/publish_cdr/{}", std::process::id());
        let subscriber = session.declare_subscriber(key.clone()).await.unwrap();
        let publisher = session.declare_publisher(key).await.unwrap();

        let payload = ZBytes::from(vec![1u8, 2, 3, 4]);
        let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/PointCloud2");
        let stamp = Time {
            sec: 1_234_567_890,
            nanosec: 123_456_789,
        };
        publish_cdr(
            &publisher,
            payload,
            enc,
            common::timestamp_id(&session),
            &stamp,
        )
        .await
        .expect("publish_cdr");

        let sample = tokio::time::timeout(Duration::from_secs(5), subscriber.recv_async())
            .await
            .expect("timed out waiting for sample")
            .expect("recv sample");
        let ts = sample
            .timestamp()
            .expect("published sample should carry a Zenoh source timestamp")
            .get_time()
            .to_duration();
        let expected = Duration::new(stamp.sec as u64, stamp.nanosec);
        assert!(ts.abs_diff(expected) <= Duration::from_nanos(1), "{ts:?}");
        assert_eq!(sample.payload().to_bytes().as_ref(), &[1u8, 2, 3, 4]);
    }

    fn ptp_json(port_state: &str, offset: f64) -> serde_json::Value {
        serde_json::json!({
            "profile": "default",
            "port_data_set": { "port_state": port_state },
            "current_data_set": { "offset_from_master": offset },
        })
    }

    fn status(port_state: &str, offset: f64) -> OusterPtpStatus {
        OusterPtpStatus::from_json(&ptp_json(port_state, offset)).unwrap()
    }

    #[test]
    fn ouster_ptp_status_parses_api_fields() {
        let locked = status("SLAVE", -250_000.0);
        assert_eq!(locked.profile, "default");
        assert_eq!(locked.port_state, "SLAVE");
        assert!(locked.locked_within(OUSTER_PTP_ENTER_OFFSET_NS));
        // Slewing after a host step: still SLAVE, far from the grandmaster.
        assert!(!status("SLAVE", 3.1e9).locked_within(OUSTER_PTP_EXIT_OFFSET_NS));
        // PTP version mismatch leaves the port uncalibrated.
        assert!(!status("UNCALIBRATED", 0.0).locked_within(OUSTER_PTP_EXIT_OFFSET_NS));
        assert!(OusterPtpStatus::from_json(&serde_json::json!({})).is_none());
    }

    #[test]
    fn ouster_ptp_tracker_enters_after_consecutive_polls() {
        let near = status("SLAVE", 400_000.0);
        let mut tracker = OusterPtpTracker::default();
        for _ in 1..OUSTER_PTP_ENTER_POLLS {
            assert!(!tracker.update(Some(&near)));
        }
        assert!(tracker.update(Some(&near)));

        // A converging servo crossing the threshold restarts the count.
        let mut tracker = OusterPtpTracker::default();
        for _ in 1..OUSTER_PTP_ENTER_POLLS {
            tracker.update(Some(&near));
        }
        assert!(!tracker.update(Some(&status("SLAVE", 7_700_000.0))));
        assert!(!tracker.update(Some(&near)));
    }

    #[test]
    fn ouster_ptp_tracker_leaves_on_large_offset_state_or_error() {
        let near = status("SLAVE", 400_000.0);
        let synced = || {
            let mut tracker = OusterPtpTracker::default();
            for _ in 0..OUSTER_PTP_ENTER_POLLS {
                tracker.update(Some(&near));
            }
            tracker
        };

        // Between the enter and exit thresholds: stays synchronized.
        let mut tracker = synced();
        assert!(tracker.update(Some(&status("SLAVE", -1_500_000.0))));
        assert!(!tracker.update(Some(&status("SLAVE", 2_500_000.0))));

        assert!(!synced().update(Some(&status("LISTENING", 0.0))));

        // A few failed polls are tolerated; the last of the limit leaves.
        let mut tracker = synced();
        for _ in 1..OUSTER_PTP_MAX_FAILED_POLLS {
            assert!(tracker.update(None));
        }
        assert!(!tracker.update(None));
        // Failed polls never enter the synchronized state.
        assert!(!OusterPtpTracker::default().update(None));

        let mut tracker = synced();
        tracker.reset();
        assert!(!tracker.update(Some(&near)));
    }

    fn datagram(data: &[u8], truncated: bool, kernel_stamped: bool) -> net::Datagram<'_> {
        net::Datagram {
            data,
            rx_time: SystemTime::now(),
            source: None,
            truncated,
            kernel_stamped,
        }
    }

    #[test]
    fn rx_monitor_drops_truncated_and_keeps_unstamped() {
        let mut monitor = RxMonitor::new("test");
        assert!(monitor.accept(&datagram(&[0; 8], false, true)));
        assert!(!monitor.accept(&datagram(&[0; 8], true, true)));
        assert!(monitor.accept(&datagram(&[0; 8], false, false)));
    }

    #[test]
    fn rx_monitor_pauses_on_errors_except_interrupts() {
        let mut monitor = RxMonitor::new("test");
        let interrupted = std::io::Error::from(std::io::ErrorKind::Interrupted);
        assert_eq!(monitor.error(&interrupted), None);
        let refused = std::io::Error::from(std::io::ErrorKind::ConnectionRefused);
        assert_eq!(monitor.error(&refused), Some(RX_ERROR_PAUSE));
        assert_eq!(monitor.error_streak, 1);
        monitor.recovered();
        assert_eq!(monitor.error_streak, 0);
    }

    #[test]
    fn stamp_from_ns_splits_and_saturates() {
        assert_eq!(
            stamp_from_ns(1_790_000_000_123_456_789),
            Time {
                sec: 1_790_000_000,
                nanosec: 123_456_789
            }
        );
        assert_eq!(
            stamp_from_ns(u64::MAX),
            Time {
                sec: i32::MAX,
                nanosec: 999_999_999
            }
        );
    }
}
