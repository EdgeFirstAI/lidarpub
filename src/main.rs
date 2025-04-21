#![feature(portable_simd)]

mod args;
mod cluster;
mod common;

use args::Args;
use cdr::{CdrLe, Infinite};
use clap::Parser as _;
use cluster::cluster_thread;
use edgefirst_schemas::{
    builtin_interfaces::Time,
    geometry_msgs::{Quaternion, Transform, TransformStamped, Vector3},
    sensor_msgs::{Image, PointCloud2, PointField},
    std_msgs::Header,
};
use kanal::Receiver;
use lidarpub::ouster::{
    BeamIntrinsics, Config, FrameBuilder, FrameReader, LidarDataFormat, Parameters, Points,
    SensorInfo,
};
use ndarray::Array2;
use std::{
    io::{IsTerminal as _, Write as _},
    net::TcpStream,
    simd::{Simd, ToBytes as _},
    thread::sleep,
    time::{Duration, Instant, SystemTime},
};
use tokio::net::UdpSocket;
use tracing::{debug, error, info, info_span, trace, Instrument};
use tracing_subscriber::{layer::SubscriberExt as _, Layer as _, Registry};
use tracy_client::frame_mark;
use zenoh::{
    bytes::{Encoding, ZBytes},
    qos::{CongestionControl, Priority},
    Session,
};

#[cfg(feature = "profiling")]
#[global_allocator]
static GLOBAL: tracy_client::ProfiledAllocator<std::alloc::System> =
    tracy_client::ProfiledAllocator::new(std::alloc::System, 100);

#[derive(Debug)]
#[allow(dead_code)]
pub enum PointFieldType {
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8,
}

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
    let mut frame_reader = FrameReader::new(&params.lidar_data_format)?;

    // On Linux [::] will bind to IPv4 and IPv6 but not on Windows so we bind
    // according to the local address IP version.
    let bind_addr = match local.is_ipv4() {
        true => format!("0.0.0.0:{}", config.udp_port_lidar),
        false => format!("[::]:{}", config.udp_port_lidar),
    };

    common::set_process_priority();
    let sock = UdpSocket::bind(bind_addr).await?;
    let sock = common::set_socket_bufsize(sock.into_std()?, 16 * 1024 * 1024);
    let sock = UdpSocket::from_std(sock)?;

    let mut buf = [0u8; 16 * 1024];
    let (tx, rx) = kanal::bounded(128);

    std::thread::Builder::new()
        .name("processor".to_string())
        .spawn(move || {
            tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(frame_processor(session, args, params, rx));
        })?;

    loop {
        let len = match sock.recv_from(&mut buf).await {
            Ok((len, _)) => len,
            Err(e) => {
                error!("udp_read recv error: {:?}", e);
                continue;
            }
        };

        frame_reader.update(&buf[..len], |timestamp, frame_id, depth, reflect| match tx
            .send((timestamp, frame_id, depth, reflect))
        {
            Ok(_) => {}
            Err(e) => error!("frame send error: {:?}", e),
        })?;
    }
}

async fn frame_processor(
    session: Session,
    args: Args,
    params: Parameters,
    rx: Receiver<(u64, u16, Array2<u16>, Array2<u8>)>,
) {
    let points_publisher = session
        .declare_publisher(format!("{}/points", args.lidar_topic))
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    let cluster_publisher = session
        .declare_publisher(format!("{}/cluster", args.lidar_topic))
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    let depth_publisher = session
        .declare_publisher(format!("{}/depth", args.lidar_topic))
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    let reflect_publisher = session
        .declare_publisher(format!("{}/reflect", args.lidar_topic))
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    let mut builder = FrameBuilder::new(&params);

    let (tx_cluster, rx_cluster) = kanal::bounded(8);
    let frame_id = args.frame_id.clone();
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
                    builder.rows,
                    builder.crop.1 - builder.crop.0,
                    frame_id,
                ));
        }) {
        Ok(_) => {}
        Err(e) => error!("Could not start clustering thread: {:?}", e),
    };

    loop {
        let (timestamp, frame_id, depth, reflect) = rx.recv().unwrap();

        let span = info_span!("frame");
        async {
            builder.update(&depth, &reflect);

            trace!(
                timestamp = timestamp,
                frame_id = frame_id,
                n_points = builder.n_points,
                "publishing frame"
            );

            let timestamp = Time::from_nanos(timestamp);
            let _ = tx_cluster.send((
                builder.range[0..builder.n_points].to_vec(),
                builder.points.clone(),
                timestamp.clone(),
            ));
            let publish_points = info_span!("publish_points");
            async {
                let (msg, enc) = format_points(
                    &builder.points,
                    builder.n_points,
                    timestamp.clone(),
                    args.frame_id.clone(),
                )
                .unwrap();
                match points_publisher.put(msg).encoding(enc).await {
                    Ok(_) => {}
                    Err(e) => error!("publish points error: {:?}", e),
                }
            }
            .instrument(publish_points)
            .await;

            let publish_depth = info_span!("publish_depth");
            async {
                let (msg, enc) = format_depth(
                    &builder.depth,
                    &builder.crop,
                    timestamp.clone(),
                    args.frame_id.clone(),
                )
                .unwrap();
                match depth_publisher.put(msg).encoding(enc).await {
                    Ok(_) => {}
                    Err(e) => error!("depth publish error: {:?}", e),
                }
            }
            .instrument(publish_depth)
            .await;

            let publish_reflect = info_span!("publish_reflect");
            async {
                let (msg, enc) = format_reflect(
                    &builder.reflect,
                    &builder.crop,
                    timestamp,
                    args.frame_id.clone(),
                )
                .unwrap();
                match reflect_publisher.put(msg).encoding(enc).await {
                    Ok(_) => {}
                    Err(e) => error!("reflect publish error: {:?}", e),
                }
            }
            .instrument(publish_reflect)
            .await;

            // match tokio::try_join!(points_span, depth_span, reflect_span)
            // {     Ok(_) => trace!("{} message sent",
            // args.lidar_topic),     Err(e) => error!("{}
            // message error: {:?}", args.lidar_topic, e), }
        }
        .instrument(span)
        .await;

        args.tracy.then(frame_mark);
    }
}

#[inline(never)]
fn format_points(
    points: &Points,
    n_points: usize,
    timestamp: Time,
    frame_id: String,
) -> Result<(ZBytes, Encoding), cdr::Error> {
    const N: usize = 4;

    let fields = vec![
        PointField {
            name: String::from("x"),
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: String::from("y"),
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: String::from("z"),
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: String::from("reflect"),
            offset: 12,
            datatype: PointFieldType::UINT8 as u8,
            count: 1,
        },
    ];

    let mut data = vec![0u8; 13 * n_points];

    for index in (0..n_points).step_by(N) {
        let x = Simd::<f32, N>::from_slice(&points.x[index..index + N]);
        let x = x.to_le_bytes();
        let y = Simd::<f32, N>::from_slice(&points.y[index..index + N]);
        let y = y.to_le_bytes();
        let z = Simd::<f32, N>::from_slice(&points.z[index..index + N]);
        let z = z.to_le_bytes();

        let out = index * 13;
        data[out..out + 4].copy_from_slice(&x[..4]);
        data[out + 4..out + 8].copy_from_slice(&y[..4]);
        data[out + 8..out + 12].copy_from_slice(&z[..4]);
        data[out + 12] = points.l[index];

        let out = (index + 1) * 13;
        data[out..out + 4].copy_from_slice(&x[4..8]);
        data[out + 4..out + 8].copy_from_slice(&y[4..8]);
        data[out + 8..out + 12].copy_from_slice(&z[4..8]);
        data[out + 12] = points.l[index];

        let out = (index + 2) * 13;
        data[out..out + 4].copy_from_slice(&x[8..12]);
        data[out + 4..out + 8].copy_from_slice(&y[8..12]);
        data[out + 8..out + 12].copy_from_slice(&z[8..12]);
        data[out + 12] = points.l[index];

        let out = (index + 3) * 13;
        data[out..out + 4].copy_from_slice(&x[12..16]);
        data[out + 4..out + 8].copy_from_slice(&y[12..16]);
        data[out + 8..out + 12].copy_from_slice(&z[12..16]);
        data[out + 12] = points.l[index];
    }

    for index in n_points - n_points % N..n_points {
        let x = points.x[index].to_le_bytes();
        let y = points.y[index].to_le_bytes();
        let z = points.z[index].to_le_bytes();
        let l = points.l[index];

        let out = index * 13;
        data[out..out + 4].copy_from_slice(&x);
        data[out + 4..out + 8].copy_from_slice(&y);
        data[out + 8..out + 12].copy_from_slice(&z);
        data[out + 12] = l;
    }

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

    let msg = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?);
    let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/PointCloud2");

    Ok((msg, enc))
}

#[inline(never)]
fn format_depth(
    depth: &Array2<u16>,
    crop: &(usize, usize),
    timestamp: Time,
    frame_id: String,
) -> Result<(ZBytes, Encoding), cdr::Error> {
    const N: usize = 8;

    let depth = depth
        .slice(ndarray::s![.., crop.0..crop.1])
        .as_standard_layout()
        .to_owned();
    let (height, width) = depth.dim();
    let depth = depth.as_slice().unwrap();
    let mut data = vec![0u8; height * width * 2];

    for index in (0..height * width).step_by(N) {
        let x = Simd::<u16, N>::from_slice(&depth[index..]);
        let x = x.to_le_bytes();

        let out = index * 2;
        x.copy_to_slice(&mut data[out..out + 16]);
    }

    let msg = Image {
        header: Header {
            stamp: timestamp,
            frame_id,
        },
        height: height as u32,
        width: width as u32,
        encoding: "mono16".to_owned(),
        is_bigendian: 0,
        step: width as u32 * 2,
        data,
    };

    let msg = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&msg, Infinite).unwrap());
    let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/Image");

    Ok((msg, enc))
}

#[inline(never)]
fn format_reflect(
    reflect: &Array2<u8>,
    crop: &(usize, usize),
    timestamp: Time,
    frame_id: String,
) -> Result<(ZBytes, Encoding), cdr::Error> {
    let reflect = reflect
        .slice(ndarray::s![.., crop.0..crop.1])
        .as_standard_layout()
        .to_owned();
    let (height, width) = reflect.dim();

    let msg = Image {
        header: Header {
            stamp: timestamp,
            frame_id,
        },
        height: height as u32,
        width: width as u32,
        encoding: "mono8".to_owned(),
        is_bigendian: 0,
        step: width as u32,
        data: reflect.as_slice().unwrap().to_vec(),
    };

    let msg = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&msg, Infinite).unwrap());
    let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/Image");

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

    let msg = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&msg, Infinite).unwrap());
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
