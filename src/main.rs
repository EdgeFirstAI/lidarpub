#![feature(portable_simd)]

use async_std::net::UdpSocket;
use cdr::{CdrLe, Infinite};
use clap::{builder::PossibleValuesParser, Parser};
use edgefirst_schemas::{
    builtin_interfaces::{self, Time},
    geometry_msgs::{Quaternion, Transform, TransformStamped, Vector3},
    sensor_msgs::{Image, PointCloud2, PointField},
    std_msgs::Header,
};
use lidarpub::ouster::{
    BeamIntrinsics, Config, FrameReader, LidarDataFormat, Parameters, Points, SensorInfo,
};
use log::{debug, error, info, trace};
use ndarray::Array2;
use std::{
    io::{IsTerminal as _, Write as _},
    net::TcpStream,
    simd::{Simd, ToBytes as _},
    str::FromStr as _,
    sync::Arc,
    thread::{self, sleep},
    time::{Duration, Instant},
};
use zenoh::prelude::{r#async::*, sync::SyncResolve};

mod common;

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

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Connect to target device or pcap file.  If target is a valid pcap file,
    /// it will be used otherwise it will be tried as a hostname or IP address.
    #[arg(env)]
    target: String,

    /// Azimuth field of view start and stop angles in degrees.  
    /// The 0 degree point is the rear connector of the LiDAR.
    #[arg(long, env, num_args = 2, value_names = ["START", "STOP"], value_delimiter=' ', default_value = "0 360")]
    azimuth: Vec<u32>,

    /// LiDAR column and refresh rate mode.  The format is "COLxHZ".
    #[arg(long, env, default_value = "1024x10", 
          value_parser = PossibleValuesParser::new(["512x10", "1024x10", "2048x10", "512x20", "1024x20",]))]
    mode: String,

    /// Frame transformation vector from the base_link
    #[arg(
        long,
        env,
        default_value = "0 0 0",
        value_delimiter = ' ',
        num_args = 3
    )]
    tf_vec: Vec<f64>,

    /// Frame transformation quaternion from the base_link
    #[arg(
        long,
        env,
        default_value = "0 0 0 1",
        value_delimiter = ' ',
        num_args = 4
    )]
    tf_quat: Vec<f64>,

    /// The name of the base frame
    #[arg(long, env, default_value = "base_link")]
    base_frame_id: String,

    /// The name of the lidar frame
    #[arg(long, env, default_value = "lidar")]
    frame_id: String,

    /// lidar base topic
    #[arg(long, env, default_value = "rt/lidar")]
    lidar_topic: String,
}

#[async_std::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    env_logger::init();

    let mut zenoh_config = zenoh::config::Config::default();
    zenoh_config
        .set_mode(Some(zenoh::config::WhatAmI::Client))
        .unwrap();
    let endpoint = zenoh::config::Locator::from_str("tcp/127.0.0.1:7447").unwrap();
    zenoh_config.connect.endpoints = vec![endpoint.into()];
    let _ = zenoh_config.scouting.multicast.set_enabled(Some(false));
    let session = zenoh::open(zenoh_config)
        .res_async()
        .await
        .unwrap()
        .into_arc();
    debug!("opened zenoh session");

    spawn_tf_static(session.clone(), &args).await.unwrap();

    let local = {
        let stream = TcpStream::connect(format!("{}:80", &args.target))?;
        stream.local_addr()?.ip()
    };

    let api = format!("http://{}//api/v1/sensor", &args.target);

    let config = Config {
        udp_dest: local.to_string(),
        lidar_mode: args.mode.clone(),
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
    let mut frame_reader = FrameReader::new(params)?;
    let mut points = Points::new(frame_reader.rows * frame_reader.cols);
    let mut depth = Array2::<u16>::zeros((frame_reader.rows, frame_reader.cols));
    let mut reflect = Array2::<u8>::zeros((frame_reader.rows, frame_reader.cols));

    // On Linux [::] will bind to IPv4 and IPv6 but not on Windows so we bind
    // according to the local address IP version.
    let bind_addr = match local.is_ipv4() {
        true => format!("0.0.0.0:{}", config.udp_port_lidar),
        false => format!("[::]:{}", config.udp_port_lidar),
    };

    common::set_process_priority();
    let sock = UdpSocket::bind(bind_addr).await?;
    let sock = common::set_socket_bufsize(sock, 16 * 1024 * 1024);

    let mut buf = [0u8; 16 * 1024];

    let points_publisher = match session
        .clone()
        .declare_publisher(format!("{}/points", args.lidar_topic))
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Block)
        .res_async()
        .await
    {
        Ok(v) => v,
        Err(e) => {
            error!(
                "Failed to create publisher {}/points: {:?}",
                args.lidar_topic, e
            );
            return Err(e);
        }
    };

    let publish_depth = match session
        .clone()
        .declare_publisher(format!("{}/depth", args.lidar_topic))
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Block)
        .res_async()
        .await
    {
        Ok(v) => v,
        Err(e) => {
            error!(
                "Failed to create publisher {}/depth: {:?}",
                args.lidar_topic, e
            );
            return Err(e);
        }
    };

    let publish_reflect = match session
        .clone()
        .declare_publisher(format!("{}/reflect", args.lidar_topic))
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Block)
        .res_async()
        .await
    {
        Ok(v) => v,
        Err(e) => {
            error!(
                "Failed to create publisher {}/reflect: {:?}",
                args.lidar_topic, e
            );
            return Err(e);
        }
    };

    loop {
        let len = match sock.recv_from(&mut buf).await {
            Ok((len, _)) => len,
            Err(e) => {
                error!("udp_read recv error: {:?}", e);
                continue;
            }
        };

        if let Some(frame) =
            frame_reader.update(&mut points, &mut depth, &mut reflect, &buf[..len])?
        {
            let points = format_points(&points, frame.n_points, &args.frame_id)?;
            let points = points_publisher.put(points).res_async();
            let depth = format_depth(&depth, &frame.crop, &args.frame_id)?;
            let depth = publish_depth.put(depth).res_async();
            let reflect = format_reflect(&reflect, &frame.crop, &args.frame_id)?;
            let reflect = publish_reflect.put(reflect).res_async();

            match futures::try_join!(points, depth, reflect) {
                Ok(_) => trace!("{} message sent", args.lidar_topic),
                Err(e) => error!("{} message error: {:?}", args.lidar_topic, e),
            }
        }
    }
}

fn format_points(points: &Points, n_points: usize, frame_id: &str) -> Result<Value, cdr::Error> {
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
            stamp: timestamp()?,
            frame_id: frame_id.to_string(),
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

    let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;
    let encoded = Value::from(encoded).encoding(Encoding::WithSuffix(
        KnownEncoding::AppOctetStream,
        "sensor_msgs/msg/PointCloud2".into(),
    ));

    Ok(encoded)
}

fn format_depth(
    depth: &Array2<u16>,
    crop: &(usize, usize),
    frame_id: &str,
) -> Result<Value, cdr::Error> {
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
            stamp: timestamp()?,
            frame_id: frame_id.to_string(),
        },
        height: height as u32,
        width: width as u32,
        encoding: "mono16".to_owned(),
        is_bigendian: 0,
        step: width as u32 * 2,
        data,
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;
    let encoded = Value::from(encoded).encoding(Encoding::WithSuffix(
        KnownEncoding::AppOctetStream,
        "sensor_msgs/msg/Image".into(),
    ));

    Ok(encoded)
}

fn format_reflect(
    reflect: &Array2<u8>,
    crop: &(usize, usize),
    frame_id: &str,
) -> Result<Value, cdr::Error> {
    let reflect = reflect
        .slice(ndarray::s![.., crop.0..crop.1])
        .as_standard_layout()
        .to_owned();
    let (height, width) = reflect.dim();

    let msg = Image {
        header: Header {
            stamp: timestamp()?,
            frame_id: frame_id.to_string(),
        },
        height: height as u32,
        width: width as u32,
        encoding: "mono8".to_owned(),
        is_bigendian: 0,
        step: width as u32,
        data: reflect.as_slice().unwrap().to_vec(),
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;
    let encoded = Value::from(encoded).encoding(Encoding::WithSuffix(
        KnownEncoding::AppOctetStream,
        "sensor_msgs/msg/Image".into(),
    ));

    Ok(encoded)
}

async fn spawn_tf_static(
    session: Arc<zenoh::Session>,
    args: &Args,
) -> Result<(), Box<dyn std::error::Error>> {
    let publisher = match session
        .declare_publisher("rt/tf_static".to_string())
        .priority(Priority::Background)
        .congestion_control(CongestionControl::Drop)
        .res_async()
        .await
    {
        Ok(v) => v,
        Err(e) => {
            error!("Failed to create publisher rt/tf_static: {:?}", e);
            return Err(e);
        }
    };

    let msg = TransformStamped {
        header: Header {
            frame_id: args.base_frame_id.clone(),
            stamp: timestamp().unwrap_or(Time { sec: 0, nanosec: 0 }),
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

    let msg =
        Value::from(cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?).encoding(Encoding::WithSuffix(
            KnownEncoding::AppOctetStream,
            "geometry_msgs/msg/TransformStamped".into(),
        ));

    thread::Builder::new()
        .name("tf_static".to_string())
        .spawn(move || {
            let interval = Duration::from_secs(1);
            let mut target_time = Instant::now() + interval;

            loop {
                publisher.put(msg.clone()).res_sync().unwrap();
                trace!("lidarpub publishing rt/tf_static");
                sleep(target_time.duration_since(Instant::now()));
                target_time += interval
            }
        })?;

    Ok(())
}

#[cfg(target_os = "linux")]
fn timestamp() -> Result<builtin_interfaces::Time, std::io::Error> {
    let mut tp = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let err = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC_RAW, &mut tp) };
    if err != 0 {
        return Err(std::io::Error::last_os_error());
    }

    Ok(builtin_interfaces::Time {
        sec: tp.tv_sec as i32,
        nanosec: tp.tv_nsec as u32,
    })
}

#[cfg(not(target_os = "linux"))]
fn timestamp() -> Result<builtin_interfaces::Time, std::io::Error> {
    Err(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        "timestamp not implemented",
    ))
}
