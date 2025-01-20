use cdr::{CdrLe, Infinite};
use clap::{builder::PossibleValuesParser, Parser};
use edgefirst_schemas::{
    builtin_interfaces::{self, Time},
    geometry_msgs::{Quaternion, Transform, TransformStamped, Vector3},
    sensor_msgs::{PointCloud2, PointField},
    std_msgs::Header,
};
use itertools::izip;
use lidarpub::ouster::{
    BeamIntrinsics, Config, FrameReader, LidarDataFormat, Parameters, SensorInfo,
};
use log::{debug, error, info, trace};
use std::{
    io::IsTerminal as _,
    io::Write as _,
    net::TcpStream,
    str::FromStr as _,
    sync::Arc,
    thread::{self, sleep},
    time::{Duration, Instant},
};
use zenoh::prelude::{r#async::*, sync::SyncResolve};

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
    #[arg(long, num_args = 2, value_names = ["START", "STOP"], value_delimiter=' ', default_value = "0 360")]
    azimuth: Vec<u32>,

    /// LiDAR columns per frame.
    #[arg(long, default_value = "1024x10", 
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

    /// lidar point cloud topic
    #[arg(long, default_value = "rt/lidar/points")]
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
        .into_json::<Config>()?;
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
                .into_json::<SensorInfo>()?;
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
        .into_json::<LidarDataFormat>()?;
    let beam_intrinsics = ureq::get(&format!("{}/metadata/beam_intrinsics", api))
        .call()?
        .into_json::<BeamIntrinsics>()?;

    let params = Parameters {
        sensor_info,
        lidar_data_format,
        beam_intrinsics,
    };

    debug!("{:?}", params);

    // On Linux [::] will bind to IPv4 and IPv6 but not on Windows so we bind
    // according to the local address IP version.
    let bind_addr = match local.is_ipv4() {
        true => format!("0.0.0.0:{}", config.udp_port_lidar),
        false => format!("[::]:{}", config.udp_port_lidar),
    };
    let socket = std::net::UdpSocket::bind(bind_addr)?;
    let mut buf = [0u8; 16 * 1024];
    let mut frame_reader = FrameReader::new(params)?;

    let publisher = match session
        .declare_publisher(args.lidar_topic.clone())
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .res_async()
        .await
    {
        Ok(v) => v,
        Err(e) => {
            error!("Failed to create publisher {}: {:?}", args.lidar_topic, e);
            return Err(e);
        }
    };

    loop {
        let (len, _src) = socket.recv_from(&mut buf)?;
        if let Some(frame) = frame_reader.update(&buf[..len])? {
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
                PointField {
                    name: String::from("nir"),
                    offset: 13,
                    datatype: PointFieldType::UINT8 as u8,
                    count: 1,
                },
            ];

            let n_points = frame.points.len();
            let data: Vec<_> = frame
                .points
                .iter()
                .flat_map(|pt| {
                    let x = pt.x.to_ne_bytes();
                    let y = pt.y.to_ne_bytes();
                    let z = pt.z.to_ne_bytes();

                    [
                        x[0], x[1], x[2], x[3], y[0], y[1], y[2], y[3], z[0], z[1], z[2], z[3],
                        pt.reflect, pt.nir,
                    ]
                })
                .collect();

            let msg = PointCloud2 {
                header: Header {
                    stamp: timestamp()?,
                    frame_id: args.frame_id.clone(),
                },
                height: 1,
                width: n_points as u32,
                fields,
                is_bigendian: false,
                point_step: 14,
                row_step: 14 * n_points as u32,
                data,
                is_dense: true,
            };

            let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;
            let encoded = Value::from(encoded).encoding(Encoding::WithSuffix(
                KnownEncoding::AppOctetStream,
                "sensor_msgs/msg/PointCloud2".into(),
            ));

            match publisher.put(encoded).res_async().await {
                Ok(_) => trace!("{} message sent", args.lidar_topic),
                Err(e) => error!("{} message error: {:?}", args.lidar_topic, e),
            }
        }
    }
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
