// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

mod common;

use clap::{Parser, builder::PossibleValuesParser};
use common::TimestampMode;
use edgefirst_lidarpub::ouster::{
    BeamIntrinsics, Config, FrameBuilder, FrameReader, LidarDataFormat, Parameters, Points,
    SensorInfo,
};
use kanal::{Receiver, Sender};
use log::error;
use ndarray::Array2;
use pcap_parser::{traits::PcapReaderIterator, *};
use rerun::RecordingStream;
use std::{
    fs::File,
    io::{BufReader, IsTerminal as _, Write as _},
    net::{TcpStream, UdpSocket},
    path::Path,
    thread::sleep,
    time::Duration,
};

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Rerun parameters
    #[command(flatten)]
    pub rerun: rerun::clap::RerunArgs,

    /// Connect to target device or pcap file.  If target is a valid pcap file,
    /// it will be used otherwise it will be tried as a hostname or IP address.
    #[arg()]
    target: String,

    /// Azimuth field of view start and stop angles in degrees.  
    /// The 0 degree point is the rear connector of the LiDAR.
    #[arg(long, env, num_args = 2, value_names = ["START", "STOP"], value_delimiter=' ', default_value = "0 360")]
    azimuth: Vec<u32>,

    /// LiDAR column and refresh rate mode.  The format is "COLxHZ".
    #[arg(long, env, default_value = "1024x10", 
          value_parser = PossibleValuesParser::new(["512x10", "1024x10", "2048x10", "512x20", "1024x20",]))]
    lidar_mode: String,

    /// LiDAR timestamp mode.  If using the PTP1588 timestamp mode the LiDAR
    /// must be connected to a PTP1588 enabled network, the Maivin can provide
    /// this time through the ptp4l service.
    #[arg(long, env, value_enum, default_value = "internal")]
    timestamp_mode: TimestampMode,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    // Create Rerun logger using the provided parameters
    let (rr, _serve_guard) = args.rerun.init("lidar")?;

    if Path::new(&args.target).exists() {
        pcap_loop(&Some(rr), &args.target)?;
    } else {
        live_loop(&Some(rr), &args)?;
    }

    Ok(())
}

fn frame_handler(
    rr: &Option<RecordingStream>,
    n_points: usize,
    points: &Points,
    depth: &Array2<u16>,
    reflect: &Array2<u8>,
    crop: (usize, usize),
) {
    if let Some(rr) = rr {
        let x = &points.x[..n_points];
        let y = &points.y[..n_points];
        let z = &points.z[..n_points];
        let l = &points.l[..n_points];

        rr.log("n_points", &rerun::Scalars::new([n_points as f64]))
            .unwrap();
        let points: Vec<_> = x
            .iter()
            .zip(y.iter())
            .zip(z.iter())
            .map(|((x, y), z)| (*x, *y, *z))
            .collect();
        let points = rerun::Points3D::new(points).with_colors(
            l.iter()
                .map(|l| rerun::Color::from_rgb(l << 1, l << 1, l << 1)),
        );
        rr.log("points", &points).unwrap();

        let reflect_slice = reflect.slice(ndarray::s![.., crop.0..crop.1]);
        let reflect_shape = reflect_slice.shape();
        let reflect_data: Vec<u8> = if reflect_slice.is_standard_layout() {
            reflect_slice.iter().copied().collect()
        } else {
            reflect_slice.as_standard_layout().iter().copied().collect()
        };
        rr.log(
            "reflect",
            &rerun::Image::from_elements(
                &reflect_data,
                [reflect_shape[1] as u32, reflect_shape[0] as u32],
                rerun::datatypes::ColorModel::L,
            ),
        )
        .unwrap();

        let depth_slice = depth.slice(ndarray::s![.., crop.0..crop.1]);
        let depth_shape = depth_slice.shape();
        let depth_data: Vec<u16> = if depth_slice.is_standard_layout() {
            depth_slice.iter().copied().collect()
        } else {
            depth_slice.as_standard_layout().iter().copied().collect()
        };
        rr.log(
            "depth",
            &rerun::Image::from_elements(
                &depth_data,
                [depth_shape[1] as u32, depth_shape[0] as u32],
                rerun::datatypes::ColorModel::L,
            ),
        )
        .unwrap();
    }
}

async fn frame_processor(
    rr: &Option<RecordingStream>,
    params: Parameters,
    rx: Receiver<(u64, u16, Array2<u16>, Array2<u8>)>,
) {
    let mut frame_builder = FrameBuilder::new(&params);

    while let Ok((timestamp, frame_id, depth, reflect)) = rx.recv() {
        frame_builder.update(&depth, &reflect);

        if let Some(rr) = &rr {
            rr.set_timestamp_secs_since_epoch("stable_time", timestamp as f64 / 1e9);
            rr.log(
                "frame",
                &rerun::TextLog::new(format!(
                    "timestamp: {} frame_id: {} n_points: {}",
                    timestamp, frame_id, frame_builder.n_points
                )),
            )
            .unwrap();
        }

        frame_handler(
            rr,
            frame_builder.n_points,
            &frame_builder.points,
            &frame_builder.depth,
            &frame_builder.reflect,
            frame_builder.crop,
        );
    }
}

fn pcap_loop(
    rr: &Option<RecordingStream>,
    path: &String,
) -> Result<(), Box<dyn std::error::Error>> {
    let json_path = Path::new(path).with_extension("json");
    let json = File::open(json_path)?;
    let params: Parameters = serde_json::from_reader(json)?;
    let mut frame_reader = FrameReader::new(&params.lidar_data_format)?;

    let pcap_path = Path::new(path).with_extension("pcap");
    let pcap = File::open(pcap_path)?;
    let buffered = BufReader::new(pcap);
    let mut pcap_reader = LegacyPcapReader::new(65536, buffered)?;

    let (tx, rx) = kanal::bounded(128);
    let rr = rr.clone();

    std::thread::Builder::new()
        .name("processor".to_string())
        .spawn(move || {
            tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(frame_processor(&rr, params, rx));
        })?;

    while let Ok((offset, block)) = pcap_reader.next() {
        let should_continue = process_block(block, &mut frame_reader, &tx)?;
        pcap_reader.consume(offset);
        if !should_continue {
            break;
        }
    }

    Ok(())
}

fn process_block(
    block: PcapBlockOwned,
    frame_reader: &mut FrameReader,
    tx: &Sender<(u64, u16, Array2<u16>, Array2<u8>)>,
) -> Result<bool, Box<dyn std::error::Error>> {
    match block {
        PcapBlockOwned::LegacyHeader(_) => Ok(false),
        PcapBlockOwned::NG(_) => Ok(false),
        PcapBlockOwned::Legacy(block) => {
            match etherparse::SlicedPacket::from_ethernet(block.data) {
                Err(err) => {
                    error!("Err {:?}", err);
                    Ok(false)
                }
                Ok(pkt) => {
                    if let Some(etherparse::TransportSlice::Udp(udp)) = pkt.transport {
                        if udp.destination_port() != 7502 {
                            return Ok(true); // Continue to next packet
                        }

                        frame_reader.update(
                            udp.payload(),
                            |timestamp, frame_id, depth, reflect| match tx
                                .send((timestamp, frame_id, depth, reflect))
                            {
                                Ok(_) => {}
                                Err(e) => error!("frame send error: {:?}", e),
                            },
                        )?;
                    }
                    Ok(false)
                }
            }
        }
    }
}

fn live_loop(rr: &Option<RecordingStream>, args: &Args) -> Result<(), Box<dyn std::error::Error>> {
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
    println!("{:?}", config);

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
                    println!("LiDAR initialization complete");
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

    println!("{:?}", params);
    let mut frame_reader = FrameReader::new(&params.lidar_data_format)?;

    // On Linux [::] will bind to IPv4 and IPv6 but not on Windows so we bind
    // according to the local address IP version.
    let bind_addr = match local.is_ipv4() {
        true => format!("0.0.0.0:{}", config.udp_port_lidar),
        false => format!("[::]:{}", config.udp_port_lidar),
    };

    common::set_process_priority();
    let sock = UdpSocket::bind(bind_addr)?;
    let sock = common::set_socket_bufsize(sock, 16 * 1024 * 1024);

    let mut buf = [0u8; 16 * 1024];
    let (tx, rx) = kanal::bounded(128);
    let rr = rr.clone();

    std::thread::Builder::new()
        .name("processor".to_string())
        .spawn(move || {
            tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(frame_processor(&rr, params, rx));
        })?;

    loop {
        let len = match sock.recv_from(&mut buf) {
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
