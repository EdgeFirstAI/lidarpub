mod common;

use async_std::{net::UdpSocket, task::block_on};
use clap::{builder::PossibleValuesParser, Parser};
use common::TimestampMode;
use lidarpub::ouster::{
    BeamIntrinsics, Config, Frame, FrameReader, LidarDataFormat, Parameters, Points, SensorInfo,
};
use log::error;
use ndarray::Array2;
use pcap_parser::{traits::PcapReaderIterator, *};
use rerun::{external::re_sdk_comms::DEFAULT_SERVER_PORT, RecordingStream};
use std::{
    error::Error,
    fs::File,
    io::{BufReader, IsTerminal as _, Write as _},
    net::{Ipv4Addr, SocketAddr, TcpStream},
    path::Path,
    thread::sleep,
    time::Duration,
};

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// connect to remote rerun viewer at this address
    #[arg(short, long)]
    connect: Option<Ipv4Addr>,

    /// record rerun data to file instead of live viewer
    #[arg(short, long)]
    record: Option<String>,

    /// launch local rerun viewer
    #[arg(short, long)]
    viewer: bool,

    /// use this port for the rerun viewer (remote or web server)
    #[arg(short, long)]
    port: Option<u16>,

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
    mode: String,

    /// LiDAR timestamp mode.  If using the PTP1588 timestamp mode the LiDAR
    /// must be connected to a PTP1588 enabled network, the Maivin can provide
    /// this time through the ptp4l service.
    #[arg(long, env, value_enum, default_value = "internal")]
    timestamp_mode: TimestampMode,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    let rr = if let Some(addr) = args.connect {
        let port = args.port.unwrap_or(DEFAULT_SERVER_PORT);
        let remote = SocketAddr::new(addr.into(), port);
        Some(
            rerun::RecordingStreamBuilder::new("radarview")
                .connect_tcp_opts(remote, rerun::default_flush_timeout())?,
        )
    } else if let Some(record) = &args.record {
        Some(rerun::RecordingStreamBuilder::new("radarview").save(record)?)
    } else if args.viewer {
        Some(rerun::RecordingStreamBuilder::new("radarview").spawn()?)
    } else {
        None
    };

    if Path::new(&args.target).exists() {
        pcap_loop(&rr, &args.target)?;
    } else {
        block_on(live_loop(&rr, &args))?;
    }

    Ok(())
}

fn pcap_loop(
    rr: &Option<RecordingStream>,
    path: &String,
) -> Result<(), Box<dyn std::error::Error>> {
    let json_path = Path::new(path).with_extension("json");
    let json = File::open(json_path)?;
    let params: Parameters = serde_json::from_reader(json)?;
    let mut frame_reader = FrameReader::new(params)?;
    let mut points = Points::new(frame_reader.rows * frame_reader.cols);
    let mut depth = Array2::<u16>::zeros((frame_reader.rows, frame_reader.cols));
    let mut reflect = Array2::<u8>::zeros((frame_reader.rows, frame_reader.cols));

    let pcap_path = Path::new(path).with_extension("pcap");
    let pcap = File::open(pcap_path)?;
    let buffered = BufReader::new(pcap);
    let mut pcap_reader = LegacyPcapReader::new(65536, buffered)?;

    loop {
        match pcap_reader.next() {
            Ok((offset, block)) => {
                match block {
                    PcapBlockOwned::LegacyHeader(_) => (),
                    PcapBlockOwned::Legacy(block) => {
                        match etherparse::SlicedPacket::from_ethernet(block.data) {
                            Err(err) => error!("Err {:?}", err),
                            Ok(pkt) => {
                                if let Some(etherparse::TransportSlice::Udp(udp)) = pkt.transport {
                                    if udp.destination_port() != 7502 {
                                        pcap_reader.consume(offset);
                                        continue;
                                    }

                                    if let Some(frame) = frame_reader.update(
                                        &mut points,
                                        &mut depth,
                                        &mut reflect,
                                        udp.payload(),
                                    )? {
                                        if let Some(rr) = rr {
                                            rr.set_time_seconds(
                                                "stable_time",
                                                frame.timestamp as f64 / 1e9,
                                            );
                                        }

                                        frame_handler(rr, frame, &points, &depth, &reflect)?;
                                    }
                                }
                            }
                        }
                    }
                    PcapBlockOwned::NG(_) => unreachable!(),
                }
                pcap_reader.consume(offset);
            }
            Err(PcapError::Eof) => break,
            Err(PcapError::Incomplete(_)) => {
                pcap_reader.refill().unwrap();
            }
            Err(e) => panic!("error while reading: {:?}", e),
        }
    }

    Ok(())
}

async fn live_loop(
    rr: &Option<RecordingStream>,
    args: &Args,
) -> Result<(), Box<dyn std::error::Error>> {
    let local = {
        let stream = TcpStream::connect(format!("{}:80", &args.target))?;
        stream.local_addr()?.ip()
    };

    let api = format!("http://{}//api/v1/sensor", &args.target);

    let config = Config {
        udp_dest: local.to_string(),
        lidar_mode: args.mode.clone(),
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
            if let Some(rr) = rr {
                rr.set_time_seconds("stable_time", frame.timestamp as f64 / 1e9);
            }

            frame_handler(rr, frame, &points, &depth, &reflect)?;
        }
    }
}

fn frame_handler(
    rr: &Option<RecordingStream>,
    frame: Frame,
    points: &Points,
    depth: &Array2<u16>,
    reflect: &Array2<u8>,
) -> Result<(), Box<dyn Error>> {
    if let Some(rr) = rr {
        let x = &points.x[..frame.n_points];
        let y = &points.y[..frame.n_points];
        let z = &points.z[..frame.n_points];
        let l = &points.l[..frame.n_points];

        rr.log("n_points", &rerun::Scalar::new(frame.n_points as f64))?;
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
        rr.log("points", &points)?;

        rr.log(
            "reflect",
            &rerun::Image::from_color_model_and_tensor(
                rerun::ColorModel::L,
                reflect
                    .slice(ndarray::s![.., frame.crop.0..frame.crop.1])
                    .as_standard_layout()
                    .to_owned(),
            )?,
        )?;

        rr.log(
            "depth",
            &rerun::Image::from_color_model_and_tensor(
                rerun::ColorModel::L,
                depth
                    .slice(ndarray::s![.., frame.crop.0..frame.crop.1])
                    .as_standard_layout()
                    .to_owned(),
            )?,
        )?;
    }

    Ok(())
}
