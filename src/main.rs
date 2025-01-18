use clap::Parser;
use lidarpub::ouster::{Config, Frame, FrameReader};
use log::error;
use pcap_parser::{traits::PcapReaderIterator, *};
use rerun::{external::re_sdk_comms::DEFAULT_SERVER_PORT, RecordingStream};
use std::{
    error::Error,
    fs::File,
    io::BufReader,
    net::{Ipv4Addr, SocketAddr},
    path::Path,
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
    } else if let Some(record) = args.record {
        Some(rerun::RecordingStreamBuilder::new("radarview").save(record)?)
    } else if args.viewer {
        Some(rerun::RecordingStreamBuilder::new("radarview").spawn()?)
    } else {
        None
    };

    if Path::new(&args.target).exists() {
        pcap_loop(&rr, &args.target)?;
    } else {
        live_loop(&rr, &args.target)?;
    }

    Ok(())
}

fn pcap_loop(
    rr: &Option<RecordingStream>,
    path: &String,
) -> Result<(), Box<dyn std::error::Error>> {
    let json_path = Path::new(path).with_extension("json");
    let json = File::open(json_path)?;
    let config: Config = serde_json::from_reader(json)?;
    let mut frame_reader = FrameReader::new(config)?;

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

                                    if let Some(frame) = frame_reader.update(udp.payload())? {
                                        if let Some(rr) = rr {
                                            rr.set_time_seconds(
                                                "stable_time",
                                                frame.frame_id as f64 / 10.0,
                                            );
                                        }

                                        frame_handler(rr, frame)?;
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

fn live_loop(
    rr: &Option<RecordingStream>,
    target: &String,
) -> Result<(), Box<dyn std::error::Error>> {
    // let socket = std::net::UdpSocket::bind("0.0.0.0:7502")?;
    // let mut buf = [0u8; 16 * 1024];
    // let mut frame = Frame::<64, 2048>::new();

    // loop {
    //     let (len, _src) = socket.recv_from(&mut buf)?;
    //     if let Some(frame) = frame.update(&buf[..len])? {
    //         frame_handler(rr, frame)?;
    //     }
    // }

    Ok(())
}

fn frame_handler(rr: &Option<RecordingStream>, frame: Frame) -> Result<(), Box<dyn Error>> {
    if let Some(rr) = rr {
        let points = rerun::Points3D::new(frame.points).with_colors(
            frame
                .reflect
                .iter()
                .map(|x| rerun::Color::from_rgb(x << 1, x << 1, x << 1)),
        );
        rr.log("points", &points)?;
    }

    Ok(())
}
