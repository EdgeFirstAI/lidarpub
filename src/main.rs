use clap::Parser;
use log::error;
use lidarpub::ouster::HeaderSlice;
use pcap_parser::{traits::PcapReaderIterator, *};
use rerun::{
    external::re_sdk_comms::DEFAULT_SERVER_PORT,
    RecordingStream,
};
use std::{
    fs::File,
    io::BufReader,
    net::{Ipv4Addr, SocketAddr},
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

    /// Read from a pcap file instead of a live interface.
    #[arg()]
    pcap: Option<String>,
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

    if let Some(pcap) = args.pcap {
        pcap_loop(&rr, &pcap)?;
    }

    Ok(())
}

fn pcap_loop(
    rr: &Option<RecordingStream>,
    path: &String,
) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(rr) = rr { rr.set_time_seconds("stable_time", 0f64) }

    let file = File::open(path)?;
    let buffered = BufReader::new(file);
    let mut reader = LegacyPcapReader::new(65536, buffered)?;
    let mut num_blocks = 0;

    loop {
        match reader.next() {
            Ok((offset, block)) => {
                num_blocks += 1;
                match block {
                    PcapBlockOwned::LegacyHeader(hdr) => println!("header: {:?}", hdr),
                    PcapBlockOwned::Legacy(block) => {
                        match etherparse::SlicedPacket::from_ethernet(block.data) {
                            Err(err) => error!("Err {:?}", err),
                            Ok(pkt) => {
                                if let Some(etherparse::TransportSlice::Udp(udp)) = pkt.transport {
                                    if udp.destination_port() != 7502 {
                                        reader.consume(offset);
                                        continue;
                                    }

                                    let header = HeaderSlice::from_slice(udp.payload())?;
                                    let header = header.to_header();
                                    println!("header: {:?}", header);
                                }
                            }
                        }
                    }
                    PcapBlockOwned::NG(_) => unreachable!(),
                }
                reader.consume(offset);
            }
            Err(PcapError::Eof) => break,
            Err(PcapError::Incomplete(_)) => {
                reader.refill().unwrap();
            }
            Err(e) => panic!("error while reading: {:?}", e),
        }
    }

    println!("\tnum_blocks: {}", num_blocks);

    Ok(())
}
