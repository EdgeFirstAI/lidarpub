// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! PCAP file packet source for testing and offline replay.
//!
//! This module provides [`PcapSource`], a [`PacketSource`] implementation that
//! reads UDP packets from PCAP/PCAPNG files. This enables integration testing
//! of LiDAR drivers without hardware.
//!
//! # Example
//!
//! ```ignore
//! use edgefirst_lidarpub::PcapSource;
//! use edgefirst_lidarpub::packet_source::PacketSource;
//!
//! // Load PCAP file, filtering by MSOP port
//! let mut source = PcapSource::from_file("sensor_data.pcap", Some(6699))?;
//!
//! let mut buf = [0u8; 2048];
//! while source.has_more() {
//!     let len = source.recv(&mut buf).await?;
//!     // Process buf[..len] with driver
//! }
//! ```

use crate::{lidar::Error, packet_source::PacketSource};
use pcap_parser::traits::PcapReaderIterator;
use std::{collections::HashMap, future::Future, path::Path, pin::Pin};

/// Extracted UDP packet with metadata.
#[derive(Clone)]
struct ExtractedPacket {
    /// UDP payload data
    payload: Vec<u8>,
}

/// PCAP file packet source for testing and offline replay.
///
/// Loads entire PCAP file into memory and provides packets via the
/// [`PacketSource`] trait. Supports both legacy PCAP and PCAPNG formats.
pub struct PcapSource {
    /// Pre-extracted UDP payloads
    packets: Vec<ExtractedPacket>,
    /// Current packet index
    index: usize,
}

impl PcapSource {
    /// Load PCAP file from disk, optionally filtering by port.
    ///
    /// # Arguments
    ///
    /// * `path` - Path to PCAP or PCAPNG file
    /// * `port` - Optional port filter (matches source OR destination)
    ///
    /// # Returns
    ///
    /// `PcapSource` with extracted UDP packets, or error on parse failure.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Load all UDP packets
    /// let source = PcapSource::from_file("capture.pcap", None)?;
    ///
    /// // Load only packets on port 6699 (Robosense MSOP)
    /// let source = PcapSource::from_file("capture.pcap", Some(6699))?;
    /// ```
    pub fn from_file<P: AsRef<Path>>(path: P, port: Option<u16>) -> Result<Self, Error> {
        let data = std::fs::read(path.as_ref()).map_err(Error::Io)?;
        Self::from_bytes(&data, port)
    }

    /// Load PCAP from bytes, optionally filtering by port.
    ///
    /// Useful for embedded test data or streaming scenarios.
    ///
    /// # Arguments
    ///
    /// * `data` - Raw PCAP/PCAPNG file contents
    /// * `port` - Optional port filter (matches source OR destination)
    pub fn from_bytes(data: &[u8], port: Option<u16>) -> Result<Self, Error> {
        let packets = Self::extract_packets(data, port)?;
        Ok(Self { packets, index: 0 })
    }

    /// Extract UDP packets from PCAP data with IP fragment reassembly.
    fn extract_packets(data: &[u8], port: Option<u16>) -> Result<Vec<ExtractedPacket>, Error> {
        // First, collect all raw Ethernet frames
        let mut raw_frames = Vec::new();

        // Try PCAPNG first, then legacy PCAP
        if data.len() >= 4 && data[0..4] == [0x0a, 0x0d, 0x0d, 0x0a] {
            Self::collect_raw_frames_pcapng(data, &mut raw_frames)?;
        } else {
            Self::collect_raw_frames_legacy(data, &mut raw_frames)?;
        }

        // Reassemble IP fragments and extract UDP payloads
        Self::reassemble_and_extract(&raw_frames, port)
    }

    /// Reassemble IP fragments and extract UDP payloads.
    fn reassemble_and_extract(
        frames: &[Vec<u8>],
        port: Option<u16>,
    ) -> Result<Vec<ExtractedPacket>, Error> {
        use etherparse::{NetSlice, SlicedPacket};

        // Key for fragment reassembly: (src_ip, dst_ip, protocol, identification)
        type FragKey = ([u8; 4], [u8; 4], u8, u16);

        // Fragment data: (offset, more_fragments, payload)
        struct Fragment {
            offset: u16,
            more_fragments: bool,
            data: Vec<u8>,
            /// Full Ethernet frame for first fragment (offset 0)
            first_frame: Option<Vec<u8>>,
        }

        let mut fragment_groups: HashMap<FragKey, Vec<Fragment>> = HashMap::new();
        let mut packets = Vec::new();

        for frame in frames {
            let Ok(parsed) = SlicedPacket::from_ethernet(frame) else {
                continue;
            };

            // Check if this is an IPv4 packet
            let Some(NetSlice::Ipv4(ipv4_slice)) = parsed.net else {
                continue;
            };

            let header = ipv4_slice.header();
            let identification = header.identification();
            let fragment_offset = header.fragments_offset().value();
            let more_fragments = header.more_fragments();

            // Non-fragmented packet (no MF flag and offset 0)
            if !more_fragments && fragment_offset == 0 {
                // Try to extract UDP directly
                if let Some(extracted) = Self::extract_udp_payload(frame, port) {
                    packets.push(extracted);
                }
                continue;
            }

            // This is a fragment - collect it
            let key: FragKey = (
                header.source(),
                header.destination(),
                header.protocol().0,
                identification,
            );

            // Get the IP payload (data after IP header)
            let ip_payload = ipv4_slice.payload().payload;

            let frag = Fragment {
                offset: fragment_offset,
                more_fragments,
                data: ip_payload.to_vec(),
                first_frame: if fragment_offset == 0 {
                    Some(frame.clone())
                } else {
                    None
                },
            };

            fragment_groups.entry(key).or_default().push(frag);
        }

        // Reassemble fragment groups
        for (_key, mut frags) in fragment_groups {
            // Sort by offset
            frags.sort_by_key(|f| f.offset);

            // Check if we have the first fragment
            let Some(first_frag) = frags.iter().find(|f| f.offset == 0) else {
                continue;
            };

            let Some(first_frame) = &first_frag.first_frame else {
                continue;
            };

            // Check if we have the last fragment (one without MF flag)
            if !frags.iter().any(|f| !f.more_fragments) {
                continue; // Incomplete - missing last fragment
            }

            // Reassemble the IP payload
            let mut reassembled_payload = Vec::new();
            let mut expected_offset = 0u16;

            for frag in &frags {
                if frag.offset != expected_offset {
                    // Gap in fragments - incomplete
                    break;
                }
                reassembled_payload.extend_from_slice(&frag.data);
                // Fragment offset is in 8-byte units
                expected_offset = frag.offset + (frag.data.len() as u16).div_ceil(8);
            }

            // Build reassembled packet: use first fragment's headers + reassembled payload
            // Parse the first frame to get header lengths
            let Ok(first_parsed) = SlicedPacket::from_ethernet(first_frame) else {
                continue;
            };

            let Some(NetSlice::Ipv4(ipv4_slice)) = first_parsed.net else {
                continue;
            };

            let eth_header_len = 14usize;
            let ip_header_len = (ipv4_slice.header().ihl() as usize) * 4;

            // Create reassembled frame: Ethernet + IP header + reassembled payload
            let mut reassembled_frame =
                Vec::with_capacity(eth_header_len + ip_header_len + reassembled_payload.len());

            // Copy Ethernet header
            reassembled_frame.extend_from_slice(&first_frame[..eth_header_len]);

            // Copy and modify IP header
            let ip_header_end = eth_header_len + ip_header_len;
            reassembled_frame.extend_from_slice(&first_frame[eth_header_len..ip_header_end]);

            // Update IP total length
            let new_total_len = (ip_header_len + reassembled_payload.len()) as u16;
            reassembled_frame[eth_header_len + 2] = (new_total_len >> 8) as u8;
            reassembled_frame[eth_header_len + 3] = (new_total_len & 0xff) as u8;

            // Clear fragment flags and offset
            reassembled_frame[eth_header_len + 6] = 0;
            reassembled_frame[eth_header_len + 7] = 0;

            // Append reassembled payload
            reassembled_frame.extend_from_slice(&reassembled_payload);

            // Now extract UDP from reassembled frame
            if let Some(extracted) = Self::extract_udp_payload(&reassembled_frame, port) {
                packets.push(extracted);
            }
        }

        Ok(packets)
    }

    /// Collect raw Ethernet frames from legacy PCAP.
    fn collect_raw_frames_legacy(data: &[u8], frames: &mut Vec<Vec<u8>>) -> Result<(), Error> {
        use pcap_parser::*;

        let mut reader = LegacyPcapReader::new(data.len(), data)
            .map_err(|e| Error::InvalidPacket(format!("Failed to create PCAP reader: {:?}", e)))?;

        loop {
            match reader.next() {
                Ok((offset, block)) => {
                    if let PcapBlockOwned::Legacy(packet) = block {
                        frames.push(packet.data.to_vec());
                    }
                    reader.consume(offset);
                }
                Err(PcapError::Eof) => break,
                Err(PcapError::Incomplete(_)) => break,
                Err(e) => {
                    return Err(Error::InvalidPacket(format!("PCAP parse error: {:?}", e)));
                }
            }
        }

        Ok(())
    }

    /// Collect raw Ethernet frames from PCAPNG.
    fn collect_raw_frames_pcapng(data: &[u8], frames: &mut Vec<Vec<u8>>) -> Result<(), Error> {
        use pcap_parser::*;

        let mut reader = PcapNGReader::new(data.len(), data).map_err(|e| {
            Error::InvalidPacket(format!("Failed to create PCAPNG reader: {:?}", e))
        })?;

        loop {
            match reader.next() {
                Ok((offset, block)) => {
                    match block {
                        PcapBlockOwned::NG(Block::EnhancedPacket(epb)) => {
                            frames.push(epb.data.to_vec());
                        }
                        PcapBlockOwned::NG(Block::SimplePacket(spb)) => {
                            frames.push(spb.data.to_vec());
                        }
                        _ => {}
                    }
                    reader.consume(offset);
                }
                Err(PcapError::Eof) => break,
                Err(PcapError::Incomplete(_)) => break,
                Err(e) => {
                    return Err(Error::InvalidPacket(format!("PCAPNG parse error: {:?}", e)));
                }
            }
        }

        Ok(())
    }

    /// Extract UDP payload from raw packet data.
    ///
    /// Uses etherparse to handle Ethernet/IP/UDP headers.
    fn extract_udp_payload(data: &[u8], port: Option<u16>) -> Option<ExtractedPacket> {
        use etherparse::SlicedPacket;

        let packet = SlicedPacket::from_ethernet(data).ok()?;

        // Check if this is a UDP packet and get the UDP slice
        let udp = match packet.transport {
            Some(etherparse::TransportSlice::Udp(udp)) => udp,
            _ => return None,
        };

        // Apply port filter if specified
        if let Some(filter_port) = port {
            let src_port = udp.source_port();
            let dst_port = udp.destination_port();
            if src_port != filter_port && dst_port != filter_port {
                return None;
            }
        }

        // Extract UDP payload from the UdpSlice
        let payload = udp.payload().to_vec();
        if payload.is_empty() {
            return None;
        }

        Some(ExtractedPacket { payload })
    }

    /// Reset source to beginning for replay.
    pub fn reset(&mut self) {
        self.index = 0;
    }

    /// Get the total number of packets.
    pub fn len(&self) -> usize {
        self.packets.len()
    }

    /// Check if the source contains no packets.
    pub fn is_empty(&self) -> bool {
        self.packets.is_empty()
    }

    /// Get the current packet index.
    pub fn current_index(&self) -> usize {
        self.index
    }

    /// Get remaining packet count.
    pub fn remaining(&self) -> usize {
        self.packets.len().saturating_sub(self.index)
    }
}

impl PacketSource for PcapSource {
    fn recv<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> Pin<Box<dyn Future<Output = Result<usize, Error>> + Send + 'a>> {
        Box::pin(async move {
            if self.index >= self.packets.len() {
                return Err(Error::Io(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "no more packets in PCAP",
                )));
            }

            let packet = &self.packets[self.index];
            let len = packet.payload.len().min(buf.len());
            buf[..len].copy_from_slice(&packet.payload[..len]);
            self.index += 1;
            Ok(len)
        })
    }

    fn has_more(&self) -> bool {
        self.index < self.packets.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Minimal valid legacy PCAP header (little-endian)
    const PCAP_HEADER: [u8; 24] = [
        0xd4, 0xc3, 0xb2, 0xa1, // Magic number (little-endian)
        0x02, 0x00, // Major version
        0x04, 0x00, // Minor version
        0x00, 0x00, 0x00, 0x00, // Timezone
        0x00, 0x00, 0x00, 0x00, // Timestamp accuracy
        0xff, 0xff, 0x00, 0x00, // Snap length
        0x01, 0x00, 0x00, 0x00, // Network type (Ethernet)
    ];

    // Create a minimal UDP packet with Ethernet + IP + UDP headers
    fn make_udp_packet(src_port: u16, dst_port: u16, payload: &[u8]) -> Vec<u8> {
        let udp_len = 8 + payload.len();
        let ip_len = 20 + udp_len;
        let total_len = 14 + ip_len; // Ethernet header is 14 bytes

        let mut packet = Vec::with_capacity(total_len);

        // Ethernet header (14 bytes)
        packet.extend_from_slice(&[0x00; 6]); // Dst MAC
        packet.extend_from_slice(&[0x00; 6]); // Src MAC
        packet.extend_from_slice(&[0x08, 0x00]); // EtherType: IPv4

        // IPv4 header (20 bytes, no options)
        packet.push(0x45); // Version + IHL
        packet.push(0x00); // DSCP + ECN
        packet.extend_from_slice(&(ip_len as u16).to_be_bytes()); // Total length
        packet.extend_from_slice(&[0x00, 0x00]); // Identification
        packet.extend_from_slice(&[0x00, 0x00]); // Flags + Fragment offset
        packet.push(0x40); // TTL
        packet.push(0x11); // Protocol: UDP
        packet.extend_from_slice(&[0x00, 0x00]); // Checksum (0 for test)
        packet.extend_from_slice(&[192, 168, 1, 1]); // Src IP
        packet.extend_from_slice(&[192, 168, 1, 2]); // Dst IP

        // UDP header (8 bytes)
        packet.extend_from_slice(&src_port.to_be_bytes());
        packet.extend_from_slice(&dst_port.to_be_bytes());
        packet.extend_from_slice(&(udp_len as u16).to_be_bytes());
        packet.extend_from_slice(&[0x00, 0x00]); // Checksum (0 for test)

        // Payload
        packet.extend_from_slice(payload);

        packet
    }

    // Create a PCAP packet record
    fn make_pcap_record(data: &[u8]) -> Vec<u8> {
        let len = data.len() as u32;
        let mut record = Vec::with_capacity(16 + data.len());

        // Packet record header (16 bytes)
        record.extend_from_slice(&[0x00; 4]); // Timestamp seconds
        record.extend_from_slice(&[0x00; 4]); // Timestamp microseconds
        record.extend_from_slice(&len.to_le_bytes()); // Captured length
        record.extend_from_slice(&len.to_le_bytes()); // Original length

        // Packet data
        record.extend_from_slice(data);

        record
    }

    #[test]
    fn test_extract_udp_payload() {
        let payload = b"test payload";
        let packet = make_udp_packet(6699, 12345, payload);

        let extracted = PcapSource::extract_udp_payload(&packet, None).unwrap();
        assert_eq!(extracted.payload, payload);
    }

    #[test]
    fn test_extract_udp_payload_port_filter() {
        let payload = b"test payload";
        let packet = make_udp_packet(6699, 12345, payload);

        // Match source port
        let extracted = PcapSource::extract_udp_payload(&packet, Some(6699)).unwrap();
        assert_eq!(extracted.payload, payload);

        // Match destination port
        let extracted = PcapSource::extract_udp_payload(&packet, Some(12345)).unwrap();
        assert_eq!(extracted.payload, payload);

        // No match
        let extracted = PcapSource::extract_udp_payload(&packet, Some(9999));
        assert!(extracted.is_none());
    }

    #[test]
    fn test_pcap_source_from_bytes() {
        // Create a minimal PCAP with one UDP packet
        let payload = b"hello world";
        let udp_packet = make_udp_packet(6699, 12345, payload);
        let record = make_pcap_record(&udp_packet);

        let mut pcap_data = Vec::new();
        pcap_data.extend_from_slice(&PCAP_HEADER);
        pcap_data.extend_from_slice(&record);

        let source = PcapSource::from_bytes(&pcap_data, None).unwrap();
        assert_eq!(source.len(), 1);
        assert!(!source.is_empty());
    }

    #[test]
    fn test_pcap_source_port_filter() {
        // Create PCAP with packets on different ports
        let payload1 = b"packet on 6699";
        let payload2 = b"packet on 7788";
        let packet1 = make_udp_packet(6699, 12345, payload1);
        let packet2 = make_udp_packet(7788, 12345, payload2);

        let mut pcap_data = Vec::new();
        pcap_data.extend_from_slice(&PCAP_HEADER);
        pcap_data.extend_from_slice(&make_pcap_record(&packet1));
        pcap_data.extend_from_slice(&make_pcap_record(&packet2));

        // Filter for port 6699
        let source = PcapSource::from_bytes(&pcap_data, Some(6699)).unwrap();
        assert_eq!(source.len(), 1);

        // No filter - both packets
        let source = PcapSource::from_bytes(&pcap_data, None).unwrap();
        assert_eq!(source.len(), 2);
    }

    #[tokio::test]
    async fn test_pcap_source_recv() {
        let payload = b"test data";
        let udp_packet = make_udp_packet(6699, 12345, payload);
        let record = make_pcap_record(&udp_packet);

        let mut pcap_data = Vec::new();
        pcap_data.extend_from_slice(&PCAP_HEADER);
        pcap_data.extend_from_slice(&record);

        let mut source = PcapSource::from_bytes(&pcap_data, None).unwrap();
        assert!(source.has_more());

        let mut buf = [0u8; 100];
        let len = source.recv(&mut buf).await.unwrap();
        assert_eq!(&buf[..len], payload);
        assert!(!source.has_more());
    }

    #[tokio::test]
    async fn test_pcap_source_reset() {
        let payload = b"test";
        let udp_packet = make_udp_packet(6699, 12345, payload);
        let record = make_pcap_record(&udp_packet);

        let mut pcap_data = Vec::new();
        pcap_data.extend_from_slice(&PCAP_HEADER);
        pcap_data.extend_from_slice(&record);

        let mut source = PcapSource::from_bytes(&pcap_data, None).unwrap();
        let mut buf = [0u8; 100];

        source.recv(&mut buf).await.unwrap();
        assert!(!source.has_more());

        source.reset();
        assert!(source.has_more());
        assert_eq!(source.current_index(), 0);
    }

    #[test]
    fn test_pcap_source_empty() {
        // PCAP with no packets
        let pcap_data = PCAP_HEADER.to_vec();
        let source = PcapSource::from_bytes(&pcap_data, None).unwrap();
        assert!(source.is_empty());
        assert_eq!(source.len(), 0);
        assert!(!source.has_more());
    }

    #[tokio::test]
    async fn test_pcap_source_exhausted() {
        let pcap_data = PCAP_HEADER.to_vec();
        let mut source = PcapSource::from_bytes(&pcap_data, None).unwrap();

        let mut buf = [0u8; 100];
        let result = source.recv(&mut buf).await;
        assert!(result.is_err());
    }

    #[test]
    fn test_remaining_packets() {
        let payload = b"test";
        let udp_packet = make_udp_packet(6699, 12345, payload);
        let record = make_pcap_record(&udp_packet);

        let mut pcap_data = Vec::new();
        pcap_data.extend_from_slice(&PCAP_HEADER);
        pcap_data.extend_from_slice(&record);
        pcap_data.extend_from_slice(&record);
        pcap_data.extend_from_slice(&record);

        let mut source = PcapSource::from_bytes(&pcap_data, None).unwrap();
        assert_eq!(source.remaining(), 3);

        source.index = 1;
        assert_eq!(source.remaining(), 2);

        source.index = 3;
        assert_eq!(source.remaining(), 0);
    }
}
