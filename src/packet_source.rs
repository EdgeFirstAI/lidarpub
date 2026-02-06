// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Packet source abstraction for LiDAR drivers.
//!
//! This module provides a [`PacketSource`] trait that abstracts the source of
//! UDP packets, enabling:
//!
//! - **Live operation**: Reading from UDP sockets
//! - **Testing**: Replaying pre-recorded packets
//! - **Pcap replay**: Reading from pcap files (with feature flag)
//!
//! # Example
//!
//! ```ignore
//! use edgefirst_lidarpub::packet_source::{PacketSource, TestSource};
//!
//! // Create a test source with pre-defined packets
//! let packets = vec![
//!     vec![0x55, 0xaa, 0x5a, 0xa5, /* ... */],
//!     vec![0x55, 0xaa, 0x5a, 0xa5, /* ... */],
//! ];
//! let mut source = TestSource::new(packets);
//!
//! // Process packets
//! let mut buf = [0u8; 2048];
//! while source.has_more() {
//!     let len = source.recv(&mut buf).await?;
//!     // Process buf[..len]
//! }
//! ```

use crate::lidar::Error;
use std::{future::Future, pin::Pin};

/// Trait for packet sources.
///
/// Implementations provide packets from various sources (UDP, pcap, test data).
pub trait PacketSource: Send {
    /// Receive the next packet into the provided buffer.
    ///
    /// # Returns
    /// - `Ok(len)` - Number of bytes received
    /// - `Err` - I/O or source error
    fn recv<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> Pin<Box<dyn Future<Output = Result<usize, Error>> + Send + 'a>>;

    /// Check if more packets are available.
    ///
    /// For infinite sources (like UDP), always returns `true`.
    /// For finite sources (test, pcap), returns `false` when exhausted.
    fn has_more(&self) -> bool;
}

/// UDP socket packet source for live sensor operation.
pub struct UdpSource {
    socket: tokio::net::UdpSocket,
}

impl UdpSource {
    /// Create a new UDP source from an existing socket.
    pub fn new(socket: tokio::net::UdpSocket) -> Self {
        Self { socket }
    }

    /// Bind to an address and create a UDP source.
    pub async fn bind(addr: &str) -> Result<Self, Error> {
        let socket = tokio::net::UdpSocket::bind(addr).await?;
        Ok(Self { socket })
    }
}

impl PacketSource for UdpSource {
    fn recv<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> Pin<Box<dyn Future<Output = Result<usize, Error>> + Send + 'a>> {
        Box::pin(async move {
            let len = self.socket.recv(buf).await?;
            Ok(len)
        })
    }

    fn has_more(&self) -> bool {
        true // UDP sources are infinite
    }
}

/// Test packet source for unit testing.
///
/// Provides a sequence of pre-defined packets for testing driver logic
/// without hardware.
pub struct TestSource {
    packets: Vec<Vec<u8>>,
    index: usize,
}

impl TestSource {
    /// Create a new test source with the given packets.
    pub fn new(packets: Vec<Vec<u8>>) -> Self {
        Self { packets, index: 0 }
    }

    /// Create an empty test source.
    pub fn empty() -> Self {
        Self::new(Vec::new())
    }

    /// Reset the source to the beginning.
    pub fn reset(&mut self) {
        self.index = 0;
    }

    /// Get the number of packets.
    pub fn len(&self) -> usize {
        self.packets.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.packets.is_empty()
    }

    /// Get the current index.
    pub fn current_index(&self) -> usize {
        self.index
    }
}

impl PacketSource for TestSource {
    fn recv<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> Pin<Box<dyn Future<Output = Result<usize, Error>> + Send + 'a>> {
        Box::pin(async move {
            if self.index >= self.packets.len() {
                return Err(Error::Io(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "no more packets",
                )));
            }

            let packet = &self.packets[self.index];
            let len = packet.len().min(buf.len());
            buf[..len].copy_from_slice(&packet[..len]);
            self.index += 1;
            Ok(len)
        })
    }

    fn has_more(&self) -> bool {
        self.index < self.packets.len()
    }
}

/// Looping test source that repeats packets indefinitely.
///
/// Useful for performance testing or simulating continuous sensor data.
pub struct LoopingTestSource {
    packets: Vec<Vec<u8>>,
    index: usize,
}

impl LoopingTestSource {
    /// Create a new looping test source.
    pub fn new(packets: Vec<Vec<u8>>) -> Self {
        Self { packets, index: 0 }
    }

    /// Get the current loop index (wraps around).
    pub fn current_index(&self) -> usize {
        self.index
    }
}

impl PacketSource for LoopingTestSource {
    fn recv<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> Pin<Box<dyn Future<Output = Result<usize, Error>> + Send + 'a>> {
        Box::pin(async move {
            if self.packets.is_empty() {
                return Err(Error::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "no packets to loop",
                )));
            }

            let packet = &self.packets[self.index % self.packets.len()];
            let len = packet.len().min(buf.len());
            buf[..len].copy_from_slice(&packet[..len]);
            self.index += 1;
            Ok(len)
        })
    }

    fn has_more(&self) -> bool {
        !self.packets.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_test_source() {
        let packets = vec![vec![1, 2, 3, 4], vec![5, 6, 7, 8, 9, 10], vec![11, 12]];
        let mut source = TestSource::new(packets);

        assert!(source.has_more());
        assert_eq!(source.len(), 3);

        let mut buf = [0u8; 100];

        // First packet
        let len = source.recv(&mut buf).await.unwrap();
        assert_eq!(len, 4);
        assert_eq!(&buf[..len], &[1, 2, 3, 4]);

        // Second packet
        let len = source.recv(&mut buf).await.unwrap();
        assert_eq!(len, 6);
        assert_eq!(&buf[..len], &[5, 6, 7, 8, 9, 10]);

        // Third packet
        assert!(source.has_more());
        let len = source.recv(&mut buf).await.unwrap();
        assert_eq!(len, 2);
        assert_eq!(&buf[..len], &[11, 12]);

        // No more
        assert!(!source.has_more());
        assert!(source.recv(&mut buf).await.is_err());
    }

    #[tokio::test]
    async fn test_test_source_reset() {
        let packets = vec![vec![1, 2], vec![3, 4]];
        let mut source = TestSource::new(packets);
        let mut buf = [0u8; 100];

        source.recv(&mut buf).await.unwrap();
        source.recv(&mut buf).await.unwrap();
        assert!(!source.has_more());

        source.reset();
        assert!(source.has_more());
        assert_eq!(source.current_index(), 0);

        let len = source.recv(&mut buf).await.unwrap();
        assert_eq!(&buf[..len], &[1, 2]);
    }

    #[tokio::test]
    async fn test_looping_source() {
        let packets = vec![vec![1, 2], vec![3, 4]];
        let mut source = LoopingTestSource::new(packets);
        let mut buf = [0u8; 100];

        // Loop several times
        for i in 0..5 {
            assert!(source.has_more());
            let len = source.recv(&mut buf).await.unwrap();
            let expected: &[u8] = if i % 2 == 0 { &[1, 2] } else { &[3, 4] };
            assert_eq!(&buf[..len], expected);
        }

        // Still has more (infinite)
        assert!(source.has_more());
    }

    #[tokio::test]
    async fn test_empty_test_source() {
        let mut source = TestSource::empty();
        assert!(!source.has_more());
        assert!(source.is_empty());

        let mut buf = [0u8; 100];
        assert!(source.recv(&mut buf).await.is_err());
    }

    #[tokio::test]
    async fn test_buffer_truncation() {
        let packets = vec![vec![1, 2, 3, 4, 5, 6, 7, 8]];
        let mut source = TestSource::new(packets);

        // Small buffer
        let mut buf = [0u8; 4];
        let len = source.recv(&mut buf).await.unwrap();
        assert_eq!(len, 4);
        assert_eq!(&buf[..len], &[1, 2, 3, 4]);
    }
}
