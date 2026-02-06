// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! EdgeFirst LiDAR Publisher Library
//!
//! This library provides drivers and utilities for working with LiDAR sensors.
//!
//! # Architecture
//!
//! The library uses a **client-owned frame** pattern for zero-allocation
//! operation:
//!
//! ```text
//! ┌─────────────────┐     ┌───────────────┐     ┌─────────────────┐
//! │  PacketSource   │ ──► │  LidarDriver  │ ──► │  LidarFrame     │
//! │  (UDP/pcap/test)│     │  (Ouster/RS)  │     │  (client-owned) │
//! └─────────────────┘     └───────────────┘     └─────────────────┘
//!                                                       │
//!                                                       ▼
//!                               ┌─────────────────────────────────────┐
//!                               │  formats::format_points_*           │
//!                               │  (SIMD: NEON/portable_simd/scalar)  │
//!                               └─────────────────────────────────────┘
//! ```
//!
//! The client owns all frame objects and provides mutable references to the
//! driver:
//! 1. Client creates frames: `OusterLidarFrame::with_capacity(N)`
//! 2. Client provides mutable reference: `driver.process(&mut frame, data)`
//! 3. Driver writes points and metadata into the frame
//! 4. When complete (returns `Ok(true)`), client owns the filled frame
//! 5. Client can swap to another frame for the next fill cycle
//!
//! # Modules
//!
//! - [`formats`]: SIMD-optimized point cloud formatting
//! - [`lidar`]: Common types, traits, and error handling
//! - [`ouster`]: Ouster OS0/OS1/OS2/OSDome driver
//! - [`robosense`]: Robosense E1R driver
//! - [`packet_source`]: Packet source abstraction for testing
//! - [`common`]: Shared utilities (priority, socket config)
//!
//! # Example
//!
//! ```ignore
//! use edgefirst_lidarpub::{
//!     lidar::{LidarDriver, LidarFrame},
//!     robosense::{RobosenseDriver, RobosenseLidarFrame},
//! };
//!
//! let mut driver = RobosenseDriver::new();
//! let mut frame = RobosenseLidarFrame::new();
//!
//! // Process packets
//! loop {
//!     let len = socket.recv(&mut buf)?;
//!     if driver.process(&mut frame, &buf[..len])? {
//!         // Frame complete - access point data directly
//!         let x = frame.x();
//!         let y = frame.y();
//!         let z = frame.z();
//!         let intensity = frame.intensity();
//!         let range = frame.range();  // Pre-computed, no sqrt needed!
//!     }
//! }
//! ```

#![cfg_attr(feature = "portable_simd", feature(portable_simd))]

pub mod common;
pub mod formats;
pub mod lidar;
pub mod ouster;
pub mod packet_source;
#[cfg(feature = "pcap")]
pub mod pcap_source;
pub mod robosense;

// Re-exports for convenience
pub use formats::PointFieldType;
pub use lidar::{Error, LidarDriver, LidarFrame, LidarFrameWriter, Points, SensorType};
pub use ouster::OusterLidarFrame;
pub use packet_source::PacketSource;
#[cfg(feature = "pcap")]
pub use pcap_source::PcapSource;
pub use robosense::RobosenseLidarFrame;
