// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! EdgeFirst LiDAR Publisher Library
//!
//! This library provides drivers and utilities for working with LiDAR sensors.
//!
//! # Architecture
//!
//! The library uses a zero-allocation architecture for steady-state operation:
//!
//! ```text
//! ┌─────────────────┐     ┌───────────────┐     ┌─────────────────┐
//! │  PacketSource   │ ──► │  LidarDriver  │ ──► │  DoubleBuffer   │
//! │  (UDP/pcap/test)│     │  (Ouster/RS)  │     │  (zero-alloc)   │
//! └─────────────────┘     └───────────────┘     └─────────────────┘
//!                                                       │
//!                                                       ▼
//!                               ┌─────────────────────────────────────┐
//!                               │  formats::format_points_*           │
//!                               │  (SIMD: NEON/portable_simd/scalar)  │
//!                               └─────────────────────────────────────┘
//! ```
//!
//! # Modules
//!
//! - [`buffer`]: Pre-allocated point buffers and double-buffer swap mechanism
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
//!     lidar::{LidarDriver, LidarDriverBuffered},
//!     robosense::RobosenseDriver,
//! };
//!
//! let mut driver = RobosenseDriver::new();
//!
//! // Process packets
//! loop {
//!     let len = socket.recv(&mut buf)?;
//!     if let Some(metadata) = driver.process_packet_buffered(&buf[..len])? {
//!         let completed = driver.swap_buffer();
//!         // Use completed.x(), completed.y(), etc.
//!     }
//! }
//! ```

#![cfg_attr(feature = "portable_simd", feature(portable_simd))]

pub mod buffer;
pub mod common;
pub mod formats;
pub mod lidar;
pub mod ouster;
pub mod packet_source;
pub mod robosense;

// Re-exports for convenience
pub use buffer::{DoubleBuffer, PointBuffer};
pub use formats::PointFieldType;
pub use lidar::{
    Error, FrameMetadata, LidarDriver, LidarDriverBuffered, LidarFrame, Points, SensorType,
};
pub use packet_source::PacketSource;
