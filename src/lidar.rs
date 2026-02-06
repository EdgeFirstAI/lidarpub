// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Common LiDAR types and trait abstraction for multi-sensor support.
//!
//! This module provides sensor-agnostic types and traits for LiDAR processing,
//! enabling a unified interface across different sensor manufacturers.
//!
//! # Architecture
//!
//! The LiDAR processing follows a **client-owned frame** pattern:
//!
//! 1. Client creates frame objects via sensor-specific constructors
//! 2. Client provides mutable references to the driver for filling
//! 3. Driver writes points and metadata into the frame
//! 4. When complete (returns `Ok(true)`), client owns the filled frame
//! 5. Client can swap to another frame for the next fill cycle
//!
//! # Example
//!
//! ```ignore
//! // Single buffer for direct inline processing
//! let mut frame = OusterLidarFrame::with_capacity(30_000);
//!
//! loop {
//!     let n = socket.recv(&mut buf)?;
//!     if driver.process(&mut frame, &buf[..n])? {
//!         // Frame complete - process directly (zero-copy access)
//!         publish_pointcloud(frame.x(), frame.y(), frame.z(), frame.intensity());
//!         // Driver will call frame.reset() on next packet
//!     }
//! }
//! ```

use clap::ValueEnum;
use std::fmt;

/// Trait for LiDAR frame data (Structure-of-Arrays layout).
///
/// Implementations store point cloud data in separate vectors for efficient
/// SIMD processing. The client owns frame objects and provides them to drivers.
///
/// # Buffer Ownership
///
/// The client creates and owns all LidarFrame objects:
/// 1. Client creates frames: `OusterLidarFrame::with_capacity(N)`
/// 2. Client provides mutable reference: `driver.process(&mut frame, data)`
/// 3. Driver writes points and metadata into the frame
/// 4. When complete (returns `Ok(true)`), client owns the filled frame
/// 5. Client can swap to another frame for the next fill cycle
pub trait LidarFrame: Send {
    /// Reset frame for new data (sets point index to 0, does NOT touch
    /// vectors).
    fn reset(&mut self);

    /// Number of valid points currently in the frame.
    fn len(&self) -> usize;

    /// Returns true if no points are stored.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Frame timestamp in nanoseconds.
    fn timestamp(&self) -> u64;

    /// Frame sequence ID (wraps at u32::MAX).
    fn frame_id(&self) -> u32;

    /// X coordinates slice (up to len()).
    fn x(&self) -> &[f32];

    /// Y coordinates slice (up to len()).
    fn y(&self) -> &[f32];

    /// Z coordinates slice (up to len()).
    fn z(&self) -> &[f32];

    /// Intensity/reflectivity values (up to len()).
    fn intensity(&self) -> &[u8];

    /// Range from sensor in meters (provided by sensor protocol, NOT computed).
    ///
    /// - Robosense: `radius` from MSOP block data (already in meters)
    /// - Ouster: `depth` data in mm, converted to meters
    fn range(&self) -> &[f32];

    /// Current capacity (max points before reallocation).
    fn capacity(&self) -> usize;
}

/// Internal trait for drivers to write into frames.
///
/// This trait is not intended for public use - clients should only use
/// the read-only `LidarFrame` trait methods.
#[allow(dead_code)]
pub trait LidarFrameWriter: LidarFrame {
    /// Set the frame timestamp in nanoseconds.
    fn set_timestamp(&mut self, ts: u64);

    /// Set the frame sequence ID.
    fn set_frame_id(&mut self, id: u32);

    /// Push a point to the frame.
    ///
    /// Returns `true` if the point was added, `false` if at capacity.
    fn push(&mut self, x: f32, y: f32, z: f32, intensity: u8, range: f32) -> bool;

    /// Set the valid length of the frame.
    fn set_len(&mut self, len: usize);

    /// Get mutable access to all buffers at once for SIMD writes.
    ///
    /// Returns (x, y, z, intensity, range) slices. This method allows
    /// borrowing all buffers simultaneously, which is necessary for
    /// efficient SIMD processing where we need to write to multiple
    /// arrays in a single pass.
    #[allow(clippy::type_complexity)]
    fn buffers_mut(&mut self) -> (&mut [f32], &mut [f32], &mut [f32], &mut [u8], &mut [f32]);
}

/// Point cloud output structure (sensor-agnostic)
///
/// This structure uses a structure-of-arrays (SoA) layout for efficient
/// SIMD processing. It is used for compatibility with existing code that
/// expects owned vectors.
#[derive(Clone, Debug)]
pub struct Points {
    pub x: Vec<f32>,
    pub y: Vec<f32>,
    pub z: Vec<f32>,
    pub intensity: Vec<u8>,
}

#[allow(dead_code)]
impl Points {
    /// Create a new Points structure with pre-allocated capacity
    pub fn new(capacity: usize) -> Self {
        Self {
            x: vec![0.0; capacity],
            y: vec![0.0; capacity],
            z: vec![0.0; capacity],
            intensity: vec![0; capacity],
        }
    }

    /// Create an empty Points structure
    pub fn empty() -> Self {
        Self {
            x: Vec::new(),
            y: Vec::new(),
            z: Vec::new(),
            intensity: Vec::new(),
        }
    }

    /// Clear all points while retaining capacity
    pub fn clear(&mut self) {
        self.x.clear();
        self.y.clear();
        self.z.clear();
        self.intensity.clear();
    }

    /// Get the current number of points
    pub fn len(&self) -> usize {
        self.x.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }
}

/// Common error type for LiDAR operations
///
/// This enum consolidates all error types from sensor-specific drivers
/// into a single error type for consistent error handling.
#[derive(Debug)]
#[allow(dead_code)] // All variants defined for completeness; some used by library consumers
pub enum Error {
    /// I/O error (socket, file operations)
    Io(std::io::Error),
    /// Invalid packet data
    InvalidPacket(String),
    /// Buffer overflow (too many points for buffer capacity)
    BufferOverflow,
    /// System time error
    SystemTime(std::time::SystemTimeError),
    /// Shape error from ndarray operations
    Shape(ndarray::ShapeError),
    /// Unsupported data format
    UnsupportedFormat(String),
    /// Unexpected end of data at given byte position
    UnexpectedEnd(usize),
    /// Unknown packet type
    UnknownPacketType(u16),
    /// Configuration error
    Config(String),
    /// Too many columns in frame (Ouster-specific)
    TooManyColumns(usize),
    /// Insufficient columns in frame (Ouster-specific)
    InsufficientColumns(usize),
    /// Unsupported number of rows (Ouster-specific)
    UnsupportedRows(usize),
}

impl std::error::Error for Error {}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Error::Io(err) => write!(f, "I/O error: {}", err),
            Error::InvalidPacket(msg) => write!(f, "invalid packet: {}", msg),
            Error::BufferOverflow => write!(f, "buffer overflow"),
            Error::SystemTime(err) => write!(f, "system time error: {}", err),
            Error::Shape(err) => write!(f, "shape error: {}", err),
            Error::UnsupportedFormat(format) => write!(f, "unsupported format: {}", format),
            Error::UnexpectedEnd(len) => write!(f, "unexpected end of data at {} bytes", len),
            Error::UnknownPacketType(typ) => write!(f, "unknown packet type: {}", typ),
            Error::Config(msg) => write!(f, "configuration error: {}", msg),
            Error::TooManyColumns(cols) => write!(f, "too many columns: {}", cols),
            Error::InsufficientColumns(cols) => write!(f, "insufficient columns: {}", cols),
            Error::UnsupportedRows(rows) => write!(f, "unsupported number of rows: {}", rows),
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(err: std::io::Error) -> Self {
        Error::Io(err)
    }
}

impl From<std::time::SystemTimeError> for Error {
    fn from(err: std::time::SystemTimeError) -> Self {
        Error::SystemTime(err)
    }
}

impl From<ndarray::ShapeError> for Error {
    fn from(err: ndarray::ShapeError) -> Self {
        Error::Shape(err)
    }
}

/// Sensor type for CLI dispatch
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, ValueEnum)]
pub enum SensorType {
    /// Ouster OS series LiDAR sensors
    #[default]
    Ouster,
    /// Robosense E1R LiDAR sensor
    Robosense,
}

impl fmt::Display for SensorType {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            SensorType::Ouster => write!(f, "ouster"),
            SensorType::Robosense => write!(f, "robosense"),
        }
    }
}

/// Unified driver trait for all LiDAR sensors.
///
/// This trait uses the client-owned frame pattern where the client owns all
/// frame objects and the driver writes into mutable references.
///
/// # Returns
///
/// - `Ok(true)` - Frame is complete, ready for consumption
/// - `Ok(false)` - More packets needed to complete frame
/// - `Err(e)` - Processing error
pub trait LidarDriver: Send {
    /// Process UDP packet data into the provided frame.
    ///
    /// The driver handles:
    /// - Resetting the buffer on new frame boundary
    /// - Tracking internal indices and state
    /// - Setting timestamp and frame_id when appropriate
    ///
    /// # Arguments
    ///
    /// * `frame` - Mutable reference to a frame implementing `LidarFrameWriter`
    /// * `data` - Raw UDP packet data
    ///
    /// # Returns
    ///
    /// - `Ok(true)` - Frame is complete and ready for consumption
    /// - `Ok(false)` - More packets needed to complete the frame
    /// - `Err(e)` - Processing error
    fn process<F: LidarFrameWriter>(&mut self, frame: &mut F, data: &[u8]) -> Result<bool, Error>;
}

/// Get current timestamp in nanoseconds.
///
/// On Linux, uses `CLOCK_MONOTONIC_RAW` for best accuracy.
/// On other platforms, falls back to `SystemTime`.
#[cfg(target_os = "linux")]
pub fn timestamp() -> Result<u64, Error> {
    let mut tp = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let err = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC_RAW, &mut tp) };
    if err != 0 {
        return Err(std::io::Error::last_os_error().into());
    }

    Ok(tp.tv_sec as u64 * 1_000_000_000 + tp.tv_nsec as u64)
}

#[cfg(not(target_os = "linux"))]
pub fn timestamp() -> Result<u64, Error> {
    let now = std::time::SystemTime::now();
    let duration = now.duration_since(std::time::UNIX_EPOCH)?;
    Ok(duration.as_nanos() as u64)
}
