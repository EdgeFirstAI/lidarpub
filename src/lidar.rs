// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Common LiDAR types and trait abstraction for multi-sensor support.
//!
//! This module provides sensor-agnostic types and traits for LiDAR processing,
//! enabling a unified interface across different sensor manufacturers.

use crate::buffer::DoubleBuffer;
use clap::ValueEnum;
use std::fmt;

/// Point cloud output structure (sensor-agnostic)
///
/// This structure uses a structure-of-arrays (SoA) layout for efficient
/// SIMD processing. It is used for compatibility with existing code that
/// expects owned vectors.
///
/// For zero-allocation operation, prefer using [`crate::buffer::PointBuffer`]
/// with [`DoubleBuffer`] instead.
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

/// Complete frame from LiDAR sensor (legacy interface)
///
/// This structure is maintained for backward compatibility. New code should
/// prefer using [`FrameMetadata`] with [`DoubleBuffer`] for zero-allocation
/// operation.
pub struct LidarFrame {
    /// Timestamp in nanoseconds
    pub timestamp: u64,
    /// Frame sequence ID
    pub frame_id: u32,
    /// Point cloud data
    pub points: Points,
    /// Number of valid points in the frame
    pub n_points: usize,
}

/// Metadata for a completed frame.
///
/// This lightweight structure contains frame metadata without the actual
/// point data. The point data is accessed through the driver's
/// [`DoubleBuffer`] via the [`LidarDriverBuffered`] trait.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // Public API - fields accessed by library consumers
pub struct FrameMetadata {
    /// Timestamp in nanoseconds (monotonic or synchronized)
    pub timestamp: u64,
    /// Frame sequence ID (wraps at u32::MAX)
    pub frame_id: u32,
    /// Number of valid points in the frame
    pub n_points: usize,
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

/// Trait for LiDAR driver implementations (legacy interface)
///
/// This trait provides a unified interface for processing UDP packets from
/// various LiDAR sensors. Implementations handle packet parsing, frame
/// assembly, and point cloud computation internally.
///
/// **Note**: For zero-allocation operation, implement [`LidarDriverBuffered`]
/// instead, which uses [`DoubleBuffer`] for point data.
pub trait LidarDriver: Send {
    /// Process a UDP packet, returning a complete frame when ready
    ///
    /// # Returns
    /// - `Ok(None)` if more packets are needed to complete the frame
    /// - `Ok(Some(frame))` when a complete frame is ready
    /// - `Err` on packet parsing errors
    fn process_packet(&mut self, data: &[u8]) -> Result<Option<LidarFrame>, Error>;
}

/// Trait for zero-allocation LiDAR driver implementations.
///
/// This trait uses a [`DoubleBuffer`] to eliminate allocations during
/// steady-state operation. When a frame completes, call [`Self::swap_buffer`]
/// to get access to the completed data.
///
/// # Example
///
/// ```ignore
/// let metadata = driver.process_packet_buffered(&packet_data)?;
/// if let Some(meta) = metadata {
///     let completed = driver.swap_buffer();
///     // Use completed.x(), completed.y(), etc.
/// }
/// ```
#[allow(dead_code)] // Public API - methods used by library consumers
pub trait LidarDriverBuffered: Send {
    /// Process a UDP packet, returning metadata when a frame completes.
    ///
    /// The actual point data is accessible via [`Self::buffer`] or
    /// [`Self::swap_buffer`].
    ///
    /// # Returns
    /// - `Ok(None)` if more packets are needed
    /// - `Ok(Some(metadata))` when a frame completes
    /// - `Err` on packet parsing errors
    fn process_packet_buffered(&mut self, data: &[u8]) -> Result<Option<FrameMetadata>, Error>;

    /// Access the double buffer directly.
    fn buffer(&self) -> &DoubleBuffer;

    /// Access the double buffer mutably.
    fn buffer_mut(&mut self) -> &mut DoubleBuffer;

    /// Swap buffers and return a reference to the completed frame data.
    ///
    /// After calling, the filling buffer is cleared for the next frame.
    #[inline]
    fn swap_buffer(&mut self) -> &crate::buffer::PointBuffer {
        self.buffer_mut().swap()
    }
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
