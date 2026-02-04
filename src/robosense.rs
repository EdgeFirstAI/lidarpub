// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Robosense E1R LiDAR driver implementation.
//!
//! The E1R is a solid-state flash LiDAR with:
//! - 120° horizontal FOV (-60° to +60°)
//! - 90° vertical FOV (-45° to +45°)
//! - ~26,000 points per frame at 10 Hz
//! - 30m range @ 10% reflectivity
//!
//! # Packet Structure
//!
//! ## MSOP (Main Data Stream Output Protocol) - 1200 bytes, port 6699
//! - Header: 32 bytes (sync, packet count, timestamp, etc.)
//! - Data: 96 blocks × 12 bytes = 1152 bytes
//! - Tail: 16 bytes
//!
//! ## DIFOP (Device Information Output Protocol) - 256 bytes, port 7788
//! - Device info: serial number, firmware version, network config, time sync
//!   status

use crate::{
    buffer::DoubleBuffer,
    lidar::{
        Error, FrameMetadata, LidarDriver, LidarDriverBuffered, LidarFrame, Points, timestamp,
    },
};

/// MSOP packet sync bytes: 0x55, 0xaa, 0x5a, 0xa5
const MSOP_SYNC: [u8; 4] = [0x55, 0xaa, 0x5a, 0xa5];

/// MSOP packet total size in bytes
const MSOP_PACKET_SIZE: usize = 1200;

/// MSOP header size in bytes
const MSOP_HEADER_SIZE: usize = 32;

/// Number of data blocks per MSOP packet
const BLOCKS_PER_PACKET: usize = 96;

/// Size of each data block in bytes
const BLOCK_SIZE: usize = 12;

/// Distance resolution in meters (5mm)
const DISTANCE_RESOLUTION: f32 = 0.005;

/// Scale factor for direction vectors (2^15)
const DIRECTION_SCALE: f32 = 32768.0;

/// Expected points per frame (~26,000 at 10Hz)
const POINTS_PER_FRAME: usize = 30_000;

/// DIFOP packet sync bytes
const DIFOP_SYNC: [u8; 8] = [0xa5, 0xff, 0x00, 0x5a, 0x11, 0x11, 0x55, 0x55];

/// DIFOP packet size
const DIFOP_PACKET_SIZE: usize = 256;

/// Time synchronization mode values
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TimeSyncMode {
    /// Internal oscillator timing
    #[default]
    Internal = 0x00,
    /// PTP E2E time synchronization
    PtpE2E = 0x02,
    /// gPTP (IEEE 802.1AS) time synchronization
    Gptp = 0x03,
}

impl From<u8> for TimeSyncMode {
    fn from(value: u8) -> Self {
        match value {
            0x02 => TimeSyncMode::PtpE2E,
            0x03 => TimeSyncMode::Gptp,
            _ => TimeSyncMode::Internal,
        }
    }
}

/// Time synchronization status
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TimeSyncStatus {
    /// Time sync failed
    #[default]
    Failed = 0x00,
    /// Time sync successful
    Success = 0x01,
    /// Time sync timeout
    Timeout = 0x02,
}

impl From<u8> for TimeSyncStatus {
    fn from(value: u8) -> Self {
        match value {
            0x01 => TimeSyncStatus::Success,
            0x02 => TimeSyncStatus::Timeout,
            _ => TimeSyncStatus::Failed,
        }
    }
}

/// Device information from DIFOP packets
#[derive(Clone, Debug, Default)]
#[allow(dead_code)]
pub struct DeviceInfo {
    /// Device serial number
    pub serial_number: [u8; 6],
    /// Software version (3 bytes)
    pub sw_version: [u8; 3],
    /// Time synchronization mode
    pub timesync_mode: TimeSyncMode,
    /// Time synchronization status
    pub timesync_status: TimeSyncStatus,
    /// LiDAR temperature in Celsius
    pub temperature: i8,
    /// Local IP address
    pub local_ip: [u8; 4],
    /// MSOP destination port
    pub msop_port: u16,
    /// DIFOP destination port
    pub difop_port: u16,
}

impl DeviceInfo {
    /// Get serial number as a hex string
    pub fn serial_string(&self) -> String {
        self.serial_number
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect()
    }

    /// Get software version as a string (e.g., "1.2.3")
    pub fn version_string(&self) -> String {
        format!(
            "{}.{}.{}",
            self.sw_version[0], self.sw_version[1], self.sw_version[2]
        )
    }
}

/// Robosense E1R LiDAR driver
pub struct RobosenseDriver {
    /// Current frame ID (increments each frame)
    frame_id: u32,
    /// Timestamp of current frame
    timestamp: u64,
    /// Double buffer for zero-allocation operation
    double_buffer: DoubleBuffer,
    /// Legacy point cloud buffer (for backward compatibility)
    points: Points,
    /// Number of points accumulated in current frame
    n_points: usize,
    /// Last packet count (for frame boundary detection)
    last_pkt_cnt: u16,
    /// Device information from DIFOP
    device_info: DeviceInfo,
    /// Whether we've received any packets yet
    first_packet: bool,
}

impl RobosenseDriver {
    /// Create a new Robosense E1R driver
    pub fn new() -> Self {
        Self {
            frame_id: 0,
            timestamp: 0,
            double_buffer: DoubleBuffer::new(POINTS_PER_FRAME),
            points: Points::new(POINTS_PER_FRAME),
            n_points: 0,
            last_pkt_cnt: u16::MAX,
            device_info: DeviceInfo::default(),
            first_packet: true,
        }
    }

    /// Process a DIFOP packet for device information
    ///
    /// DIFOP packets are 256 bytes and contain device configuration
    /// and status information.
    pub fn process_difop(&mut self, data: &[u8]) -> Result<(), Error> {
        if data.len() < DIFOP_PACKET_SIZE {
            return Err(Error::InvalidPacket(format!(
                "DIFOP packet too small: {} bytes",
                data.len()
            )));
        }

        // Verify sync bytes
        if data[0..8] != DIFOP_SYNC {
            return Err(Error::InvalidPacket("Invalid DIFOP sync bytes".to_string()));
        }

        // Parse device info
        // SW Version: bytes 16-18
        self.device_info.sw_version.copy_from_slice(&data[16..19]);

        // Serial Number: bytes 20-25
        self.device_info
            .serial_number
            .copy_from_slice(&data[20..26]);

        // Local IP: bytes 44-47
        self.device_info.local_ip.copy_from_slice(&data[44..48]);

        // MSOP local port: bytes 62-63
        self.device_info.msop_port = u16::from_be_bytes([data[62], data[63]]);

        // DIFOP local port: bytes 70-71
        self.device_info.difop_port = u16::from_be_bytes([data[70], data[71]]);

        // Time sync mode: byte 101
        self.device_info.timesync_mode = TimeSyncMode::from(data[101]);

        // Time sync status: byte 102
        self.device_info.timesync_status = TimeSyncStatus::from(data[102]);

        Ok(())
    }

    /// Get current device information
    pub fn device_info(&self) -> &DeviceInfo {
        &self.device_info
    }

    /// Parse MSOP header and extract metadata
    fn parse_header(&self, data: &[u8]) -> Result<MsopHeader, Error> {
        if data.len() < MSOP_HEADER_SIZE {
            return Err(Error::UnexpectedEnd(data.len()));
        }

        // Verify sync bytes
        if data[0..4] != MSOP_SYNC {
            return Err(Error::InvalidPacket("Invalid MSOP sync bytes".to_string()));
        }

        // Packet count (2 bytes, big-endian)
        let pkt_cnt = u16::from_be_bytes([data[4], data[5]]);

        // Timestamp: bytes 10-19
        // High 6 bytes = seconds, low 4 bytes = microseconds
        let seconds = u64::from_be_bytes([
            0, 0, data[10], data[11], data[12], data[13], data[14], data[15],
        ]);
        let microseconds = u32::from_be_bytes([data[16], data[17], data[18], data[19]]);
        let timestamp_ns = seconds * 1_000_000_000 + microseconds as u64 * 1_000;

        // Temperature: byte 31, Temp = LidarTmp - 80
        let temperature = data[31] as i8 - 80;

        Ok(MsopHeader {
            pkt_cnt,
            timestamp_ns,
            temperature,
        })
    }

    /// Parse a single data block and add the point to the buffer
    fn parse_block(&mut self, block: &[u8]) -> Result<(), Error> {
        if block.len() < BLOCK_SIZE {
            return Err(Error::UnexpectedEnd(block.len()));
        }

        // TimeOffset: bytes 0-1 (not used for point computation)
        // let _time_offset = u16::from_be_bytes([block[0], block[1]]);

        // Radius: bytes 2-3, distance in 5mm units
        let radius_raw = u16::from_be_bytes([block[2], block[3]]);

        // Skip invalid points (radius = 0)
        if radius_raw == 0 {
            return Ok(());
        }

        let radius = radius_raw as f32 * DISTANCE_RESOLUTION;

        // Direction vectors: signed 16-bit, divide by 2^15 for unit vector
        // DirVectorX: bytes 4-5
        let dir_x = i16::from_be_bytes([block[4], block[5]]) as f32 / DIRECTION_SCALE;
        // DirVectorY: bytes 6-7
        let dir_y = i16::from_be_bytes([block[6], block[7]]) as f32 / DIRECTION_SCALE;
        // DirVectorZ: bytes 8-9
        let dir_z = i16::from_be_bytes([block[8], block[9]]) as f32 / DIRECTION_SCALE;

        // Intensity: byte 10
        let intensity = block[10];

        // PointAttribute: byte 11 (1 = normal, 2 = noisy)
        let _attribute = block[11];

        // Compute XYZ coordinates
        let x = radius * dir_x;
        let y = radius * dir_y;
        let z = radius * dir_z;

        // Add point to double buffer (with pre-computed range)
        self.double_buffer
            .filling_mut()
            .push(x, y, z, intensity, radius);

        // Also add to legacy points buffer for backward compatibility
        if self.n_points < self.points.x.len() {
            self.points.x[self.n_points] = x;
            self.points.y[self.n_points] = y;
            self.points.z[self.n_points] = z;
            self.points.intensity[self.n_points] = intensity;
            self.n_points += 1;
        }

        Ok(())
    }

    /// Check if this packet indicates a new frame boundary
    fn is_frame_boundary(&self, pkt_cnt: u16) -> bool {
        // Frame boundary when packet count wraps (goes from high to 0)
        // or on first packet
        self.first_packet || (pkt_cnt < self.last_pkt_cnt && self.last_pkt_cnt != u16::MAX)
    }

    /// Finalize the current frame and return it (legacy interface)
    fn finalize_frame(&mut self) -> LidarFrame {
        let frame = LidarFrame {
            timestamp: self.timestamp,
            frame_id: self.frame_id,
            points: self.points.clone(),
            n_points: self.n_points,
        };

        // Reset for next frame
        self.frame_id = self.frame_id.wrapping_add(1);
        self.n_points = 0;

        // Note: We intentionally do NOT zero the point buffers.
        // The n_points counter tracks valid data - no need to clear old values.

        frame
    }

    /// Finalize frame for buffered interface (zero-allocation)
    #[allow(dead_code)] // Used by LidarDriverBuffered implementation
    fn finalize_frame_buffered(&mut self) -> FrameMetadata {
        let n_points = self.double_buffer.filling().len();

        let metadata = FrameMetadata {
            timestamp: self.timestamp,
            frame_id: self.frame_id,
            n_points,
        };

        // Reset for next frame
        self.frame_id = self.frame_id.wrapping_add(1);
        self.n_points = 0;

        metadata
    }

    /// Process a packet using the buffered interface (returns metadata only)
    #[allow(dead_code)] // Used by LidarDriverBuffered implementation
    fn process_packet_internal(&mut self, data: &[u8]) -> Result<Option<FrameMetadata>, Error> {
        // Validate packet size
        if data.len() < MSOP_PACKET_SIZE {
            return Err(Error::InvalidPacket(format!(
                "MSOP packet too small: {} bytes, expected {}",
                data.len(),
                MSOP_PACKET_SIZE
            )));
        }

        // Parse header
        let header = self.parse_header(data)?;

        // Check for frame boundary
        let frame_complete = self.is_frame_boundary(header.pkt_cnt);
        let mut result = None;

        if frame_complete && !self.first_packet && !self.double_buffer.filling().is_empty() {
            // Complete the previous frame
            result = Some(self.finalize_frame_buffered());
            // Swap buffers for the completed frame
            self.double_buffer.swap();
        }

        // Update timestamp on first packet of new frame
        if frame_complete || self.first_packet {
            // Use local timestamp if available, otherwise use packet timestamp
            self.timestamp = timestamp().unwrap_or(header.timestamp_ns);
        }

        self.first_packet = false;
        self.last_pkt_cnt = header.pkt_cnt;

        // Parse all data blocks
        let data_start = MSOP_HEADER_SIZE;
        for i in 0..BLOCKS_PER_PACKET {
            let block_start = data_start + i * BLOCK_SIZE;
            let block_end = block_start + BLOCK_SIZE;
            if block_end <= data.len() {
                self.parse_block(&data[block_start..block_end])?;
            }
        }

        Ok(result)
    }
}

impl Default for RobosenseDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl LidarDriver for RobosenseDriver {
    fn process_packet(&mut self, data: &[u8]) -> Result<Option<LidarFrame>, Error> {
        // Validate packet size
        if data.len() < MSOP_PACKET_SIZE {
            return Err(Error::InvalidPacket(format!(
                "MSOP packet too small: {} bytes, expected {}",
                data.len(),
                MSOP_PACKET_SIZE
            )));
        }

        // Parse header
        let header = self.parse_header(data)?;

        // Check for frame boundary
        let frame_complete = self.is_frame_boundary(header.pkt_cnt);
        let mut result = None;

        if frame_complete && !self.first_packet && self.n_points > 0 {
            // Complete the previous frame
            result = Some(self.finalize_frame());
            // Also swap the double buffer
            self.double_buffer.swap();
        }

        // Update timestamp on first packet of new frame
        if frame_complete || self.first_packet {
            // Use local timestamp if available, otherwise use packet timestamp
            self.timestamp = timestamp().unwrap_or(header.timestamp_ns);
        }

        self.first_packet = false;
        self.last_pkt_cnt = header.pkt_cnt;

        // Parse all data blocks
        let data_start = MSOP_HEADER_SIZE;
        for i in 0..BLOCKS_PER_PACKET {
            let block_start = data_start + i * BLOCK_SIZE;
            let block_end = block_start + BLOCK_SIZE;
            if block_end <= data.len() {
                self.parse_block(&data[block_start..block_end])?;
            }
        }

        Ok(result)
    }
}

impl LidarDriverBuffered for RobosenseDriver {
    fn process_packet_buffered(&mut self, data: &[u8]) -> Result<Option<FrameMetadata>, Error> {
        self.process_packet_internal(data)
    }

    fn buffer(&self) -> &DoubleBuffer {
        &self.double_buffer
    }

    fn buffer_mut(&mut self) -> &mut DoubleBuffer {
        &mut self.double_buffer
    }
}

/// Parsed MSOP header information
struct MsopHeader {
    /// Packet sequence number (wraps per frame)
    pkt_cnt: u16,
    /// Timestamp in nanoseconds
    timestamp_ns: u64,
    /// LiDAR temperature in Celsius
    #[allow(dead_code)]
    temperature: i8,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_driver_creation() {
        let driver = RobosenseDriver::new();
        assert_eq!(driver.frame_id, 0);
        assert_eq!(driver.n_points, 0);
        assert!(driver.first_packet);
        assert_eq!(driver.double_buffer.capacity(), POINTS_PER_FRAME);
    }

    #[test]
    fn test_device_info_strings() {
        let info = DeviceInfo {
            serial_number: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
            sw_version: [1, 2, 3],
            ..Default::default()
        };

        assert_eq!(info.serial_string(), "010203040506");
        assert_eq!(info.version_string(), "1.2.3");
    }

    #[test]
    fn test_timesync_mode_from() {
        assert_eq!(TimeSyncMode::from(0x00), TimeSyncMode::Internal);
        assert_eq!(TimeSyncMode::from(0x02), TimeSyncMode::PtpE2E);
        assert_eq!(TimeSyncMode::from(0x03), TimeSyncMode::Gptp);
        assert_eq!(TimeSyncMode::from(0xFF), TimeSyncMode::Internal);
    }

    #[test]
    fn test_timesync_status_from() {
        assert_eq!(TimeSyncStatus::from(0x00), TimeSyncStatus::Failed);
        assert_eq!(TimeSyncStatus::from(0x01), TimeSyncStatus::Success);
        assert_eq!(TimeSyncStatus::from(0x02), TimeSyncStatus::Timeout);
        assert_eq!(TimeSyncStatus::from(0xFF), TimeSyncStatus::Failed);
    }

    #[test]
    fn test_invalid_packet_size() {
        let mut driver = RobosenseDriver::new();
        let small_packet = vec![0u8; 100];
        let result = driver.process_packet(&small_packet);
        assert!(result.is_err());
    }

    #[test]
    fn test_invalid_sync_bytes() {
        let mut driver = RobosenseDriver::new();
        let mut packet = vec![0u8; MSOP_PACKET_SIZE];
        // Wrong sync bytes
        packet[0..4].copy_from_slice(&[0x00, 0x00, 0x00, 0x00]);
        let result = driver.process_packet(&packet);
        assert!(result.is_err());
    }

    #[test]
    fn test_coordinate_calculation() {
        // Test that coordinate calculation matches the spec:
        // X = radius * (DirVectorX / 2^15)
        // Y = radius * (DirVectorY / 2^15)
        // Z = radius * (DirVectorZ / 2^15)

        // Example: radius = 1020 (5.1m), direction = (0.5, 0.5, 0.707)
        let radius_raw: u16 = 1020;
        let radius = radius_raw as f32 * DISTANCE_RESOLUTION; // 5.1m

        // Direction vector components (scaled by 2^15 = 32768)
        let dir_x_raw: i16 = 16384; // 0.5 * 32768
        let dir_y_raw: i16 = 16384; // 0.5 * 32768
        let dir_z_raw: i16 = 23170; // ~0.707 * 32768

        let dir_x = dir_x_raw as f32 / DIRECTION_SCALE;
        let dir_y = dir_y_raw as f32 / DIRECTION_SCALE;
        let dir_z = dir_z_raw as f32 / DIRECTION_SCALE;

        let x = radius * dir_x;
        let y = radius * dir_y;
        let z = radius * dir_z;

        // Verify calculations
        assert!((x - 2.55).abs() < 0.01);
        assert!((y - 2.55).abs() < 0.01);
        assert!((z - 3.606).abs() < 0.01);
    }

    #[test]
    fn test_double_buffer_integration() {
        let driver = RobosenseDriver::new();
        assert_eq!(driver.buffer().filling().len(), 0);
        assert_eq!(driver.buffer().ready().len(), 0);
    }
}
