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

use crate::lidar::{Error, LidarDriver, LidarFrame, LidarFrameWriter, timestamp};

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

/// Return mode values from MSOP header byte 8
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ReturnMode {
    /// Dual return mode
    Dual = 0x00,
    /// Strongest return mode
    #[default]
    Strongest = 0x04,
    /// Last return mode
    Last = 0x05,
    /// Nearest return mode
    Nearest = 0x06,
}

impl From<u8> for ReturnMode {
    fn from(value: u8) -> Self {
        match value {
            0x00 => ReturnMode::Dual,
            0x05 => ReturnMode::Last,
            0x06 => ReturnMode::Nearest,
            _ => ReturnMode::Strongest,
        }
    }
}

impl std::fmt::Display for ReturnMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ReturnMode::Dual => write!(f, "Dual"),
            ReturnMode::Strongest => write!(f, "Strongest"),
            ReturnMode::Last => write!(f, "Last"),
            ReturnMode::Nearest => write!(f, "Nearest"),
        }
    }
}

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

/// IMU data from DIFOP packets (bytes 208-231)
#[derive(Clone, Debug, PartialEq)]
pub struct ImuData {
    /// Accelerometer X-axis (m/s²)
    pub accel_x: f32,
    /// Accelerometer Y-axis (m/s²)
    pub accel_y: f32,
    /// Accelerometer Z-axis (m/s²)
    pub accel_z: f32,
    /// Gyroscope X-axis (rad/s)
    pub gyro_x: f32,
    /// Gyroscope Y-axis (rad/s)
    pub gyro_y: f32,
    /// Gyroscope Z-axis (rad/s)
    pub gyro_z: f32,
}

/// Device information from DIFOP packets
#[derive(Clone, Debug, Default, PartialEq)]
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
    /// IMU data (accelerometer and gyroscope)
    pub imu: Option<ImuData>,
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

/// Robosense-specific LidarFrame implementation.
///
/// Stores point cloud data in Structure-of-Arrays layout for efficient SIMD
/// processing. The client owns this frame and provides it to the driver.
pub struct RobosenseLidarFrame {
    x: Vec<f32>,
    y: Vec<f32>,
    z: Vec<f32>,
    intensity: Vec<u8>,
    range: Vec<f32>,
    len: usize,
    timestamp: u64,
    frame_id: u32,
}

impl RobosenseLidarFrame {
    /// Create a new frame with specified capacity.
    ///
    /// Memory is allocated once at construction; no allocations occur during
    /// normal operation.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            x: vec![0.0; capacity],
            y: vec![0.0; capacity],
            z: vec![0.0; capacity],
            intensity: vec![0; capacity],
            range: vec![0.0; capacity],
            len: 0,
            timestamp: 0,
            frame_id: 0,
        }
    }

    /// Create a new frame with default capacity for Robosense E1R (~30k
    /// points).
    pub fn new() -> Self {
        Self::with_capacity(POINTS_PER_FRAME)
    }
}

impl Default for RobosenseLidarFrame {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: RobosenseLidarFrame contains only primitive types in Vecs,
// which are inherently Send.
unsafe impl Send for RobosenseLidarFrame {}

impl LidarFrame for RobosenseLidarFrame {
    fn reset(&mut self) {
        self.len = 0;
    }

    fn len(&self) -> usize {
        self.len
    }

    fn timestamp(&self) -> u64 {
        self.timestamp
    }

    fn frame_id(&self) -> u32 {
        self.frame_id
    }

    fn x(&self) -> &[f32] {
        &self.x[..self.len]
    }

    fn y(&self) -> &[f32] {
        &self.y[..self.len]
    }

    fn z(&self) -> &[f32] {
        &self.z[..self.len]
    }

    fn intensity(&self) -> &[u8] {
        &self.intensity[..self.len]
    }

    fn range(&self) -> &[f32] {
        &self.range[..self.len]
    }

    fn capacity(&self) -> usize {
        self.x.len()
    }
}

impl LidarFrameWriter for RobosenseLidarFrame {
    fn set_timestamp(&mut self, ts: u64) {
        self.timestamp = ts;
    }

    fn set_frame_id(&mut self, id: u32) {
        self.frame_id = id;
    }

    fn push(&mut self, x: f32, y: f32, z: f32, intensity: u8, range: f32) -> bool {
        if self.len >= self.capacity() {
            return false;
        }
        self.x[self.len] = x;
        self.y[self.len] = y;
        self.z[self.len] = z;
        self.intensity[self.len] = intensity;
        self.range[self.len] = range;
        self.len += 1;
        true
    }

    fn set_len(&mut self, len: usize) {
        self.len = len.min(self.capacity());
    }

    fn buffers_mut(&mut self) -> (&mut [f32], &mut [f32], &mut [f32], &mut [u8], &mut [f32]) {
        (
            &mut self.x,
            &mut self.y,
            &mut self.z,
            &mut self.intensity,
            &mut self.range,
        )
    }
}

/// Robosense E1R LiDAR driver
pub struct RobosenseDriver {
    /// Current frame ID (increments each frame)
    frame_id: u32,
    /// Last packet count (for frame boundary detection)
    last_pkt_cnt: u16,
    /// Device information from DIFOP
    device_info: DeviceInfo,
    /// Whether we've received any packets yet
    first_packet: bool,
    /// Frame was completed on previous call; next call must reset before processing
    needs_reset: bool,
    /// Return mode from MSOP header
    return_mode: ReturnMode,
    /// Whether to filter noisy points (PointAttribute == 2)
    filter_noisy: bool,
}

impl RobosenseDriver {
    /// Create a new Robosense E1R driver
    pub fn new() -> Self {
        Self {
            frame_id: 0,
            last_pkt_cnt: u16::MAX,
            device_info: DeviceInfo::default(),
            first_packet: true,
            needs_reset: false,
            return_mode: ReturnMode::default(),
            filter_noisy: true,
        }
    }

    /// Get the current return mode
    pub fn return_mode(&self) -> ReturnMode {
        self.return_mode
    }

    /// Set whether to filter noisy points
    pub fn set_filter_noisy(&mut self, filter: bool) {
        self.filter_noisy = filter;
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

        // IMU data: bytes 208-231, 6× big-endian f32
        // Verified against rs_driver RSE1DifopPkt struct (packed):
        //   id[8] + reserved1[93] + timeMode[1] + timeSyncStatus[1] +
        //   timestamp[10] + reserved2[95] = 208 → acceIx starts here.
        // Order: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        if data.len() >= 232 {
            let accel_x = f32::from_be_bytes([data[208], data[209], data[210], data[211]]);
            let accel_y = f32::from_be_bytes([data[212], data[213], data[214], data[215]]);
            let accel_z = f32::from_be_bytes([data[216], data[217], data[218], data[219]]);
            let gyro_x = f32::from_be_bytes([data[220], data[221], data[222], data[223]]);
            let gyro_y = f32::from_be_bytes([data[224], data[225], data[226], data[227]]);
            let gyro_z = f32::from_be_bytes([data[228], data[229], data[230], data[231]]);

            // Only store if values are finite (not NaN or Inf from uninitialized data)
            if accel_x.is_finite()
                && accel_y.is_finite()
                && accel_z.is_finite()
                && gyro_x.is_finite()
                && gyro_y.is_finite()
                && gyro_z.is_finite()
            {
                self.device_info.imu = Some(ImuData {
                    accel_x,
                    accel_y,
                    accel_z,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                });
            }
        }

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

        // Return mode: byte 8
        let return_mode = ReturnMode::from(data[8]);

        // Temperature: byte 31, Temp = LidarTmp - 80
        // Subtraction done in i16 to avoid overflow when data[31] > 127
        let temperature = (i16::from(data[31]) - 80) as i8;

        Ok(MsopHeader {
            pkt_cnt,
            timestamp_ns,
            temperature,
            return_mode,
        })
    }

    /// Parse a single data block and add the point to the frame
    fn parse_block<F: LidarFrameWriter>(&self, frame: &mut F, block: &[u8]) -> Result<(), Error> {
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

        // PointAttribute: byte 11 (1 = normal, 2 = noisy)
        // Filter noisy points when enabled
        if self.filter_noisy && block[11] == 2 {
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

        // Compute XYZ coordinates
        let x = radius * dir_x;
        let y = radius * dir_y;
        let z = radius * dir_z;

        // Single write directly to client's frame (no dual buffer!)
        // Range is provided by sensor protocol (radius in meters)
        if !frame.push(x, y, z, intensity, radius) {
            log::warn!(
                "Frame buffer full (capacity={}), dropping point",
                frame.capacity()
            );
        }

        Ok(())
    }

    /// Check if this packet indicates a new frame boundary
    fn is_frame_boundary(&self, pkt_cnt: u16) -> bool {
        // First packet always starts a new frame
        if self.first_packet {
            return true;
        }
        // Frame boundary when packet count wraps (high → low)
        pkt_cnt < self.last_pkt_cnt
    }
}

impl Default for RobosenseDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl LidarDriver for RobosenseDriver {
    fn process<F: LidarFrameWriter>(&mut self, frame: &mut F, data: &[u8]) -> Result<bool, Error> {
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
        self.return_mode = header.return_mode;

        // If previous call returned Ok(true), reset frame for the new cycle.
        // The boundary packet that triggered completion was already consumed,
        // so this packet is the first data packet of the new frame.
        if self.needs_reset {
            frame.reset();
            let ts = timestamp().unwrap_or(header.timestamp_ns);
            frame.set_timestamp(ts);
            frame.set_frame_id(self.frame_id);
            self.needs_reset = false;
        } else {
            // Check for frame boundary
            let is_boundary = self.is_frame_boundary(header.pkt_cnt);
            let frame_complete = is_boundary && !self.first_packet && !frame.is_empty();

            if frame_complete {
                self.frame_id = self.frame_id.wrapping_add(1);
                self.last_pkt_cnt = header.pkt_cnt;
                self.first_packet = false;
                self.needs_reset = true;
                return Ok(true);
            }

            // Start new frame if at boundary (first packet case)
            if is_boundary {
                frame.reset();
                let ts = timestamp().unwrap_or(header.timestamp_ns);
                frame.set_timestamp(ts);
                frame.set_frame_id(self.frame_id);
            }
        }

        self.first_packet = false;
        self.last_pkt_cnt = header.pkt_cnt;

        // Parse all data blocks directly into frame
        let data_start = MSOP_HEADER_SIZE;
        for i in 0..BLOCKS_PER_PACKET {
            let block_start = data_start + i * BLOCK_SIZE;
            let block_end = block_start + BLOCK_SIZE;
            if block_end <= data.len() {
                self.parse_block(frame, &data[block_start..block_end])?;
            }
        }

        Ok(false)
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
    /// Return mode
    return_mode: ReturnMode,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_robosense_frame_basic() {
        let mut frame = RobosenseLidarFrame::with_capacity(100);
        assert_eq!(frame.len(), 0);
        assert!(frame.is_empty());
        assert_eq!(frame.capacity(), 100);

        frame.push(1.0, 2.0, 3.0, 128, 3.74);
        assert_eq!(frame.len(), 1);
        assert!(!frame.is_empty());
        assert_eq!(frame.x()[0], 1.0);
        assert_eq!(frame.y()[0], 2.0);
        assert_eq!(frame.z()[0], 3.0);
        assert_eq!(frame.intensity()[0], 128);
        assert_eq!(frame.range()[0], 3.74);

        frame.reset();
        assert!(frame.is_empty());
        assert_eq!(frame.capacity(), 100);
    }

    #[test]
    fn test_robosense_frame_capacity() {
        let mut frame = RobosenseLidarFrame::with_capacity(2);
        assert!(frame.push(1.0, 1.0, 1.0, 1, 1.0));
        assert!(frame.push(2.0, 2.0, 2.0, 2, 2.0));
        assert!(!frame.push(3.0, 3.0, 3.0, 3, 3.0)); // At capacity
        assert_eq!(frame.len(), 2);
    }

    #[test]
    fn test_robosense_frame_metadata() {
        let mut frame = RobosenseLidarFrame::new();
        frame.set_timestamp(12345678);
        frame.set_frame_id(42);
        assert_eq!(frame.timestamp(), 12345678);
        assert_eq!(frame.frame_id(), 42);
    }

    #[test]
    fn test_driver_creation() {
        let driver = RobosenseDriver::new();
        assert_eq!(driver.frame_id, 0);
        assert!(driver.first_packet);
        assert!(!driver.needs_reset);
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
        let mut frame = RobosenseLidarFrame::new();
        let small_packet = vec![0u8; 100];
        let result = driver.process(&mut frame, &small_packet);
        assert!(result.is_err());
    }

    #[test]
    fn test_invalid_sync_bytes() {
        let mut driver = RobosenseDriver::new();
        let mut frame = RobosenseLidarFrame::new();
        let mut packet = vec![0u8; MSOP_PACKET_SIZE];
        // Wrong sync bytes
        packet[0..4].copy_from_slice(&[0x00, 0x00, 0x00, 0x00]);
        let result = driver.process(&mut frame, &packet);
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
    fn test_frame_boundary_first_packet() {
        let driver = RobosenseDriver::new();
        // First packet is always a boundary
        assert!(driver.is_frame_boundary(0));
        assert!(driver.is_frame_boundary(100));
    }

    #[test]
    fn test_frame_boundary_wrap() {
        let mut driver = RobosenseDriver::new();
        driver.first_packet = false;
        driver.last_pkt_cnt = 630;

        // pkt_cnt wraps from 630 to 0 → frame boundary
        assert!(driver.is_frame_boundary(0));
        // pkt_cnt wraps from 630 to 10 → frame boundary
        assert!(driver.is_frame_boundary(10));
        // Normal increment → not boundary
        assert!(!driver.is_frame_boundary(631));
    }

    #[test]
    fn test_multi_packet_frame_assembly() {
        let mut driver = RobosenseDriver::new();
        let mut frame = RobosenseLidarFrame::with_capacity(50_000);

        // Create valid MSOP packets with incrementing packet counts
        fn make_msop_packet(pkt_cnt: u16) -> Vec<u8> {
            let mut packet = vec![0u8; MSOP_PACKET_SIZE];
            // Sync bytes
            packet[0..4].copy_from_slice(&MSOP_SYNC);
            // Packet count (big-endian)
            packet[4..6].copy_from_slice(&pkt_cnt.to_be_bytes());
            // Timestamp (minimal valid)
            packet[10..20].fill(0);
            // Temperature
            packet[31] = 105; // 25°C (105 - 80)
            packet
        }

        // Process first packet (pkt_cnt=0)
        let packet0 = make_msop_packet(0);
        let result = driver.process(&mut frame, &packet0);
        assert!(result.is_ok());
        assert!(!result.unwrap()); // Not complete yet

        // Process second packet (pkt_cnt=1)
        let packet1 = make_msop_packet(1);
        let result = driver.process(&mut frame, &packet1);
        assert!(result.is_ok());
        assert!(!result.unwrap()); // Not complete yet

        // Process packet that wraps (pkt_cnt=0 again) - this signals frame complete
        // But first we need to add some points to the frame for it to be non-empty
        frame.push(1.0, 2.0, 3.0, 100, 2.0);

        let packet_wrap = make_msop_packet(0);
        let result = driver.process(&mut frame, &packet_wrap);
        assert!(result.is_ok());
        assert!(result.unwrap()); // Frame complete!
    }
}
