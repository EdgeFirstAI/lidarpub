// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Ouster LiDAR driver implementation.
//!
//! Supports OS0, OS1, OS2, and OSDome series sensors with the RNG15_RFL8_NIR8
//! UDP profile.

// Allow these for API stability - methods intentionally take &self even though
// types are Copy for potential future non-Copy changes, and some methods are
// available for future use or testing purposes.
#![allow(clippy::wrong_self_convention)]
#![allow(dead_code)]

use crate::lidar::{Error, LidarDriver, LidarFrame, LidarFrameWriter, timestamp};
use ndarray::Array2;
use serde::{Deserialize, Serialize};
use std::{f32::consts::PI, fmt};
use tracing::{instrument, warn};

// SIMD imports: NEON on aarch64 (stable), portable_simd elsewhere (nightly)
#[cfg(all(feature = "portable_simd", not(target_arch = "aarch64")))]
use std::simd::Simd;

#[cfg(target_arch = "aarch64")]
use std::arch::aarch64::*;

#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    pub udp_dest: String,
    pub udp_port_lidar: u16,
    pub udp_profile_lidar: String,
    pub lidar_mode: String,
    pub azimuth_window: [u32; 2],
    pub timestamp_mode: String,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            udp_dest: "".to_owned(),
            udp_port_lidar: 7502,
            udp_profile_lidar: "RNG15_RFL8_NIR8".to_owned(),
            lidar_mode: "1024x10".to_owned(),
            azimuth_window: [0, 360000],
            timestamp_mode: "TIME_FROM_INTERNAL_OSC".to_owned(),
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SensorInfo {
    pub status: String,
    pub build_rev: String,
    pub prod_sn: String,
    pub prod_pn: String,
    pub prod_line: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct LidarDataFormat {
    pub udp_profile_lidar: String,
    pub udp_profile_imu: String,
    pub columns_per_packet: usize,
    pub columns_per_frame: usize,
    pub pixels_per_column: usize,
    pub column_window: [usize; 2],
    pub pixel_shift_by_row: Vec<i16>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct BeamIntrinsics {
    pub beam_azimuth_angles: Vec<f32>,
    pub beam_altitude_angles: Vec<f32>,
    pub beam_to_lidar_transform: Vec<f32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Parameters {
    pub sensor_info: SensorInfo,
    pub lidar_data_format: LidarDataFormat,
    pub beam_intrinsics: BeamIntrinsics,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ShotLimiting {
    Normal,
    Imminent(u8),
    Limiting(u8),
    Invalid(u8),
}

impl fmt::Display for ShotLimiting {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        match self {
            ShotLimiting::Normal => write!(f, "Normal"),
            ShotLimiting::Imminent(seconds) => write!(f, "Limiting in {} seconds", seconds),
            ShotLimiting::Limiting(range) => {
                write!(f, "Limiting to approximately {}% range", range)
            }
            ShotLimiting::Invalid(val) => write!(f, "Invalid shot limiting value: {}", val),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ShutdownStatus {
    Normal,
    Imminent(u8),
    Invalid(u8),
}

impl fmt::Display for ShutdownStatus {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        match self {
            ShutdownStatus::Normal => write!(f, "Normal"),
            ShutdownStatus::Imminent(seconds) => {
                write!(f, "Shutdown imminent in {} seconds", seconds)
            }
            ShutdownStatus::Invalid(val) => write!(f, "Invalid shutdown status value: {}", val),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlertStatus {
    Normal,
    Active(u8),
    Overflow(u8),
}

impl AlertStatus {
    pub fn from_flags(flags: u8) -> AlertStatus {
        if flags & (1 << 6) != 0 {
            AlertStatus::Overflow(flags & 0b00111111)
        } else if flags & (1 << 7) != 0 {
            AlertStatus::Active(flags & 0b00111111)
        } else {
            AlertStatus::Normal
        }
    }
}

impl fmt::Display for AlertStatus {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        match self {
            AlertStatus::Normal => write!(f, "Normal"),
            AlertStatus::Active(cursor) => write!(f, "Alert: {}", cursor),
            AlertStatus::Overflow(cursor) => write!(f, "Alert (overflow): {}", cursor),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Header {
    /// Identifies lidar data vs. other packets in stream.
    /// Packet Type is 0x1 for Lidar packets.
    pub packet_type: u16,
    /// Index of the lidar scan, increments every time the sensor completes a
    /// rotation, crossing the zero azimuth angle.
    pub frame_id: u16,
    /// Initialization ID. Updates on every reinit, which may be triggered by
    /// the user or an error, and every reboot.
    pub init_id: u32,
    /// Serial number of the sensor. This value is unique to each sensor and
    /// can be found on a sticker affixed to the top of the sensor.
    pub serial_number: u64,
    /// Indicates the shot limiting status of the sensor.
    pub shot_limiting: ShotLimiting,
    /// Indicates whether thermal shutdown is imminent.
    pub shutdown_status: ShutdownStatus,
    /// Alert flags is a bitmask of various sensor alerts.
    pub alert_status: AlertStatus,
}

impl Header {
    /// Length of the header in bytes/octets.
    pub const LEN: usize = 32;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct HeaderSlice<'a> {
    slice: &'a [u8],
}

impl<'a> HeaderSlice<'a> {
    pub fn from_slice(slice: &'a [u8]) -> Result<HeaderSlice<'a>, Error> {
        if slice.len() < Header::LEN {
            return Err(Error::UnexpectedEnd(slice.len()));
        }

        let packet_type = u16::from_le_bytes([slice[0], slice[1]]);
        if packet_type != 1 {
            return Err(Error::UnknownPacketType(packet_type));
        }

        Ok(HeaderSlice { slice })
    }

    pub fn to_header(&self) -> Header {
        Header {
            packet_type: self.packet_type(),
            frame_id: self.frame_id(),
            init_id: self.init_id(),
            serial_number: self.serial_number(),
            shot_limiting: self.shot_limiting(),
            shutdown_status: self.shutdown_status(),
            alert_status: self.alert_status(),
        }
    }

    pub fn packet_type(&self) -> u16 {
        u16::from_le_bytes([self.slice[0], self.slice[1]])
    }

    pub fn frame_id(&self) -> u16 {
        u16::from_le_bytes([self.slice[2], self.slice[3]])
    }

    pub fn init_id(&self) -> u32 {
        u32::from_le_bytes([self.slice[4], self.slice[5], self.slice[6], 0])
    }

    pub fn serial_number(&self) -> u64 {
        u64::from_le_bytes([
            self.slice[7],
            self.slice[8],
            self.slice[9],
            self.slice[10],
            self.slice[11],
            0,
            0,
            0,
        ])
    }

    pub fn shot_limiting(&self) -> ShotLimiting {
        match self.slice[19] >> 4 {
            0 => ShotLimiting::Normal,
            1 => ShotLimiting::Imminent(self.shot_limiting_countdown()),
            2 => ShotLimiting::Limiting(3),
            3 => ShotLimiting::Limiting(6),
            4 => ShotLimiting::Limiting(9),
            5 => ShotLimiting::Limiting(12),
            6 => ShotLimiting::Limiting(16),
            7 => ShotLimiting::Limiting(21),
            8 => ShotLimiting::Limiting(25),
            9 => ShotLimiting::Limiting(27),
            val => ShotLimiting::Invalid(val),
        }
    }

    pub fn shutdown_status(&self) -> ShutdownStatus {
        match self.slice[18] >> 4 {
            0 => ShutdownStatus::Normal,
            1 => ShutdownStatus::Imminent(self.shutdown_countdown()),
            val => ShutdownStatus::Invalid(val),
        }
    }

    pub fn shot_limiting_countdown(&self) -> u8 {
        self.slice[17]
    }

    pub fn shutdown_countdown(&self) -> u8 {
        self.slice[16]
    }

    pub fn alert_status(&self) -> AlertStatus {
        AlertStatus::from_flags(self.slice[12])
    }

    /// Returns the column at the given index.
    pub fn column(&self, rows: usize, col: usize) -> Result<ColumnHeaderSlice<'a>, Error> {
        let offset = Header::LEN + (ColumnHeader::LEN + DataBlock::LEN * rows) * col;
        ColumnHeaderSlice::from_slice(&self.slice[offset..])
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ColumnHeader {
    /// Timestamp of the measurement in nanoseconds.
    pub timestamp: u64,
    /// Sequentially incrementing measurement counting up from 0 to 511,
    /// or 0 to 1023, or 0 to 2047 depending on lidar_mode.
    pub measurement_id: u16,
    /// Indicates validity of the measurements. Status is true for valid
    /// measurements. Status is false for dropped or disabled columns.
    pub status: bool,
}

impl ColumnHeader {
    /// Length of the column header in bytes/octets.
    pub const LEN: usize = 12;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ColumnHeaderSlice<'a> {
    slice: &'a [u8],
}

impl<'a> ColumnHeaderSlice<'a> {
    pub fn from_slice(slice: &'a [u8]) -> Result<ColumnHeaderSlice<'a>, Error> {
        if slice.len() < ColumnHeader::LEN {
            return Err(Error::UnexpectedEnd(slice.len()));
        }

        Ok(ColumnHeaderSlice { slice })
    }

    pub fn to_column(&self) -> ColumnHeader {
        ColumnHeader {
            timestamp: self.timestamp(),
            measurement_id: self.measurement_id(),
            status: self.status(),
        }
    }

    pub fn timestamp(&self) -> u64 {
        u64::from_le_bytes([
            self.slice[0],
            self.slice[1],
            self.slice[2],
            self.slice[3],
            self.slice[4],
            self.slice[5],
            self.slice[6],
            self.slice[7],
        ])
    }

    pub fn measurement_id(&self) -> u16 {
        u16::from_le_bytes([self.slice[8], self.slice[9]])
    }

    pub fn status(&self) -> bool {
        self.slice[10] & 1 != 0
    }

    pub fn row(&self, row: usize) -> Result<DataBlock, Error> {
        let offset = ColumnHeader::LEN + row * DataBlock::LEN;

        if self.slice.len() < offset + DataBlock::LEN {
            let delta = offset + DataBlock::LEN - self.slice.len();
            return Err(Error::UnexpectedEnd(delta));
        }

        Ok(DataBlock {
            range: u16::from_le_bytes([self.slice[offset], self.slice[offset + 1] & 0x7F]),
            reflect: self.slice[offset + 2],
            nir: self.slice[offset + 3],
        })
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct DataBlock {
    /// Range in millimeters, discretized to the nearest 1 millimeters.
    pub range: u16,
    /// Sensor Signal Photons measurements are scaled based on measured range
    /// and sensor sensitivity at that range, providing an indication of target
    /// reflectivity.
    pub reflect: u8,
    /// NIR photons related to natural environmental illumination are reported.
    pub nir: u8,
}

impl DataBlock {
    /// Length of the data block in bytes/octets.
    pub const LEN: usize = 4;
}

pub struct FrameReader {
    pub rows: usize,
    pub cols: usize,
    columns_per_packet: usize,
    frame_id: u16,
    /// Depth buffer owned by FrameReader - never copied
    depth: Array2<u16>,
    /// Reflectivity buffer owned by FrameReader - never copied
    reflect: Array2<u8>,
    timestamp: u64,
    /// True when a complete frame is ready for processing
    frame_ready: bool,
}

impl FrameReader {
    pub fn new(lidar_data_format: &LidarDataFormat) -> Result<FrameReader, Error> {
        if lidar_data_format.udp_profile_lidar != "RNG15_RFL8_NIR8" {
            return Err(Error::UnsupportedFormat(
                lidar_data_format.udp_profile_lidar.clone(),
            ));
        }

        let cols = lidar_data_format.columns_per_frame;
        let rows = lidar_data_format.pixels_per_column;
        let columns_per_packet = lidar_data_format.columns_per_packet;

        Ok(FrameReader {
            rows,
            cols,
            columns_per_packet,
            frame_id: 0,
            depth: Array2::zeros((rows, cols)),
            reflect: Array2::zeros((rows, cols)),
            timestamp: 0,
            frame_ready: false,
        })
    }

    /// Process a UDP packet, accumulating data into internal buffers.
    ///
    /// Returns `Ok(true)` when a frame boundary is detected (new frame_id),
    /// meaning the previous frame's data is complete and ready for processing.
    ///
    /// # Zero-Copy Design
    ///
    /// The depth and reflect buffers are owned by FrameReader and never copied.
    /// After this returns `true`, call `depth()` and `reflect()` to get
    /// references to the completed frame data, then call `start_new_frame()`
    /// to prepare for the next frame.
    pub fn update(&mut self, slice: &[u8]) -> Result<bool, Error> {
        let header = HeaderSlice::from_slice(slice)?;

        // Detect frame boundary
        let is_new_frame = self.frame_id != header.frame_id();
        if is_new_frame {
            // Previous frame is complete - mark ready and update frame_id
            self.frame_ready = true;
            self.frame_id = header.frame_id();
        }

        // If we just detected a boundary, don't process this packet yet
        // The caller should first consume the completed frame
        if self.frame_ready {
            return Ok(true);
        }

        // Process packet columns into our buffers
        for i in 0..self.columns_per_packet {
            let column = header.column(self.rows, i)?;
            if column.status() {
                let col = column.measurement_id() as usize;
                if col < self.cols {
                    for row in 0..self.rows {
                        let data = column.row(row)?;
                        self.depth[[row, col]] = data.range;
                        self.reflect[[row, col]] = data.reflect;
                    }
                }
            }

            if i == 0 {
                self.timestamp = timestamp()?;
            }
        }

        Ok(false)
    }

    /// Start a new frame after the previous one has been consumed.
    ///
    /// This clears the buffers and resets the frame_ready flag.
    pub fn start_new_frame(&mut self) {
        self.depth.fill(0);
        self.reflect.fill(0);
        self.frame_ready = false;
    }

    /// Get the timestamp of the current/completed frame.
    pub fn timestamp(&self) -> u64 {
        self.timestamp
    }

    /// Get the frame_id of the current/completed frame.
    pub fn frame_id(&self) -> u16 {
        self.frame_id
    }

    /// Get reference to the depth buffer (zero-copy access).
    pub fn depth(&self) -> &Array2<u16> {
        &self.depth
    }

    /// Get reference to the reflect buffer (zero-copy access).
    pub fn reflect(&self) -> &Array2<u8> {
        &self.reflect
    }
}

/// Point cloud coordinate calculator with pre-computed lookup tables.
///
/// Uses pre-computed coefficients in linear output order for efficient
/// SIMD processing. All lookup tables are computed once at construction.
pub struct FrameBuilder {
    pub rows: usize,
    pub cols: usize,
    pub n_points: usize,
    pub crop: (usize, usize),
    /// Scratch buffer for range values (internal units before conversion)
    pub range: Vec<f32>,
    /// Range delta for coordinate calculation
    range_delta: f32,

    // Pre-computed coefficients in linear output order for direct indexing.
    /// Source indices: linear index into packet depth/reflect arrays
    src_indices: Vec<usize>,
    /// Pre-flattened x_range coefficients (for r * x_coeff calculation)
    x_coeff: Vec<f32>,
    /// Pre-flattened y_range coefficients (for r * y_coeff calculation)
    y_coeff: Vec<f32>,
    /// Pre-flattened x_delta offsets
    x_offset: Vec<f32>,
    /// Pre-flattened y_delta offsets
    y_offset: Vec<f32>,
    /// Pre-flattened altitude coefficients (sin of altitude angle)
    z_coeff: Vec<f32>,
    /// beam_to_lidar Z offset (cached for SIMD broadcast)
    beam_z: f32,
}

impl FrameBuilder {
    pub fn new(params: &Parameters) -> Self {
        let rows = params.lidar_data_format.pixels_per_column;
        let cols = params.lidar_data_format.columns_per_frame;
        let pixel_shift_by_row = params.lidar_data_format.pixel_shift_by_row.clone();
        let column_window = params.lidar_data_format.column_window;
        let beam_to_lidar = Array2::from_shape_vec(
            (4, 4),
            params.beam_intrinsics.beam_to_lidar_transform.clone(),
        )
        .unwrap();
        let range_delta = (beam_to_lidar[[0, 3]].powi(2) + beam_to_lidar[[2, 3]].powi(2)).sqrt();

        // The left and right columns are incomplete within the pixel shift
        // region.  We crop out this region to return the clean subset.
        let crop = {
            let mut pixel_shift = pixel_shift_by_row.clone();
            pixel_shift.sort_unstable();
            let left = pixel_shift[pixel_shift.len() - 1];
            let right = pixel_shift[0].abs();
            let right = (column_window[1] - column_window[0]) as i16 - right;
            (left as usize, right as usize)
        };

        let enc = (0..cols)
            .map(|col| 2.0 * PI * (1.0 - col as f32 / cols as f32))
            .collect::<Vec<_>>();

        let azimuth_angles: Vec<_> = params
            .beam_intrinsics
            .beam_azimuth_angles
            .iter()
            .map(|x| -2.0 * PI * (x / 360.0))
            .collect();

        let altitude_angles: Vec<_> = params
            .beam_intrinsics
            .beam_altitude_angles
            .iter()
            .map(|x| 2.0 * PI * (x / 360.0))
            .collect();

        let mut x_range = Array2::zeros((rows, cols));
        let mut y_range = Array2::zeros((rows, cols));

        for row in 0..rows {
            for col in 0..cols {
                x_range[[row, col]] =
                    (enc[col] + azimuth_angles[row]).cos() * altitude_angles[row].cos();
                y_range[[row, col]] =
                    (enc[col] + azimuth_angles[row]).sin() * altitude_angles[row].cos();
            }
        }

        let x_delta: Vec<f32> = enc
            .iter()
            .map(|x| beam_to_lidar[[0, 3]] * x.cos())
            .collect();
        let y_delta: Vec<f32> = enc
            .iter()
            .map(|x| beam_to_lidar[[0, 3]] * x.sin())
            .collect();

        // Pre-compute altitude sin values
        let altitude_sin: Vec<f32> = altitude_angles.iter().map(|x| x.sin()).collect();

        // Pre-compute coefficients in linear output order.
        let output_cols = crop.1 - crop.0;
        let n_output_points = rows * output_cols;
        let mut src_indices = Vec::with_capacity(n_output_points);
        let mut x_coeff = Vec::with_capacity(n_output_points);
        let mut y_coeff = Vec::with_capacity(n_output_points);
        let mut x_offset = Vec::with_capacity(n_output_points);
        let mut y_offset = Vec::with_capacity(n_output_points);
        let mut z_coeff = Vec::with_capacity(n_output_points);

        let col_base = column_window[0] as i16;
        let packet_cols = column_window[1] - column_window[0];

        for row in 0..rows {
            let col_offset = (col_base - pixel_shift_by_row[row]).max(0) as usize;

            for col in crop.0..crop.1 {
                // Source index: reverse pixel shift to find packet data location
                let shift = pixel_shift_by_row[row];
                let src_col = ((col as i16 - shift).clamp(0, packet_cols as i16 - 1)) as usize
                    + column_window[0];
                src_indices.push(row * cols + src_col);

                // Lookup table index with column offset adjustment
                let lookup_col = col + col_offset;
                x_coeff.push(x_range[[row, lookup_col]]);
                y_coeff.push(y_range[[row, lookup_col]]);
                x_offset.push(x_delta[lookup_col]);
                y_offset.push(y_delta[lookup_col]);
                z_coeff.push(altitude_sin[row]);
            }
        }

        let beam_z = beam_to_lidar[[2, 3]];

        FrameBuilder {
            rows,
            cols,
            n_points: 0,
            crop,
            range: vec![0f32; n_output_points],
            range_delta,
            src_indices,
            x_coeff,
            y_coeff,
            x_offset,
            y_offset,
            z_coeff,
            beam_z,
        }
    }

    /// Process depth and reflectivity data into XYZ point cloud.
    ///
    /// Reads directly from packet arrays, applies range scaling, computes
    /// Cartesian coordinates using pre-computed lookup tables, and writes
    /// directly to the output frame buffers (true zero-copy).
    #[instrument(skip_all, fields(rows = self.rows, cols = self.cols))]
    pub fn update_fused<F: LidarFrameWriter>(
        &mut self,
        depth: &Array2<u16>,
        reflect: &Array2<u8>,
        frame: &mut F,
    ) {
        let n_points = self.src_indices.len();
        let depth_raw = depth.as_slice().expect("depth array not contiguous");
        let reflect_raw = reflect.as_slice().expect("reflect array not contiguous");

        // Get all mutable slices at once (avoids borrow checker issues)
        let (frame_x, frame_y, frame_z, frame_intensity, frame_range) = frame.buffers_mut();

        // Convert depth to range and extract intensity directly into frame
        // Using indexed loop for clarity with multiple source arrays
        #[allow(clippy::needless_range_loop)]
        for i in 0..n_points {
            let src_idx = self.src_indices[i];
            let d = depth_raw[src_idx];
            // Range calculation: 0 stays 0, otherwise apply scale and offset
            self.range[i] = if d == 0 {
                0.0
            } else {
                d as f32 * 8.0 - self.range_delta
            };
            frame_intensity[i] = reflect_raw[src_idx];
        }

        // Calculate XYZ coordinates directly into frame buffers (zero-copy)
        self.calculate_points_fused_into(n_points, frame_x, frame_y, frame_z);
        self.n_points = n_points;

        // Copy range to frame (convert to meters)
        for (i, frame_r) in frame_range.iter_mut().enumerate().take(n_points) {
            *frame_r = self.range[i] * 0.001;
        }

        // Set the valid length
        frame.set_len(n_points);
    }

    /// Calculate XYZ coordinates using NEON SIMD, writing directly to provided
    /// output slices (true zero-copy).
    #[cfg(target_arch = "aarch64")]
    #[inline(never)]
    fn calculate_points_fused_into(
        &self,
        len: usize,
        out_x: &mut [f32],
        out_y: &mut [f32],
        out_z: &mut [f32],
    ) {
        const LANES: usize = 4;
        let n_simd = len - len % LANES;

        // SAFETY: NEON intrinsics are always available on aarch64.
        unsafe {
            let beam_z = vdupq_n_f32(self.beam_z);
            let scale = vdupq_n_f32(0.001);

            let range_ptr = self.range.as_ptr();
            let x_coeff_ptr = self.x_coeff.as_ptr();
            let x_offset_ptr = self.x_offset.as_ptr();
            let y_coeff_ptr = self.y_coeff.as_ptr();
            let y_offset_ptr = self.y_offset.as_ptr();
            let z_coeff_ptr = self.z_coeff.as_ptr();
            let x_out_ptr = out_x.as_mut_ptr();
            let y_out_ptr = out_y.as_mut_ptr();
            let z_out_ptr = out_z.as_mut_ptr();

            for i in (0..n_simd).step_by(LANES) {
                let r = vld1q_f32(range_ptr.add(i));

                // X = (r * x_coeff + x_offset) * scale
                let xc = vld1q_f32(x_coeff_ptr.add(i));
                let xo = vld1q_f32(x_offset_ptr.add(i));
                let x = vmulq_f32(vfmaq_f32(xo, r, xc), scale);
                vst1q_f32(x_out_ptr.add(i), x);

                // Y = (r * y_coeff + y_offset) * scale
                let yc = vld1q_f32(y_coeff_ptr.add(i));
                let yo = vld1q_f32(y_offset_ptr.add(i));
                let y = vmulq_f32(vfmaq_f32(yo, r, yc), scale);
                vst1q_f32(y_out_ptr.add(i), y);

                // Z = (r * z_coeff + beam_z) * scale
                let zc = vld1q_f32(z_coeff_ptr.add(i));
                let z = vmulq_f32(vfmaq_f32(beam_z, r, zc), scale);
                vst1q_f32(z_out_ptr.add(i), z);
            }
        }

        // Scalar remainder
        for i in n_simd..len {
            let r = self.range[i];
            out_x[i] = (r * self.x_coeff[i] + self.x_offset[i]) * 0.001;
            out_y[i] = (r * self.y_coeff[i] + self.y_offset[i]) * 0.001;
            out_z[i] = (r * self.z_coeff[i] + self.beam_z) * 0.001;
        }
    }

    /// Calculate XYZ coordinates using portable_simd, writing directly to
    /// provided output slices (true zero-copy).
    #[cfg(all(feature = "portable_simd", not(target_arch = "aarch64")))]
    #[inline(never)]
    fn calculate_points_fused_into(
        &self,
        len: usize,
        out_x: &mut [f32],
        out_y: &mut [f32],
        out_z: &mut [f32],
    ) {
        const LANES: usize = 4;
        let n_simd = len - len % LANES;

        let beam_z = Simd::<f32, LANES>::splat(self.beam_z);
        let scale = Simd::<f32, LANES>::splat(0.001);

        for i in (0..n_simd).step_by(LANES) {
            let r = Simd::<f32, LANES>::from_slice(&self.range[i..]);

            let xc = Simd::<f32, LANES>::from_slice(&self.x_coeff[i..]);
            let xo = Simd::<f32, LANES>::from_slice(&self.x_offset[i..]);
            let x = (r * xc + xo) * scale;
            x.copy_to_slice(&mut out_x[i..]);

            let yc = Simd::<f32, LANES>::from_slice(&self.y_coeff[i..]);
            let yo = Simd::<f32, LANES>::from_slice(&self.y_offset[i..]);
            let y = (r * yc + yo) * scale;
            y.copy_to_slice(&mut out_y[i..]);

            let zc = Simd::<f32, LANES>::from_slice(&self.z_coeff[i..]);
            let z = (r * zc + beam_z) * scale;
            z.copy_to_slice(&mut out_z[i..]);
        }

        // Scalar remainder
        for i in n_simd..len {
            let r = self.range[i];
            out_x[i] = (r * self.x_coeff[i] + self.x_offset[i]) * 0.001;
            out_y[i] = (r * self.y_coeff[i] + self.y_offset[i]) * 0.001;
            out_z[i] = (r * self.z_coeff[i] + self.beam_z) * 0.001;
        }
    }

    /// Calculate XYZ coordinates using scalar code, writing directly to
    /// provided output slices (true zero-copy).
    #[cfg(all(not(feature = "portable_simd"), not(target_arch = "aarch64")))]
    #[inline(never)]
    fn calculate_points_fused_into(
        &self,
        len: usize,
        out_x: &mut [f32],
        out_y: &mut [f32],
        out_z: &mut [f32],
    ) {
        for i in 0..len {
            let r = self.range[i];
            out_x[i] = (r * self.x_coeff[i] + self.x_offset[i]) * 0.001;
            out_y[i] = (r * self.y_coeff[i] + self.y_offset[i]) * 0.001;
            out_z[i] = (r * self.z_coeff[i] + self.beam_z) * 0.001;
        }
    }
}

/// Ouster-specific LidarFrame implementation.
///
/// Stores point cloud data in Structure-of-Arrays layout for efficient SIMD
/// processing. The client owns this frame and provides it to the driver.
pub struct OusterLidarFrame {
    x: Vec<f32>,
    y: Vec<f32>,
    z: Vec<f32>,
    intensity: Vec<u8>,
    range: Vec<f32>,
    len: usize,
    timestamp: u64,
    frame_id: u32,
}

impl OusterLidarFrame {
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

    /// Create a new frame with default capacity for Ouster sensors.
    ///
    /// Default is 128 rows × 1024 columns = 131,072 points.
    pub fn new() -> Self {
        Self::with_capacity(128 * 1024)
    }
}

impl Default for OusterLidarFrame {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: OusterLidarFrame contains only primitive types in Vecs,
// which are inherently Send.
unsafe impl Send for OusterLidarFrame {}

impl LidarFrame for OusterLidarFrame {
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

impl LidarFrameWriter for OusterLidarFrame {
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

/// Maximum UDP packet size for Ouster sensors (OS1-128 at 1024 mode)
const MAX_PACKET_SIZE: usize = 16 * 1024;

/// Ouster LiDAR driver implementing the common LidarDriver trait
///
/// This driver wraps the FrameReader and FrameBuilder to provide
/// a unified interface for processing Ouster UDP packets.
///
/// # Zero-Copy Architecture
///
/// The driver uses a true zero-copy design:
/// - FrameReader owns internal depth/reflect buffers (never copied)
/// - FrameBuilder writes directly to client-provided frame buffers
/// - No intermediate copies or allocations during processing
/// - Pre-allocated packet buffer handles frame boundary packets
pub struct OusterDriver {
    /// Frame reader for packet parsing (owns depth/reflect buffers)
    frame_reader: FrameReader,
    /// Frame builder for point cloud computation
    frame_builder: FrameBuilder,
    /// Pre-allocated buffer for packet that triggers frame boundary
    pending_packet_buf: Vec<u8>,
    /// Length of valid data in pending_packet_buf (0 = no pending packet)
    pending_packet_len: usize,
}

impl OusterDriver {
    /// Create a new Ouster driver from sensor parameters
    pub fn new(params: &Parameters) -> Result<Self, Error> {
        let frame_reader = FrameReader::new(&params.lidar_data_format)?;
        let frame_builder = FrameBuilder::new(params);

        Ok(Self {
            frame_reader,
            frame_builder,
            pending_packet_buf: vec![0u8; MAX_PACKET_SIZE],
            pending_packet_len: 0,
        })
    }

    /// Get the number of rows in the frame
    pub fn rows(&self) -> usize {
        self.frame_builder.rows
    }

    /// Get the number of columns in the frame
    pub fn cols(&self) -> usize {
        self.frame_builder.cols
    }

    /// Get the crop boundaries
    pub fn crop(&self) -> (usize, usize) {
        self.frame_builder.crop
    }
}

impl LidarDriver for OusterDriver {
    fn process<F: LidarFrameWriter>(&mut self, frame: &mut F, data: &[u8]) -> Result<bool, Error> {
        // Process pending packet from previous frame boundary first
        if self.pending_packet_len > 0 {
            let pending = &self.pending_packet_buf[..self.pending_packet_len];
            self.frame_reader.update(pending)?;
            self.pending_packet_len = 0;
        }

        // Process current packet
        let frame_ready = self.frame_reader.update(data)?;

        if frame_ready {
            // Store this packet in pre-allocated buffer (no allocation)
            let len = data.len().min(MAX_PACKET_SIZE);
            self.pending_packet_buf[..len].copy_from_slice(&data[..len]);
            self.pending_packet_len = len;

            // Get zero-copy references to FrameReader's buffers
            let timestamp = self.frame_reader.timestamp();
            let frame_id = self.frame_reader.frame_id();
            let depth = self.frame_reader.depth();
            let reflect = self.frame_reader.reflect();

            // Reset frame for new data
            frame.reset();
            frame.set_timestamp(timestamp);
            frame.set_frame_id(frame_id as u32);

            // Build point cloud directly into frame buffers (true zero-copy)
            self.frame_builder.update_fused(depth, reflect, frame);

            // Prepare FrameReader for next frame
            self.frame_reader.start_new_frame();

            Ok(true)
        } else {
            Ok(false)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ouster_frame_basic() {
        let mut frame = OusterLidarFrame::with_capacity(100);
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
    fn test_ouster_frame_capacity() {
        let mut frame = OusterLidarFrame::with_capacity(2);
        assert!(frame.push(1.0, 1.0, 1.0, 1, 1.0));
        assert!(frame.push(2.0, 2.0, 2.0, 2, 2.0));
        assert!(!frame.push(3.0, 3.0, 3.0, 3, 3.0)); // At capacity
        assert_eq!(frame.len(), 2);
    }

    #[test]
    fn test_ouster_frame_metadata() {
        let mut frame = OusterLidarFrame::new();
        frame.set_timestamp(12345678);
        frame.set_frame_id(42);
        assert_eq!(frame.timestamp(), 12345678);
        assert_eq!(frame.frame_id(), 42);
    }

    #[test]
    fn test_ouster_frame_default_capacity() {
        let frame = OusterLidarFrame::new();
        // Default is 128 × 1024 = 131,072
        assert_eq!(frame.capacity(), 128 * 1024);
    }

    #[test]
    fn test_shot_limiting_display() {
        assert_eq!(format!("{}", ShotLimiting::Normal), "Normal");
        assert_eq!(
            format!("{}", ShotLimiting::Imminent(5)),
            "Limiting in 5 seconds"
        );
        assert_eq!(
            format!("{}", ShotLimiting::Limiting(50)),
            "Limiting to approximately 50% range"
        );
    }

    #[test]
    fn test_shutdown_status_display() {
        assert_eq!(format!("{}", ShutdownStatus::Normal), "Normal");
        assert_eq!(
            format!("{}", ShutdownStatus::Imminent(10)),
            "Shutdown imminent in 10 seconds"
        );
    }

    #[test]
    fn test_alert_status_from_flags() {
        assert_eq!(AlertStatus::from_flags(0x00), AlertStatus::Normal);
        assert_eq!(AlertStatus::from_flags(0x80), AlertStatus::Active(0));
        assert_eq!(AlertStatus::from_flags(0x85), AlertStatus::Active(5));
        assert_eq!(AlertStatus::from_flags(0x45), AlertStatus::Overflow(5));
    }
}
