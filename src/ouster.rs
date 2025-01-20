use ndarray::Array2;
use serde::{Deserialize, Serialize};
use std::{f32::consts::PI, fmt};

#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    pub udp_dest: String,
    pub udp_port_lidar: u16,
    pub udp_profile_lidar: String,
    pub lidar_mode: String,
    pub azimuth_window: [u32; 2],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            udp_dest: "".to_owned(),
            udp_port_lidar: 7502,
            udp_profile_lidar: "RNG15_RFL8_NIR8".to_owned(),
            lidar_mode: "1024x10".to_owned(),
            azimuth_window: [0, 360000],
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

#[derive(Debug)]
pub enum Error {
    IoError(std::io::Error),
    ShapeError(ndarray::ShapeError),
    UnsupportedDataFormat(String),
    UnexpectedEndOfSlice(usize),
    UnknownPacketType(u16),
    TooManyColumns(usize),
    InsufficientColumns(usize),
    UnsupportedRows(usize),
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(err: std::io::Error) -> Error {
        Error::IoError(err)
    }
}

impl From<ndarray::ShapeError> for Error {
    fn from(err: ndarray::ShapeError) -> Error {
        Error::ShapeError(err)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        match self {
            Error::IoError(err) => write!(f, "io error: {}", err),
            Error::ShapeError(err) => write!(f, "shape error: {}", err),
            Error::UnsupportedDataFormat(format) => {
                write!(f, "unsupported lidar data format: {}", format)
            }
            Error::UnexpectedEndOfSlice(len) => write!(f, "unexpected end of slice: {} bytes", len),
            Error::UnknownPacketType(typ) => write!(f, "unknown packet type: {}", typ),
            Error::TooManyColumns(cols) => write!(f, "too many columns: {}", cols),
            Error::InsufficientColumns(cols) => write!(f, "insufficient columns: {}", cols),
            Error::UnsupportedRows(rows) => write!(f, "unsupported number of rows: {}", rows),
        }
    }
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
            return Err(Error::UnexpectedEndOfSlice(slice.len()));
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
            return Err(Error::UnexpectedEndOfSlice(slice.len()));
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
            return Err(Error::UnexpectedEndOfSlice(delta));
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

#[derive(Clone, Debug)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub reflect: u8,
    pub nir: u8,
}

#[derive(Clone, Debug)]
pub struct Frame {
    pub frame_id: u16,
    pub points: Vec<Point>,
}

pub struct FrameReader {
    pub rows: usize,
    pub cols: usize,
    columns_per_packet: usize,
    beam_to_lidar: Array2<f32>,
    altitude_angles: Vec<f32>,
    range_delta: f32,
    x_range: Array2<f32>,
    y_range: Array2<f32>,
    x_delta: Vec<f32>,
    y_delta: Vec<f32>,
    frame_id: u16,
    range: Array2<u16>,
    reflect: Array2<u8>,
    nir: Array2<u8>,
}

impl FrameReader {
    pub fn new(params: Parameters) -> Result<FrameReader, Error> {
        if params.lidar_data_format.udp_profile_lidar != "RNG15_RFL8_NIR8" {
            return Err(Error::UnsupportedDataFormat(
                params.lidar_data_format.udp_profile_lidar,
            ));
        }

        let cols = params.lidar_data_format.columns_per_frame;
        let rows = params.lidar_data_format.pixels_per_column;
        let columns_per_packet = params.lidar_data_format.columns_per_packet;
        let beam_to_lidar =
            Array2::from_shape_vec((4, 4), params.beam_intrinsics.beam_to_lidar_transform)?;
        let range_delta = (beam_to_lidar[[0, 3]].powi(2) + beam_to_lidar[[2, 3]].powi(2)).sqrt();

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

        let mut x_range = Array2::zeros((cols, rows));
        let mut y_range = Array2::zeros((cols, rows));

        for col in 0..cols {
            for row in 0..rows {
                x_range[[col, row]] =
                    (enc[col] + azimuth_angles[row]).cos() * altitude_angles[row].cos();
                y_range[[col, row]] =
                    (enc[col] + azimuth_angles[row]).sin() * altitude_angles[row].cos();
            }
        }

        let x_delta = enc
            .iter()
            .map(|x| beam_to_lidar[[0, 3]] * x.cos())
            .collect();
        let y_delta = enc
            .iter()
            .map(|x| beam_to_lidar[[0, 3]] * x.sin())
            .collect();

        Ok(FrameReader {
            rows,
            cols,
            columns_per_packet,
            beam_to_lidar,
            altitude_angles: altitude_angles.iter().map(|x| x.sin()).collect(),
            range_delta,
            x_range,
            y_range,
            x_delta,
            y_delta,
            frame_id: 0,
            range: Array2::zeros((cols, rows)),
            reflect: Array2::zeros((cols, rows)),
            nir: Array2::zeros((cols, rows)),
        })
    }

    pub fn update(&mut self, slice: &[u8]) -> Result<Option<Frame>, Error> {
        let mut frame = None;
        let header = HeaderSlice::from_slice(slice)?;

        if self.frame_id != header.frame_id() {
            frame = Some(Frame {
                frame_id: self.frame_id,
                points: self.points(),
            });

            self.frame_id = header.frame_id();
            self.range.fill(0);
            self.reflect.fill(0);
            self.nir.fill(0);
        }

        for i in 0..self.columns_per_packet {
            let column = header.column(self.rows, i)?;
            if column.status() {
                let col = column.measurement_id() as usize;
                if col >= self.cols {
                    return Err(Error::TooManyColumns(col));
                }

                for row in 0..self.rows {
                    let data = column.row(row)?;
                    self.range[[col, row]] = data.range;
                    self.reflect[[col, row]] = data.reflect;
                    self.nir[[col, row]] = data.nir;
                }
            }
        }

        Ok(frame)
    }

    fn points(&self) -> Vec<Point> {
        let mut points = Vec::with_capacity(self.cols * self.rows);

        for col in 0..self.cols {
            let x_delta_col = self.x_delta[col];
            let y_delta_col = self.y_delta[col];

            for row in 0..self.rows {
                if self.range[[col, row]] > 0 {
                    let r = self.range[[col, row]] as f32 * 8.0 - self.range_delta;
                    let x = r * self.x_range[[col, row]] + x_delta_col;
                    let y = r * self.y_range[[col, row]] + y_delta_col;
                    let z = r * self.altitude_angles[row] + self.beam_to_lidar[[2, 3]];

                    points.push(Point {
                        x: x * 0.001,
                        y: y * 0.001,
                        z: z * 0.001,
                        reflect: self.reflect[[col, row]],
                        nir: self.nir[[col, row]],
                    });
                }
            }
        }

        points
    }
}
