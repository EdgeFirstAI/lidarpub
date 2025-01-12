use std::fmt;

#[derive(Debug)]
pub enum Error {
    IoError(std::io::Error),
    UnexpectedEndOfSlice(usize),
    UnknownPacketType(u16),
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(err: std::io::Error) -> Error {
        Error::IoError(err)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        match self {
            Error::IoError(err) => write!(f, "io error: {}", err),
            Error::UnexpectedEndOfSlice(len) => write!(f, "unexpected end of slice: {} bytes", len),
            Error::UnknownPacketType(typ) => write!(f, "unknown packet type: {}", typ),
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
        if flags & 2 != 0 {
            AlertStatus::Overflow(flags >> 6)
        } else if flags & 1 != 0 {
            AlertStatus::Active(flags >> 6)
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
        u32::from_le_bytes([0, self.slice[4], self.slice[5], self.slice[6]])
    }

    pub fn serial_number(&self) -> u64 {
        u64::from_le_bytes([
            0,
            0,
            0,
            self.slice[7],
            self.slice[8],
            self.slice[9],
            self.slice[10],
            self.slice[11],
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
        // println!("flags: {:b}", self.slice[12]);
        AlertStatus::from_flags(self.slice[12])
    }
}
