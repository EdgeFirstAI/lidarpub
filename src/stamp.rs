// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Acquisition stamps for LiDAR frames.
//!
//! A frame is stamped with the sensor's own start-of-frame time when the
//! sensor reports its clock synchronized to the host (PTP with the host or
//! another system as grandmaster), and otherwise with the host receive time
//! of the frame's first packet minus a configured sensor latency.
//!
//! A synchronized sensor time is only accepted when it is plausible against
//! the host receive time of the packet that carried it. This rejects sensor
//! clocks that report synchronization but are not in the host domain, such
//! as a clock that is still slewing towards its grandmaster after a step.

use std::{
    fmt,
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tracing::{info, warn};

/// Largest accepted delay from a sensor timestamp to the host receive time
/// of the packet carrying it. Measured delays are about 16 ms for the
/// Robosense E1R start-of-sweep packet and about 3 ms for Ouster packets.
pub const MAX_SENSOR_LATENCY: Duration = Duration::from_millis(100);

/// Largest accepted lead of a sensor timestamp over the host receive time of
/// the packet carrying it, allowing for residual PTP offset.
pub const MAX_SENSOR_LEAD: Duration = Duration::from_millis(5);

/// Clock that a frame stamp was taken from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StampSource {
    /// The sensor clock, synchronized to the host by PTP.
    SensorClock,
    /// The host receive time of the frame's first packet, less the
    /// configured sensor latency.
    HostReceive,
}

impl fmt::Display for StampSource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StampSource::SensorClock => write!(f, "sensor clock (PTP)"),
            StampSource::HostReceive => write!(f, "host receive time"),
        }
    }
}

/// Chooses the stamp of each frame and logs when the choice changes.
#[derive(Debug)]
pub struct FrameStamper {
    sensor: &'static str,
    host_latency_ns: u64,
    source: Option<StampSource>,
    rejecting: bool,
}

impl FrameStamper {
    /// Creates a stamper for the named sensor with no host latency.
    pub fn new(sensor: &'static str) -> Self {
        Self {
            sensor,
            host_latency_ns: 0,
            source: None,
            rejecting: false,
        }
    }

    /// Sets the sensor latency subtracted from host receive times.
    pub fn set_host_latency(&mut self, latency: Duration) {
        self.host_latency_ns = u64::try_from(latency.as_nanos()).unwrap_or(u64::MAX);
    }

    /// Source of the most recent stamp, `None` before the first frame.
    #[allow(dead_code)] // Used by library consumers and tests
    pub fn source(&self) -> Option<StampSource> {
        self.source
    }

    /// Returns the stamp of a frame in nanoseconds since the Unix epoch.
    ///
    /// `rx_ns` is the host receive time of the frame's first packet.
    /// `sensor_ns` is the sensor's start-of-frame time, given only when the
    /// sensor reports its clock synchronized to the host.
    pub fn stamp(&mut self, rx_ns: u64, sensor_ns: Option<u64>) -> u64 {
        let accepted = sensor_ns.filter(|&sensor_ns| plausible(rx_ns, sensor_ns));

        match (sensor_ns, accepted) {
            (Some(sensor_ns), None) if !self.rejecting => {
                warn!(
                    sensor = self.sensor,
                    receive_minus_sensor_ms = (rx_ns as i128 - sensor_ns as i128) as f64 / 1e6,
                    "sensor reports a synchronized clock that disagrees with the host receive time"
                );
                self.rejecting = true;
            }
            (Some(_), None) => {}
            _ => self.rejecting = false,
        }

        let (stamp, source) = match accepted {
            Some(sensor_ns) => (sensor_ns, StampSource::SensorClock),
            None => (
                rx_ns.saturating_sub(self.host_latency_ns),
                StampSource::HostReceive,
            ),
        };

        if self.source != Some(source) {
            info!(
                sensor = self.sensor,
                source = %source,
                host_latency_ms = self.host_latency_ns as f64 / 1e6,
                "frame stamp source"
            );
            self.source = Some(source);
        }

        stamp
    }
}

/// Whether a sensor timestamp is consistent with the host receive time of
/// the packet that carried it.
pub fn plausible(rx_ns: u64, sensor_ns: u64) -> bool {
    let delay = rx_ns as i128 - sensor_ns as i128;
    delay >= -(MAX_SENSOR_LEAD.as_nanos() as i128) && delay <= MAX_SENSOR_LATENCY.as_nanos() as i128
}

/// Nanoseconds since the Unix epoch, zero for instants before it.
pub fn system_time_ns(time: SystemTime) -> u64 {
    time.duration_since(UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_nanos()).unwrap_or(u64::MAX))
}

#[cfg(test)]
mod tests {
    use super::*;

    const RX: u64 = 1_790_000_000_000_000_000;
    const MS: u64 = 1_000_000;

    #[test]
    fn unsynchronized_sensor_uses_host_receive_time() {
        let mut stamper = FrameStamper::new("test");
        assert_eq!(stamper.stamp(RX, None), RX);
        assert_eq!(stamper.source(), Some(StampSource::HostReceive));
    }

    #[test]
    fn host_latency_is_subtracted_from_receive_time_only() {
        let mut stamper = FrameStamper::new("test");
        stamper.set_host_latency(Duration::from_millis(15));
        assert_eq!(stamper.stamp(RX, None), RX - 15 * MS);
        assert_eq!(stamper.stamp(RX, Some(RX - 16 * MS)), RX - 16 * MS);
        assert_eq!(stamper.source(), Some(StampSource::SensorClock));
    }

    #[test]
    fn synchronized_sensor_within_window_is_used() {
        let mut stamper = FrameStamper::new("test");
        for sensor in [RX - 99 * MS, RX - 16 * MS, RX, RX + 4 * MS] {
            assert_eq!(stamper.stamp(RX, Some(sensor)), sensor);
            assert_eq!(stamper.source(), Some(StampSource::SensorClock));
        }
    }

    #[test]
    fn implausible_sensor_time_falls_back_to_host() {
        let mut stamper = FrameStamper::new("test");
        // Too old: a sensor clock still slewing after a host step, or
        // drifting in holdover.
        assert_eq!(stamper.stamp(RX, Some(RX - 3_000 * MS)), RX);
        assert_eq!(stamper.source(), Some(StampSource::HostReceive));
        // Ahead of the host: a TAI clock read as UTC.
        assert_eq!(stamper.stamp(RX, Some(RX + 37_000 * MS)), RX);
        // Recovers once the sensor clock converges.
        assert_eq!(stamper.stamp(RX, Some(RX - 3 * MS)), RX - 3 * MS);
        assert_eq!(stamper.source(), Some(StampSource::SensorClock));
    }

    #[test]
    fn host_latency_saturates_at_epoch() {
        let mut stamper = FrameStamper::new("test");
        stamper.set_host_latency(Duration::from_secs(10));
        assert_eq!(stamper.stamp(5, None), 0);
    }

    #[test]
    fn plausibility_window_bounds() {
        let latency = MAX_SENSOR_LATENCY.as_nanos() as u64;
        let lead = MAX_SENSOR_LEAD.as_nanos() as u64;
        assert!(plausible(RX, RX - latency));
        assert!(!plausible(RX, RX - latency - 1));
        assert!(plausible(RX, RX + lead));
        assert!(!plausible(RX, RX + lead + 1));
    }

    #[test]
    fn system_time_ns_clamps_pre_epoch() {
        assert_eq!(system_time_ns(UNIX_EPOCH), 0);
        assert_eq!(system_time_ns(UNIX_EPOCH + Duration::from_nanos(RX)), RX);
        assert_eq!(system_time_ns(UNIX_EPOCH - Duration::from_secs(1)), 0);
    }
}
