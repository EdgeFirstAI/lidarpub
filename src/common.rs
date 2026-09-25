// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use clap::ValueEnum;
use edgefirst_schemas::builtin_interfaces::Time;
use std::{fmt, time::Duration};
use zenoh::{
    Session,
    time::{NTP64, Timestamp, TimestampId},
};

#[cfg(target_os = "linux")]
use log::warn;

#[derive(Copy, Clone, Debug, PartialEq, ValueEnum)]
pub enum TimestampMode {
    Internal,
    SyncPulse,
    Ptp1588,
}

impl fmt::Display for TimestampMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TimestampMode::Internal => write!(f, "TIME_FROM_INTERNAL_OSC"),
            TimestampMode::SyncPulse => write!(f, "TIME_FROM_SYNC_PULSE_IN"),
            TimestampMode::Ptp1588 => write!(f, "TIME_FROM_PTP_1588"),
        }
    }
}

impl TryFrom<&str> for TimestampMode {
    type Error = String;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        match value {
            "TIME_FROM_INTERNAL_OSC" => Ok(TimestampMode::Internal),
            "TIME_FROM_SYNC_PULSE_IN" => Ok(TimestampMode::SyncPulse),
            "TIME_FROM_PTP_1588" => Ok(TimestampMode::Ptp1588),
            _ => Err(format!("Invalid timestamp mode: {}", value)),
        }
    }
}

#[cfg(target_os = "linux")]
pub fn set_process_priority() {
    let mut param = libc::sched_param { sched_priority: 10 };
    let pid = unsafe { libc::pthread_self() };
    let err = unsafe {
        libc::pthread_setschedparam(pid, libc::SCHED_FIFO, &mut param as *mut libc::sched_param)
    };
    if err != 0 {
        let err = std::io::Error::last_os_error();
        warn!("unable to set udp_read real-time fifo scheduler: {}", err);
    }
}

#[cfg(not(target_os = "linux"))]
pub fn set_process_priority() {}

/// Returns the Zenoh timestamp source ID of the session.
pub fn timestamp_id(session: &Session) -> TimestampId {
    *session.new_timestamp().get_id()
}

/// Builds the Zenoh sample timestamp for a message stamp so the sample
/// timestamp and `header.stamp` denote the same instant. NTP64 quantizes the
/// fraction to 2^-32 s (about 0.23 ns), so the round trip is exact only to
/// within a nanosecond.
pub fn zenoh_timestamp(id: TimestampId, stamp: &Time) -> Timestamp {
    let time = Duration::new(stamp.sec.max(0) as u64, stamp.nanosec);
    Timestamp::new(NTP64::from(time), id)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip(stamp: Time) -> Duration {
        zenoh_timestamp(TimestampId::rand(), &stamp)
            .get_time()
            .to_duration()
    }

    #[test]
    fn zenoh_timestamp_matches_stamp() {
        for nanosec in [0, 1, 123_456_789, 500_000_000, 999_999_999] {
            let stamp = Time {
                sec: 1_790_000_000,
                nanosec,
            };
            let expected = Duration::new(stamp.sec as u64, stamp.nanosec);
            let diff = round_trip(stamp).abs_diff(expected);
            assert!(diff <= Duration::from_nanos(1), "{nanosec}: {diff:?}");
        }
    }

    #[test]
    fn zenoh_timestamp_clamps_negative_seconds() {
        let stamp = Time {
            sec: -5,
            nanosec: 7,
        };
        assert!(round_trip(stamp) <= Duration::from_nanos(8));
    }
}
