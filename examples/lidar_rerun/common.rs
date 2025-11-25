// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use clap::ValueEnum;
use std::{fmt, net::UdpSocket};

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

#[cfg(target_os = "linux")]
pub fn set_socket_bufsize(socket: UdpSocket, size: usize) -> UdpSocket {
    use std::os::fd::{FromRawFd, IntoRawFd};

    let fd = socket.into_raw_fd();
    let size = size as libc::c_int;
    let err = unsafe {
        libc::setsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_RCVBUF,
            &size as *const _ as *const libc::c_void,
            std::mem::size_of_val(&size) as libc::socklen_t,
        )
    };
    if err != 0 {
        warn!(
            "setsockopt SO_RCVBUF failed: {}",
            std::io::Error::last_os_error()
        );
    }

    unsafe { UdpSocket::from_raw_fd(fd) }
}

#[cfg(not(target_os = "linux"))]
pub fn set_socket_bufsize(socket: UdpSocket, _size: usize) -> UdpSocket {
    socket
}
