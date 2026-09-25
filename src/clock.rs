// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Detection of host clock steps.
//!
//! A timerfd on `CLOCK_REALTIME` armed with `TFD_TIMER_CANCEL_ON_SET` is
//! cancelled by the kernel the moment the clock is set discontinuously
//! (`clock_settime`, `settimeofday`, or an `adjtimex` step such as chrony's
//! `makestep`). Slews do not cancel it. The step size is measured as the
//! change in `CLOCK_REALTIME - CLOCK_MONOTONIC`, which only a step alters.

use std::{
    io, mem,
    os::fd::{AsRawFd, FromRawFd, OwnedFd},
};

/// Largest gap accepted between the two `CLOCK_REALTIME` reads that bracket
/// a `CLOCK_MONOTONIC` read; preemption inside the bracket adds an error
/// equal to the gap.
const MAX_BRACKET_NS: i128 = 10_000;

/// Brackets taken at most by [`realtime_minus_monotonic`].
const BRACKET_ATTEMPTS: usize = 5;

/// Far-future expiry for the watch timer, in seconds after arming.
const WATCH_HORIZON_SECS: libc::time_t = 10 * 365 * 24 * 3600;

/// Waits for steps of the host `CLOCK_REALTIME`.
pub struct ClockStepWatcher {
    fd: OwnedFd,
    offset_ns: i128,
}

impl ClockStepWatcher {
    /// Creates and arms the watch timer.
    pub fn new() -> io::Result<Self> {
        // SAFETY: plain system call; the returned descriptor is checked and
        // owned below.
        let fd = unsafe { libc::timerfd_create(libc::CLOCK_REALTIME, libc::TFD_CLOEXEC) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: fd is a freshly created descriptor owned by nothing else.
        let fd = unsafe { OwnedFd::from_raw_fd(fd) };
        let mut watcher = Self { fd, offset_ns: 0 };
        watcher.arm()?;
        Ok(watcher)
    }

    /// Arms the timer, then records the clock offset: a step before the
    /// timer is armed shows in the recorded offset, and a step after it
    /// cancels the timer, so none is missed.
    fn arm(&mut self) -> io::Result<()> {
        let now = clock_ns(libc::CLOCK_REALTIME);
        let spec = libc::itimerspec {
            it_interval: libc::timespec {
                tv_sec: 0,
                tv_nsec: 0,
            },
            it_value: libc::timespec {
                tv_sec: (now / 1_000_000_000) as libc::time_t + WATCH_HORIZON_SECS,
                tv_nsec: 0,
            },
        };
        // SAFETY: fd is a valid timerfd and spec is a valid itimerspec.
        let err = unsafe {
            libc::timerfd_settime(
                self.fd.as_raw_fd(),
                libc::TFD_TIMER_ABSTIME | libc::TFD_TIMER_CANCEL_ON_SET,
                &spec,
                std::ptr::null_mut(),
            )
        };
        if err != 0 {
            return Err(io::Error::last_os_error());
        }
        self.offset_ns = realtime_minus_monotonic();
        Ok(())
    }

    /// Blocks until `CLOCK_REALTIME` is set discontinuously and returns the
    /// size of the step in nanoseconds (positive when the clock moved
    /// forward). A set that leaves the clock where it was returns a step
    /// near zero.
    pub fn wait(&mut self) -> io::Result<i128> {
        loop {
            let mut expirations = 0u64;
            // SAFETY: reads at most 8 bytes into a u64 owned by this frame.
            let n = unsafe {
                libc::read(
                    self.fd.as_raw_fd(),
                    &mut expirations as *mut u64 as *mut libc::c_void,
                    mem::size_of::<u64>(),
                )
            };
            if n < 0 {
                // ECANCELED is the step notification; an expiry (after the
                // far-future horizon) is handled the same way.
                let err = io::Error::last_os_error();
                match err.raw_os_error() {
                    Some(libc::ECANCELED) => {}
                    Some(libc::EINTR) => continue,
                    _ => return Err(err),
                }
            }

            let before = self.offset_ns;
            self.arm()?;
            return Ok(self.offset_ns - before);
        }
    }
}

/// Current `CLOCK_REALTIME - CLOCK_MONOTONIC` in nanoseconds.
///
/// The clocks cannot be read atomically, so the monotonic read is bracketed
/// by two realtime reads and the midpoint is used. Preemption inside the
/// bracket adds an error up to its width, so up to `BRACKET_ATTEMPTS`
/// brackets are taken until one is narrower than `MAX_BRACKET_NS`, keeping
/// the narrowest.
pub fn realtime_minus_monotonic() -> i128 {
    let mut best = (i128::MAX, 0);
    for _ in 0..BRACKET_ATTEMPTS {
        let rt0 = clock_ns(libc::CLOCK_REALTIME);
        let mono = clock_ns(libc::CLOCK_MONOTONIC);
        let rt1 = clock_ns(libc::CLOCK_REALTIME);
        let width = rt1 - rt0;
        if width < best.0 {
            best = (width, (rt0 + rt1) / 2 - mono);
        }
        if width <= MAX_BRACKET_NS {
            break;
        }
    }
    best.1
}

fn clock_ns(clock: libc::clockid_t) -> i128 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: ts is a valid timespec owned by this frame.
    unsafe { libc::clock_gettime(clock, &mut ts) };
    ts.tv_sec as i128 * 1_000_000_000 + ts.tv_nsec as i128
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offset_is_stable_without_steps() {
        let a = realtime_minus_monotonic();
        std::thread::sleep(std::time::Duration::from_millis(20));
        let b = realtime_minus_monotonic();
        // Only slews can move it here: well under a millisecond in 20 ms.
        assert!((b - a).abs() < 1_000_000, "offset moved {} ns", b - a);
    }

    #[test]
    fn watcher_arms() {
        let watcher = ClockStepWatcher::new().unwrap();
        assert!(watcher.offset_ns > 0);
    }
}
