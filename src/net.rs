// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! UDP reception with kernel receive timestamps.
//!
//! [`Receiver`] reads batches of datagrams with one system call and reports
//! each datagram's host receive time. On Linux the time comes from the
//! kernel `SO_TIMESTAMPNS` control message, taken when the datagram enters
//! the network stack, so it does not depend on when the application gets to
//! read the socket. Datagrams without a kernel timestamp are stamped with the
//! clock read right after the receive call.

use std::{
    io, mem,
    net::{IpAddr, Ipv4Addr, Ipv6Addr},
    os::fd::RawFd,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

/// Largest datagram a [`Receiver`] slot accepts by default. Robosense MSOP
/// packets are 1200 bytes and Ouster RNG15_RFL8_NIR8 packets are at most
/// about 8.5 KiB.
pub const MAX_DATAGRAM: usize = 16 * 1024;

/// Control message buffer for one datagram, large enough for an
/// `SCM_TIMESTAMPNS` message. The `u64` elements keep it aligned for
/// `cmsghdr`.
type CmsgBuf = [u64; 8];

/// One received datagram.
#[derive(Debug)]
pub struct Datagram<'a> {
    /// Datagram payload, cut to the slot size when truncated.
    pub data: &'a [u8],
    /// Host receive time (`CLOCK_REALTIME`).
    pub rx_time: SystemTime,
    /// Source address, when the kernel reported one.
    pub source: Option<IpAddr>,
    /// The datagram was larger than the slot and was cut.
    pub truncated: bool,
    /// `rx_time` is the kernel receive timestamp; otherwise it is the clock
    /// read right after the receive call.
    pub kernel_stamped: bool,
}

/// Enables kernel receive timestamps (`SO_TIMESTAMPNS`) on a socket.
///
/// The kernel enables timestamping through deferred work, so datagrams
/// received in the first moments after this call may be stamped when read
/// instead.
#[cfg(target_os = "linux")]
pub fn enable_rx_timestamps(fd: RawFd) -> io::Result<()> {
    let enable: libc::c_int = 1;
    // SAFETY: fd is a socket owned by the caller; the option value is a
    // properly sized c_int.
    let err = unsafe {
        libc::setsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_TIMESTAMPNS,
            &enable as *const _ as *const libc::c_void,
            mem::size_of_val(&enable) as libc::socklen_t,
        )
    };
    match err {
        0 => Ok(()),
        _ => Err(io::Error::last_os_error()),
    }
}

/// Kernel receive timestamps are Linux-only; elsewhere every datagram is
/// stamped when read.
#[cfg(not(target_os = "linux"))]
pub fn enable_rx_timestamps(_fd: RawFd) -> io::Result<()> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "kernel receive timestamps require Linux",
    ))
}

/// Reusable buffers for reading batches of datagrams.
pub struct Receiver {
    slot: usize,
    buf: Vec<u8>,
    lens: Vec<usize>,
    truncated: Vec<bool>,
    kernel_stamped: Vec<bool>,
    rx_times: Vec<SystemTime>,
    sources: Vec<Option<IpAddr>>,
    received: usize,
    #[cfg(target_os = "linux")]
    mmsgs: Vec<libc::mmsghdr>,
    #[cfg(target_os = "linux")]
    iovecs: Vec<libc::iovec>,
    #[cfg(target_os = "linux")]
    cmsgs: Vec<CmsgBuf>,
    #[cfg(target_os = "linux")]
    addrs: Vec<libc::sockaddr_storage>,
}

// SAFETY: the raw pointers in the mmsghdr and iovec arrays only ever point
// into this receiver's own heap buffers and are rewritten before every
// receive call, which takes `&mut self`. Moving the receiver to another
// thread is sound, and shared references only read owned buffers.
unsafe impl Send for Receiver {}
unsafe impl Sync for Receiver {}

impl Receiver {
    /// Creates a receiver reading up to `batch` datagrams of up to `slot`
    /// bytes per call.
    pub fn new(batch: usize, slot: usize) -> Self {
        let batch = batch.max(1);
        Self {
            slot,
            buf: vec![0; batch * slot],
            lens: vec![0; batch],
            truncated: vec![false; batch],
            kernel_stamped: vec![false; batch],
            rx_times: vec![UNIX_EPOCH; batch],
            sources: vec![None; batch],
            received: 0,
            // SAFETY: mmsghdr, iovec and sockaddr_storage are plain C structs
            // for which all zero bytes (null pointers, zero lengths) is valid.
            #[cfg(target_os = "linux")]
            mmsgs: vec![unsafe { mem::zeroed() }; batch],
            #[cfg(target_os = "linux")]
            iovecs: vec![unsafe { mem::zeroed() }; batch],
            #[cfg(target_os = "linux")]
            cmsgs: vec![[0; 8]; batch],
            #[cfg(target_os = "linux")]
            addrs: vec![unsafe { mem::zeroed() }; batch],
        }
    }

    /// Maximum number of datagrams read per call.
    pub fn batch(&self) -> usize {
        self.lens.len()
    }

    /// Reads up to [`batch`](Self::batch) queued datagrams without blocking
    /// and returns how many were read.
    ///
    /// # Errors
    ///
    /// Returns the receive error, including `WouldBlock` when no datagram
    /// is queued.
    #[cfg(target_os = "linux")]
    pub fn recv(&mut self, fd: RawFd) -> io::Result<usize> {
        let batch = self.batch();
        // Base pointers are taken once so that filling in one header does not
        // re-borrow the buffers the previous headers point into.
        let buf = self.buf.as_mut_ptr();
        let iovecs = self.iovecs.as_mut_ptr();
        let cmsgs = self.cmsgs.as_mut_ptr();
        let addrs = self.addrs.as_mut_ptr();
        let mmsgs = self.mmsgs.as_mut_ptr();

        // SAFETY: every index is below `batch`, the length of each array, and
        // `i * slot` stays within `buf`, which holds `batch * slot` bytes. The
        // headers point into buffers owned by self that stay alive and
        // unmoved for the duration of the recvmmsg call.
        let n = unsafe {
            for i in 0..batch {
                let iov = iovecs.add(i);
                (*iov).iov_base = buf.add(i * self.slot) as *mut libc::c_void;
                (*iov).iov_len = self.slot;
                let mmsg = mmsgs.add(i);
                mmsg.write(mem::zeroed());
                let hdr = &mut (*mmsg).msg_hdr;
                hdr.msg_name = addrs.add(i) as *mut libc::c_void;
                hdr.msg_namelen = mem::size_of::<libc::sockaddr_storage>() as libc::socklen_t;
                hdr.msg_iov = iov;
                hdr.msg_iovlen = 1;
                hdr.msg_control = cmsgs.add(i) as *mut libc::c_void;
                hdr.msg_controllen = mem::size_of::<CmsgBuf>() as _;
            }
            libc::recvmmsg(
                fd,
                mmsgs,
                batch as libc::c_uint,
                libc::MSG_DONTWAIT,
                std::ptr::null_mut(),
            )
        };
        let now = SystemTime::now();
        if n < 0 {
            self.received = 0;
            return Err(io::Error::last_os_error());
        }

        let n = n as usize;
        for i in 0..n {
            let mmsg = &self.mmsgs[i];
            self.lens[i] = (mmsg.msg_len as usize).min(self.slot);
            self.truncated[i] = mmsg.msg_hdr.msg_flags & libc::MSG_TRUNC != 0;
            // SAFETY: the kernel filled in this header and its control buffer,
            // which are still alive in self.cmsgs. A truncated control buffer
            // cannot be trusted to hold the timestamp.
            let kernel = match mmsg.msg_hdr.msg_flags & libc::MSG_CTRUNC {
                0 => unsafe { cmsg_timestamp(&mmsg.msg_hdr) },
                _ => None,
            };
            self.kernel_stamped[i] = kernel.is_some();
            self.rx_times[i] = kernel.unwrap_or(now);
            self.sources[i] = sockaddr_ip(&self.addrs[i], mmsg.msg_hdr.msg_namelen);
        }
        self.received = n;
        Ok(n)
    }

    /// Reads queued datagrams one at a time without blocking; each is
    /// stamped when read.
    #[cfg(not(target_os = "linux"))]
    pub fn recv(&mut self, fd: RawFd) -> io::Result<usize> {
        let mut n = 0;
        while n < self.batch() {
            // SAFETY: the address and slot buffers are owned by self and
            // sized as passed.
            let mut addr: libc::sockaddr_storage = unsafe { mem::zeroed() };
            let mut addrlen = mem::size_of::<libc::sockaddr_storage>() as libc::socklen_t;
            let len = unsafe {
                libc::recvfrom(
                    fd,
                    self.buf[n * self.slot..].as_mut_ptr() as *mut libc::c_void,
                    self.slot,
                    libc::MSG_DONTWAIT,
                    &mut addr as *mut _ as *mut libc::sockaddr,
                    &mut addrlen,
                )
            };
            if len < 0 {
                let err = io::Error::last_os_error();
                if n > 0 && err.kind() == io::ErrorKind::WouldBlock {
                    break;
                }
                self.received = 0;
                return Err(err);
            }
            self.lens[n] = len as usize;
            self.truncated[n] = false;
            self.kernel_stamped[n] = false;
            self.rx_times[n] = SystemTime::now();
            self.sources[n] = sockaddr_ip(&addr, addrlen);
            n += 1;
        }
        self.received = n;
        Ok(n)
    }

    /// Datagrams read by the last [`recv`](Self::recv) call.
    pub fn datagrams(&self) -> impl Iterator<Item = Datagram<'_>> {
        (0..self.received).map(|i| Datagram {
            data: &self.buf[i * self.slot..i * self.slot + self.lens[i]],
            rx_time: self.rx_times[i],
            source: self.sources[i],
            truncated: self.truncated[i],
            kernel_stamped: self.kernel_stamped[i],
        })
    }
}

/// Returns the `SCM_TIMESTAMPNS` receive time of a received message.
///
/// # Safety
///
/// `hdr` must describe a message filled in by recvmsg or recvmmsg whose
/// control buffer is still valid.
#[cfg(target_os = "linux")]
unsafe fn cmsg_timestamp(hdr: &libc::msghdr) -> Option<SystemTime> {
    // SAFETY: guaranteed by the caller.
    unsafe {
        let mut cmsg = libc::CMSG_FIRSTHDR(hdr);
        while !cmsg.is_null() {
            if (*cmsg).cmsg_level == libc::SOL_SOCKET && (*cmsg).cmsg_type == libc::SCM_TIMESTAMPNS
            {
                let ts = std::ptr::read_unaligned(libc::CMSG_DATA(cmsg) as *const libc::timespec);
                return timespec_to_system_time(&ts);
            }
            cmsg = libc::CMSG_NXTHDR(hdr, cmsg);
        }
    }
    None
}

#[cfg(target_os = "linux")]
fn timespec_to_system_time(ts: &libc::timespec) -> Option<SystemTime> {
    let secs = u64::try_from(ts.tv_sec).ok()?;
    let nanos = u32::try_from(ts.tv_nsec).ok()?;
    UNIX_EPOCH.checked_add(Duration::new(secs, nanos))
}

/// IP address of a socket address filled in by the kernel.
fn sockaddr_ip(addr: &libc::sockaddr_storage, len: libc::socklen_t) -> Option<IpAddr> {
    let len = len as usize;
    match addr.ss_family as libc::c_int {
        libc::AF_INET if len >= mem::size_of::<libc::sockaddr_in>() => {
            // SAFETY: the family and length identify a sockaddr_in, and
            // sockaddr_storage is large enough and suitably aligned for it.
            let sin = unsafe { &*(addr as *const _ as *const libc::sockaddr_in) };
            Some(IpAddr::V4(Ipv4Addr::from(u32::from_be(
                sin.sin_addr.s_addr,
            ))))
        }
        libc::AF_INET6 if len >= mem::size_of::<libc::sockaddr_in6>() => {
            // SAFETY: as above, for sockaddr_in6.
            let sin6 = unsafe { &*(addr as *const _ as *const libc::sockaddr_in6) };
            let ip = Ipv6Addr::from(sin6.sin6_addr.s6_addr);
            Some(match ip.to_ipv4_mapped() {
                Some(v4) => IpAddr::V4(v4),
                None => IpAddr::V6(ip),
            })
        }
        _ => None,
    }
}

/// Requests a UDP receive buffer of `size` bytes.
///
/// Uses `SO_RCVBUFFORCE`, which ignores the `net.core.rmem_max` limit but
/// requires `CAP_NET_ADMIN`, and falls back to `SO_RCVBUF`, which the kernel
/// silently caps at `net.core.rmem_max` (208 KiB by default). Returns the
/// usable size granted by the kernel.
#[cfg(target_os = "linux")]
pub fn set_recv_buffer(fd: RawFd, size: usize) -> io::Result<usize> {
    let requested = libc::c_int::try_from(size).unwrap_or(libc::c_int::MAX);
    let set = |option: libc::c_int| -> io::Result<()> {
        // SAFETY: fd is a socket owned by the caller; the option value is a
        // properly sized c_int.
        let err = unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                option,
                &requested as *const _ as *const libc::c_void,
                mem::size_of_val(&requested) as libc::socklen_t,
            )
        };
        match err {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error()),
        }
    };

    if set(libc::SO_RCVBUFFORCE).is_err() {
        set(libc::SO_RCVBUF)?;
    }

    let mut granted: libc::c_int = 0;
    let mut len = mem::size_of_val(&granted) as libc::socklen_t;
    // SAFETY: as above; granted and len are valid for writes.
    let err = unsafe {
        libc::getsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_RCVBUF,
            &mut granted as *mut _ as *mut libc::c_void,
            &mut len,
        )
    };
    if err != 0 {
        return Err(io::Error::last_os_error());
    }
    // The kernel reports double the usable size to account for bookkeeping.
    Ok(usize::try_from(granted / 2).unwrap_or(0))
}

/// The receive buffer is left at the system default outside Linux.
#[cfg(not(target_os = "linux"))]
pub fn set_recv_buffer(_fd: RawFd, _size: usize) -> io::Result<usize> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "receive buffer sizing requires Linux",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{net::UdpSocket, os::fd::AsRawFd, thread};

    fn loopback_pair() -> (UdpSocket, UdpSocket) {
        let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
        rx.set_nonblocking(true).unwrap();
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        (rx, tx)
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn recv_uses_kernel_timestamps_and_reports_source() {
        let (rx, tx) = loopback_pair();
        enable_rx_timestamps(rx.as_raw_fd()).unwrap();

        // The kernel enables receive timestamping through deferred work;
        // until it runs, datagrams are stamped when read instead.
        thread::sleep(Duration::from_millis(50));

        let before = SystemTime::now();
        for i in 0..3u8 {
            tx.send_to(&[i; 16], rx.local_addr().unwrap()).unwrap();
        }
        let sent = SystemTime::now();

        // Delay the read so a clock read after the receive call would land
        // well after `sent`; only a kernel timestamp can fall before it.
        thread::sleep(Duration::from_millis(50));

        let mut receiver = Receiver::new(8, 64);
        assert_eq!(receiver.recv(rx.as_raw_fd()).unwrap(), 3);

        let datagrams: Vec<_> = receiver.datagrams().collect();
        assert_eq!(datagrams.len(), 3);
        for (i, d) in datagrams.iter().enumerate() {
            assert!(d.rx_time >= before && d.rx_time <= sent, "datagram {i}");
            assert_eq!(d.data, &[i as u8; 16]);
            assert_eq!(d.source, Some(tx.local_addr().unwrap().ip()));
            assert!(!d.truncated);
            assert!(d.kernel_stamped);
        }
        assert!(datagrams.windows(2).all(|w| w[0].rx_time <= w[1].rx_time));
    }

    #[test]
    fn recv_on_empty_socket_would_block() {
        let (rx, _tx) = loopback_pair();
        let mut receiver = Receiver::new(4, 64);
        let err = receiver.recv(rx.as_raw_fd()).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::WouldBlock);
        assert_eq!(receiver.datagrams().count(), 0);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn oversized_datagram_is_truncated_and_flagged() {
        let (rx, tx) = loopback_pair();
        tx.send_to(&[7u8; 100], rx.local_addr().unwrap()).unwrap();
        thread::sleep(Duration::from_millis(20));

        let mut receiver = Receiver::new(2, 32);
        assert_eq!(receiver.recv(rx.as_raw_fd()).unwrap(), 1);
        let d = receiver.datagrams().next().unwrap();
        assert!(d.truncated);
        assert_eq!(d.data.len(), 32);
    }

    #[test]
    fn batch_is_bounded_by_capacity() {
        let (rx, tx) = loopback_pair();
        for i in 0..5u8 {
            tx.send_to(&[i; 8], rx.local_addr().unwrap()).unwrap();
        }
        thread::sleep(Duration::from_millis(20));

        let mut receiver = Receiver::new(2, 64);
        assert_eq!(receiver.recv(rx.as_raw_fd()).unwrap(), 2);
        assert_eq!(receiver.recv(rx.as_raw_fd()).unwrap(), 2);
        assert_eq!(receiver.recv(rx.as_raw_fd()).unwrap(), 1);
        assert_eq!(receiver.datagrams().next().unwrap().data, &[4u8; 8]);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn set_recv_buffer_grants_at_least_default() {
        let (rx, _tx) = loopback_pair();
        let granted = set_recv_buffer(rx.as_raw_fd(), 4 * 1024 * 1024).unwrap();
        assert!(granted > 0);
    }
}
