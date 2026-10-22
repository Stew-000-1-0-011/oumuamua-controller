//! Receives the bridge's BEACON broadcasts on UDP `BEACON_PORT`.

use proto::{BEACON_PORT, Datagram};
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

pub type Beacons = Receiver<(String, SocketAddr)>;

pub struct Discovery {
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl Discovery {
    /// Listen on `BEACON_PORT`. Yields `(robot_name, bridge address)` for every beacon.
    pub fn start() -> std::io::Result<(Discovery, Beacons)> {
        let (d, rx, _) = Self::start_on(SocketAddr::from(([0, 0, 0, 0], BEACON_PORT)))?;
        Ok((d, rx))
    }

    /// Like `start` but on an explicit bind address; also returns the bound address.
    pub fn start_on(
        bind: SocketAddr,
    ) -> std::io::Result<(Discovery, Beacons, SocketAddr)> {
        let sock = UdpSocket::bind(bind)?;
        sock.set_read_timeout(Some(Duration::from_millis(50)))?;
        let local = sock.local_addr()?;
        let (tx, rx) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let handle = std::thread::spawn(move || {
            let mut buf = [0u8; 2048];
            while !stop2.load(Ordering::Relaxed) {
                let Ok((n, src)) = sock.recv_from(&mut buf) else { continue };
                if let Ok(Datagram::Beacon { robot_name, port }) = Datagram::decode(&buf[..n]) {
                    let mut addr = src;
                    addr.set_port(port);
                    if tx.send((robot_name, addr)).is_err() {
                        break;
                    }
                }
            }
        });
        Ok((Discovery { stop, handle: Some(handle) }, rx, local))
    }

    pub fn stop(self) {}
}

impl Drop for Discovery {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beacon_yields_name_and_addr_with_advertised_port() {
        let (d, rx, local) = Discovery::start_on("127.0.0.1:0".parse().unwrap()).unwrap();
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let b = Datagram::Beacon { robot_name: "oumuamua".into(), port: 47800 };
        tx.send_to(&Datagram::Bye { session: 1, reason: "x".into() }.encode(), local).unwrap();
        tx.send_to(b"garbage", local).unwrap();
        tx.send_to(&b.encode(), local).unwrap();
        let (name, addr) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(name, "oumuamua");
        assert_eq!(addr, "127.0.0.1:47800".parse().unwrap());
        d.stop();
    }
}
