//! UDP client thread: handshake, CONTROL stream, reliable channel, STATE/SCAN intake.
//! std threads and a short socket read timeout only; no async runtime.

use crate::reliable::{Receiver as RelReceiver, Sender as RelSender};
use proto::{Control, Datagram, Msg, State, points_to_m};
use std::collections::hash_map::RandomState;
use std::hash::{BuildHasher, Hasher};
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const TICK_MS: u64 = 20;
const HELLO_MS: u64 = 500;
const LOST_MS: u64 = 1000;
const REHELLO_MS: u64 = 2000;
const READ_TIMEOUT: Duration = Duration::from_millis(5);
/// The UI must call `set_control` at least this often; older input is not trusted to still be held.
const CONTROL_STALE: Duration = Duration::from_millis(250);

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ControlInput {
    pub vx: f64,
    pub vy: f64,
    pub wz: f64,
    pub field_frame: bool,
    pub jog_left: i8,
    pub jog_right: i8,
    pub estop: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum LinkEvent {
    Connected { robot_name: String },
    Rejected { reason: String },
    State { body: State, rtt_ms: Option<u32> },
    /// One complete scan, points in metres (field frame).
    Scan(Vec<(f32, f32)>),
    Msg(Msg),
    /// No STATE for 1 s. Emitted once per silence.
    Lost,
    Bye { reason: String },
    /// The link thread could not start (e.g. the UDP socket could not be bound).
    Failed { message: String },
}

struct Shared {
    control: Mutex<(ControlInput, Instant)>,
    outbox: Mutex<Vec<Msg>>,
    next_req: AtomicU32,
    stop: AtomicBool,
}

pub struct Client {
    shared: Arc<Shared>,
    handle: Option<JoinHandle<()>>,
}

impl Client {
    /// Start the link thread. It sends HELLO until the bridge answers.
    /// After a rejection (`Rejected`) or `Bye` the thread stays idle; start a new `Client` to retry.
    pub fn start(
        addr: SocketAddr,
        client_name: String,
        takeover: bool,
    ) -> (Client, Receiver<LinkEvent>) {
        let shared = Arc::new(Shared {
            control: Mutex::new((ControlInput::default(), Instant::now())),
            outbox: Mutex::new(Vec::new()),
            next_req: AtomicU32::new(1),
            stop: AtomicBool::new(false),
        });
        let (tx, rx) = mpsc::channel();
        let sh = shared.clone();
        let handle = std::thread::spawn(move || {
            if let Err(e) = run(&sh, addr, &client_name, takeover, &tx) {
                eprintln!("link: client thread ended: {e}");
                let _ = tx.send(LinkEvent::Failed { message: e.to_string() });
            }
        });
        (Client { shared, handle: Some(handle) }, rx)
    }

    /// Set the input to stream. The UI must repeat this every frame (about 30 ms): when the last
    /// call is older than 250 ms the motion axes and jogs go out as zero (estop is kept).
    pub fn set_control(&self, c: ControlInput) {
        *self.shared.control.lock().unwrap() = (c, Instant::now());
    }

    /// Next request id (1, 2, ...).
    pub fn next_req(&self) -> u32 {
        self.shared.next_req.fetch_add(1, Ordering::Relaxed)
    }

    /// Queue a message on the reliable channel.
    pub fn send(&self, msg: Msg) {
        self.shared.outbox.lock().unwrap().push(msg);
    }

    /// Send BYE and end the thread.
    pub fn stop(self) {}
}

impl Drop for Client {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// The input to put on the wire: stale motion input becomes "no motion", estop is preserved.
fn fresh_control((c, set_at): (ControlInput, Instant), now: Instant) -> ControlInput {
    if now.saturating_duration_since(set_at) <= CONTROL_STALE {
        return c;
    }
    ControlInput { field_frame: c.field_frame, estop: c.estop, ..ControlInput::default() }
}

fn random_session(avoid: u32) -> u32 {
    loop {
        let n = RandomState::new().build_hasher().finish() as u32;
        if n != 0 && n != avoid {
            return n;
        }
    }
}

/// Fragments of the scan currently being assembled.
struct ScanAsm {
    id: u32,
    parts: Vec<Option<Vec<(i16, i16)>>>,
}

struct Link<'a> {
    sock: UdpSocket,
    addr: SocketAddr,
    tx: &'a Sender<LinkEvent>,
    t0: Instant,
    session: u32,
    welcomed: bool,
    /// Rejected or told BYE: stay quiet until stopped.
    halted: bool,
    last_hello: Option<u64>,
    last_state: u64,
    lost_sent: bool,
    seq: u32,
    out: RelSender<Msg>,
    inc: RelReceiver<Msg>,
    scan: Option<ScanAsm>,
    last_scan_done: Option<u32>,
}

impl Link<'_> {
    fn now(&self) -> u64 {
        self.t0.elapsed().as_millis() as u64
    }

    fn send(&self, d: &Datagram) {
        // Send errors (e.g. unreachable network) are transient; the protocol retries.
        let _ = self.sock.send_to(&d.encode(), self.addr);
    }

    fn emit(&self, ev: LinkEvent) {
        let _ = self.tx.send(ev);
    }

    fn new_session(&mut self) {
        self.session = random_session(self.session);
        self.welcomed = false;
        self.last_hello = None;
        self.lost_sent = false;
        self.seq = 0;
        self.out.reset();
        self.inc.reset();
        self.scan = None;
        self.last_scan_done = None;
    }

    fn on_datagram(&mut self, d: Datagram) {
        let now = self.now();
        match d {
            Datagram::Welcome { session, robot_name, accepted, reason }
                if session == self.session && !self.welcomed && !self.halted =>
            {
                if accepted {
                    self.welcomed = true;
                    self.last_state = now;
                    self.emit(LinkEvent::Connected { robot_name });
                } else {
                    self.halted = true;
                    self.emit(LinkEvent::Rejected { reason });
                }
            }
            Datagram::State { session, ack_seq, ack_t_ms, body, .. }
                if session == self.session && self.welcomed =>
            {
                self.last_state = now;
                self.lost_sent = false;
                let rtt_ms = (ack_seq != 0).then(|| (now as u32).wrapping_sub(ack_t_ms));
                self.emit(LinkEvent::State { body, rtt_ms });
            }
            Datagram::Scan { session, scan_id, frag, frag_count, points }
                if session == self.session && self.welcomed =>
            {
                self.on_scan(scan_id, frag, frag_count, points);
            }
            Datagram::Rel { session, seq, msg } if session == self.session && self.welcomed => {
                for m in self.inc.on_rel(seq, msg) {
                    // Unknown kinds are consumed (so the peer stops retransmitting) but not shown.
                    if !matches!(m, Msg::Unknown { .. }) {
                        self.emit(LinkEvent::Msg(m));
                    }
                }
                self.send(&Datagram::Ack { session: self.session, cum: self.inc.cum });
            }
            Datagram::Ack { session, cum } if session == self.session && self.welcomed => {
                self.out.on_ack(cum);
            }
            Datagram::Bye { session, reason } if session == self.session => {
                self.welcomed = false;
                self.halted = true;
                self.emit(LinkEvent::Bye { reason });
            }
            _ => {}
        }
    }

    fn on_scan(&mut self, id: u32, frag: u8, count: u8, points: Vec<(i16, i16)>) {
        if count == 0 || frag >= count || self.last_scan_done == Some(id) {
            return;
        }
        match &self.scan {
            Some(s) if s.id == id => {}
            // A different scan_id: the one being assembled is abandoned, unless this one is older.
            Some(s) if id.wrapping_sub(s.id) >= 1 << 31 => return,
            _ => self.scan = Some(ScanAsm { id, parts: vec![None; usize::from(count)] }),
        }
        let s = self.scan.as_mut().unwrap();
        if s.parts.len() != usize::from(count) {
            return;
        }
        s.parts[usize::from(frag)] = Some(points);
        if s.parts.iter().all(Option::is_some) {
            let all: Vec<(i16, i16)> = s.parts.iter().flatten().flatten().copied().collect();
            self.scan = None;
            self.last_scan_done = Some(id);
            self.emit(LinkEvent::Scan(points_to_m(&all)));
        }
    }

    fn tick(&mut self, sh: &Shared, client_name: &str, takeover: bool) {
        let now = self.now();
        if self.halted {
            return;
        }
        if !self.welcomed {
            if self.last_hello.is_none_or(|t| now - t >= HELLO_MS) {
                self.last_hello = Some(now);
                self.send(&Datagram::Hello {
                    session: self.session,
                    client_name: client_name.to_owned(),
                    takeover,
                });
            }
            return;
        }
        let silent = now - self.last_state;
        if silent >= REHELLO_MS {
            // Start over: new session, HELLO from scratch, queued REL dropped.
            sh.outbox.lock().unwrap().clear();
            self.new_session();
            self.tick(sh, client_name, takeover);
            return;
        }
        if silent >= LOST_MS && !self.lost_sent {
            self.lost_sent = true;
            self.emit(LinkEvent::Lost);
        }
        let c = fresh_control(*sh.control.lock().unwrap(), Instant::now());
        self.seq += 1;
        self.send(&Datagram::Control(Control {
            session: self.session,
            seq: self.seq,
            t_ms: now as u32,
            vx: c.vx,
            vy: c.vy,
            wz: c.wz,
            field_frame: c.field_frame,
            jog_left: c.jog_left,
            jog_right: c.jog_right,
            estop: c.estop,
        }));
        for m in std::mem::take(&mut *sh.outbox.lock().unwrap()) {
            self.out.push(m);
        }
        for (seq, msg) in self.out.poll(now) {
            self.send(&Datagram::Rel { session: self.session, seq, msg });
        }
    }
}

fn run(
    sh: &Shared,
    addr: SocketAddr,
    client_name: &str,
    takeover: bool,
    tx: &Sender<LinkEvent>,
) -> std::io::Result<()> {
    let bind: SocketAddr =
        if addr.is_ipv6() { "[::]:0".parse().unwrap() } else { "0.0.0.0:0".parse().unwrap() };
    let sock = UdpSocket::bind(bind)?;
    sock.set_read_timeout(Some(READ_TIMEOUT))?;
    let mut link = Link {
        sock,
        addr,
        tx,
        t0: Instant::now(),
        session: 0,
        welcomed: false,
        halted: false,
        last_hello: None,
        last_state: 0,
        lost_sent: false,
        seq: 0,
        out: RelSender::new(),
        inc: RelReceiver::new(),
        scan: None,
        last_scan_done: None,
    };
    link.new_session();
    let mut buf = [0u8; 65536];
    let mut next_tick = 0u64;
    while !sh.stop.load(Ordering::Relaxed) {
        if let Ok((n, src)) = link.sock.recv_from(&mut buf)
            && src == addr
            && let Ok(d) = Datagram::decode(&buf[..n])
        {
            link.on_datagram(d);
        }
        let now = link.now();
        if now >= next_tick {
            next_tick = now + TICK_MS;
            link.tick(sh, client_name, takeover);
        }
    }
    link.send(&Datagram::Bye { session: link.session, reason: "client closed".into() });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stale_control_is_zeroed_but_estop_kept() {
        let c = ControlInput {
            vx: 1.0,
            vy: -1.0,
            wz: 2.0,
            field_frame: true,
            jog_left: 1,
            jog_right: -1,
            estop: true,
        };
        let t0 = Instant::now();
        assert_eq!(fresh_control((c, t0), t0 + Duration::from_millis(100)), c);
        let z = fresh_control((c, t0), t0 + Duration::from_millis(300));
        assert_eq!((z.vx, z.vy, z.wz, z.jog_left, z.jog_right), (0.0, 0.0, 0.0, 0, 0));
        assert!(z.estop && z.field_frame);
    }
}
