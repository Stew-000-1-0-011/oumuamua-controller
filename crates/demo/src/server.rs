//! UDP bridge server for the demo robot: one worker thread handles sessions, the reliable
//! streams, the stop rules and requests, and sends STATE / SCAN / LOG on a timer.

use crate::shell::{self, Shell};
use crate::sim::{self, Event, Sim};
use link::reliable::{Receiver as RelRx, Sender as RelTx};
use proto::{Control, Datagram, MAX_POINTS_PER_SCAN, Mode, Msg, State, Wheel};
use std::collections::HashMap;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const READ_TIMEOUT: Duration = Duration::from_millis(4);
const CONTROL_TIMEOUT_MS: u64 = 300;
const OPERATOR_TIMEOUT_MS: u64 = 5000;
const STATE_MS: u64 = 50;
const SCAN_MS: u64 = 100;
const LOG_MS: u64 = 3000;
const LOG_PENDING: usize = 32;
const MAX_V: f64 = 2.0;
const MAX_W: f64 = 2.5;
const LOG_LINES: [&str; 3] = ["デモ: 車輪の電圧 24.0 V", "デモ: 吸着ポンプは正常です", "デモ: LiDAR 10 Hz で動作中"];

/// The running demo robot. Dropping it (or `stop`) ends the worker thread.
pub struct DemoRobot {
    addr: SocketAddr,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl DemoRobot {
    /// Listen on a free port of 127.0.0.1 and start the simulation.
    pub fn start() -> io::Result<DemoRobot> {
        let sock = UdpSocket::bind("127.0.0.1:0")?;
        sock.set_read_timeout(Some(READ_TIMEOUT))?;
        let addr = sock.local_addr()?;
        let stop = Arc::new(AtomicBool::new(false));
        let flag = stop.clone();
        let handle = std::thread::Builder::new()
            .name("demo-robot".into())
            .spawn(move || Server::new(sock).run(&flag))?;
        Ok(DemoRobot { addr, stop, handle: Some(handle) })
    }

    pub fn addr(&self) -> SocketAddr {
        self.addr
    }

    pub fn stop(self) {}
}

impl Drop for DemoRobot {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

struct Session {
    id: u32,
    addr: SocketAddr,
    last_rx: u64,
    max_seq: u32,
    ack_seq: u32,
    ack_t: u32,
}

struct Server {
    sock: UdpSocket,
    t0: Instant,
    last_advance: u64,
    sim: Sim,
    shell: Shell,
    sess: Option<Session>,
    out: RelTx<Msg>,
    inc: RelRx<Msg>,
    // stop rules
    mode: Mode,
    reason: String,
    last_control: Option<u64>,
    estop: bool,
    v: [f64; 3],
    field_frame: bool,
    jog: [i8; 2],
    // reporting
    pending_lifts: HashMap<u32, (usize, Vec<String>)>,
    log_level: u8,
    state_seq: u32,
    scan_id: u32,
    log_n: usize,
    sent_memory: sim::Memory,
    next: [u64; 3], // state, scan, log
}

fn finite(v: f64) -> f64 {
    if v.is_finite() { v } else { 0.0 }
}

fn targets(gripper: u8) -> Result<Vec<usize>, String> {
    match gripper {
        0 | 1 => Ok(vec![usize::from(gripper)]),
        2 => Ok(vec![0, 1]),
        g => Err(format!("bad request: bad gripper {g}")),
    }
}

impl Server {
    fn new(sock: UdpSocket) -> Server {
        Server {
            sock,
            t0: Instant::now(),
            last_advance: 0,
            sim: Sim::default(),
            shell: Shell::default(),
            sess: None,
            out: RelTx::new(),
            inc: RelRx::new(),
            mode: Mode::Stopped,
            reason: "link lost".into(),
            last_control: None,
            estop: false,
            v: [0.0; 3],
            field_frame: false,
            jog: [0; 2],
            pending_lifts: HashMap::new(),
            log_level: 20,
            state_seq: 0,
            scan_id: 0,
            log_n: 0,
            sent_memory: sim::Memory::default(),
            next: [0; 3],
        }
    }

    fn run(&mut self, stop: &AtomicBool) {
        let mut buf = [0u8; 2048];
        while !stop.load(Ordering::Relaxed) {
            if let Ok((n, src)) = self.sock.recv_from(&mut buf)
                && let Ok(d) = Datagram::decode(&buf[..n])
            {
                self.on_datagram(d, src);
            }
            self.tick();
        }
    }

    fn now(&self) -> u64 {
        self.t0.elapsed().as_millis() as u64
    }

    fn tx(&self, d: &Datagram, to: SocketAddr) {
        let _ = self.sock.send_to(&d.encode(), to);
    }

    /// Queue a REL for the operator (dropped when nobody is connected).
    fn send(&mut self, m: Msg) {
        if self.sess.is_some() {
            self.out.push(m);
        }
    }

    fn reply(&mut self, req: u32, ok: bool, message: impl Into<String>) {
        self.send(Msg::Result { req, ok, message: message.into() });
    }

    // ----------------------------------------------------------- datagrams
    fn on_datagram(&mut self, d: Datagram, src: SocketAddr) {
        let now = self.now();
        let session = match &d {
            Datagram::Hello { session, client_name, .. } => return self.hello(*session, client_name, src),
            Datagram::Control(c) => c.session,
            Datagram::Rel { session, .. } | Datagram::Ack { session, .. } | Datagram::Bye { session, .. } => *session,
            _ => return,
        };
        match &mut self.sess {
            Some(s) if s.id == session => s.last_rx = now,
            _ => return,
        }
        match d {
            Datagram::Control(c) => self.on_control(c, now),
            Datagram::Rel { seq, msg, .. } => {
                for m in self.inc.on_rel(seq, msg) {
                    self.handle(m);
                }
                if let Some(s) = &self.sess {
                    self.tx(&Datagram::Ack { session: s.id, cum: self.inc.cum }, s.addr);
                }
            }
            Datagram::Ack { cum, .. } => self.out.on_ack(cum),
            Datagram::Bye { .. } => {
                self.sess = None;
                self.link_lost();
            }
            _ => {}
        }
    }

    /// The demo takes the latest HELLO: a new session replaces the old one.
    fn hello(&mut self, id: u32, _name: &str, addr: SocketAddr) {
        if id == 0 {
            return;
        }
        let now = self.now();
        let welcome = Datagram::Welcome { session: id, robot_name: sim::ROBOT_NAME.into(), accepted: true, reason: String::new() };
        if let Some(s) = &mut self.sess {
            if s.id == id {
                (s.addr, s.last_rx) = (addr, now);
                return self.tx(&welcome, addr);
            }
            let (old, old_addr) = (s.id, s.addr);
            self.tx(&Datagram::Bye { session: old, reason: "taken over".into() }, old_addr);
        }
        self.link_lost();
        self.sess = Some(Session { id, addr, last_rx: now, max_seq: 0, ack_seq: 0, ack_t: 0 });
        self.out.reset();
        self.inc.reset();
        self.pending_lifts.clear();
        self.tx(&welcome, addr);
        self.initial_burst();
        self.next = [now; 3];
    }

    fn initial_burst(&mut self) {
        self.sent_memory = self.sim.memory().clone();
        let mut burst = vec![
            Msg::Config {
                robot_name: sim::ROBOT_NAME.into(),
                paths: sim::PATHS.iter().map(|s| (*s).to_owned()).collect(),
                lift_presets: sim::LIFT_PRESETS.iter().map(|(n, h)| ((*n).to_owned(), *h)).collect(),
                lift_max: sim::LIFT_MAX,
                speed_levels: sim::SPEED_LEVELS,
                blocks: sim::BLOCKS,
                columns: sim::COLUMNS,
                rows: sim::ROWS,
            },
            Msg::Field {
                bounds: sim::BOUNDS,
                obstacles: sim::OBSTACLES.to_vec(),
                shelf: sim::SHELF,
                start: sim::START,
                robot_radius: sim::ROBOT_RADIUS,
            },
            self.path_shape_msg(),
            self.memory_msg(),
        ];
        for tab in self.shell.tabs() {
            burst.extend(shell::opened_msgs(&tab, self.shell.replay(&tab).unwrap_or("")));
        }
        for m in burst {
            self.send(m);
        }
    }

    fn path_shape_msg(&self) -> Msg {
        let name = self.sim.path_state().map(|p| p.0).unwrap_or_default();
        Msg::PathShape { name, points: self.sim.path_points() }
    }

    fn memory_msg(&self) -> Msg {
        let m = self.sim.memory();
        Msg::TaskMemory { holding: m.holding, picked: m.picked.clone(), placed: m.placed.clone() }
    }

    // ------------------------------------------------------- stop / control
    fn on_control(&mut self, c: Control, now: u64) {
        let Some(s) = &mut self.sess else { return };
        if c.seq <= s.max_seq {
            return;
        }
        (s.max_seq, s.ack_seq, s.ack_t) = (c.seq, c.seq, c.t_ms);
        self.last_control = Some(now);
        self.estop = c.estop;
        self.v = [finite(c.vx), finite(c.vy), finite(c.wz)];
        self.field_frame = c.field_frame;
        if self.estop {
            if self.mode == Mode::Stopped {
                self.reason = "estop".into();
            } else {
                self.enter_stop("estop");
            }
        }
        let jog = [c.jog_left, c.jog_right];
        if self.mode != Mode::Stopped {
            for (g, &j) in jog.iter().enumerate() {
                if j != self.jog[g] {
                    let ev = match j {
                        1.. => self.sim.lift_to(g, sim::LIFT_MAX, None),
                        0 => self.sim.lift_cancel(g),
                        _ => self.sim.lift_to(g, 0.0, None),
                    };
                    self.events(ev);
                }
            }
        }
        // While stopped, held keys do not start anything after the release.
        self.jog = jog;
    }

    fn link_lost(&mut self) {
        if self.mode != Mode::Stopped {
            self.enter_stop("link lost");
        }
    }

    fn enter_stop(&mut self, reason: &str) {
        self.mode = Mode::Stopped;
        self.reason = reason.to_owned();
        self.sim.cancel_task();
        self.cancel_path();
        self.sim.tracker = false;
        for g in 0..2 {
            let ev = self.sim.lift_cancel(g);
            self.events(ev);
            self.sim.suction[g] = false;
        }
    }

    /// Cancels a running path; the phone gets an empty PATH_SHAPE.
    fn cancel_path(&mut self) {
        if self.sim.cancel_path() {
            self.sim.tracker = false;
            self.send(Msg::PathShape { name: String::new(), points: Vec::new() });
        }
    }

    fn release(&mut self) -> Result<(), &'static str> {
        let fresh = self.last_control.is_some_and(|t| self.now().saturating_sub(t) <= CONTROL_TIMEOUT_MS);
        if self.estop {
            return Err("estop is still pressed");
        }
        if !fresh {
            return Err("no control received");
        }
        if self.mode == Mode::Stopped {
            self.mode = Mode::Manual;
            self.reason.clear();
            self.v = [0.0; 3]; // drive only on a CONTROL received after the release
        }
        Ok(())
    }

    fn enter_auto(&mut self) {
        if self.mode != Mode::Stopped {
            self.mode = Mode::Auto;
        }
    }

    fn leave_auto(&mut self) {
        if self.mode == Mode::Auto {
            self.mode = Mode::Manual;
        }
    }

    /// Answers a refused actuation request; true if the robot is stopped.
    fn refuse_if_stopped(&mut self, req: u32) -> bool {
        let stopped = self.mode == Mode::Stopped;
        if stopped {
            let why = format!("stopped: {}", self.reason);
            self.reply(req, false, why);
        }
        stopped
    }

    // -------------------------------------------------------------- events
    fn events(&mut self, events: Vec<Event>) {
        for e in events {
            match e {
                Event::Lift { req, g, ok, msg } => {
                    let Some(entry) = self.pending_lifts.get_mut(&req) else { continue };
                    entry.0 -= 1;
                    if !ok {
                        entry.1.push(format!("gripper {g}: {msg}"));
                    }
                    if entry.0 == 0 {
                        let (_, fails) = self.pending_lifts.remove(&req).unwrap();
                        self.reply(req, fails.is_empty(), fails.join("; "));
                    }
                }
                Event::TaskDone { kind } => {
                    self.send(Msg::TaskDone { kind, ok: true, message: String::new() });
                    self.leave_auto();
                }
                Event::PathDone => {
                    self.sim.tracker = false;
                    self.send(Msg::PathShape { name: String::new(), points: Vec::new() });
                    self.leave_auto();
                }
            }
        }
    }

    // ------------------------------------------------------------ requests
    fn handle(&mut self, m: Msg) {
        match m {
            Msg::ReleaseStop { req } => match self.release() {
                Ok(()) => self.reply(req, true, ""),
                Err(e) => self.reply(req, false, e),
            },
            Msg::SetSuction { req, gripper, on } => match targets(gripper) {
                Err(e) => self.reply(req, false, e),
                Ok(gs) => {
                    if on && self.refuse_if_stopped(req) {
                        return;
                    }
                    for g in gs {
                        self.sim.suction[g] = on;
                    }
                    self.reply(req, true, "");
                }
            },
            Msg::LiftTo { req, gripper, height } => {
                let gs = match targets(gripper) {
                    Ok(gs) if (0.0..=sim::LIFT_MAX).contains(&height) => gs,
                    Ok(_) => return self.reply(req, false, format!("height out of range (0..{})", sim::LIFT_MAX)),
                    Err(e) => return self.reply(req, false, e),
                };
                if self.refuse_if_stopped(req) {
                    return;
                }
                self.pending_lifts.insert(req, (gs.len(), Vec::new()));
                for g in gs {
                    let ev = self.sim.lift_to(g, height, Some(req));
                    self.events(ev);
                }
            }
            Msg::LiftZero { req, gripper } => match targets(gripper) {
                Err(e) => self.reply(req, false, e),
                Ok(gs) => {
                    if self.refuse_if_stopped(req) {
                        return;
                    }
                    for g in gs {
                        let ev = self.sim.lift_zero(g);
                        self.events(ev);
                    }
                    self.reply(req, true, "");
                }
            },
            Msg::RunTask { req, kind, gripper, block, column, row } => {
                if self.sim.busy() {
                    self.reply(req, false, "busy");
                } else if !self.refuse_if_stopped(req) {
                    self.enter_auto();
                    self.sim.start_task(kind, [gripper, block, column, row]);
                    self.reply(req, true, "");
                }
            }
            Msg::Cancel { req } => {
                self.sim.cancel_task();
                self.cancel_path();
                self.sim.tracker = false;
                self.leave_auto();
                self.reply(req, true, "");
            }
            Msg::StartPath { req, name } => {
                if self.sim.busy() {
                    self.reply(req, false, "busy");
                } else if self.refuse_if_stopped(req) {
                } else if !sim::PATHS.contains(&name.as_str()) {
                    self.reply(req, false, format!("unknown path: {name}"));
                } else {
                    self.sim.tracker = true;
                    self.sim.start_path(&name);
                    self.enter_auto();
                    self.reply(req, true, "");
                    let shape = self.path_shape_msg();
                    self.send(shape);
                }
            }
            Msg::SetTracker { req, on } => {
                if !(on && self.refuse_if_stopped(req)) {
                    self.sim.tracker = on;
                    self.reply(req, true, "");
                }
            }
            Msg::SetPose { req, x, y, yaw } => {
                self.sim.pose = [x, y, yaw];
                self.reply(req, true, "");
            }
            Msg::TaskReset { req } => {
                self.sim.reset_memory();
                self.reply(req, true, "");
            }
            Msg::LogLevel { req, min_level } => {
                self.log_level = min_level;
                self.reply(req, true, "");
            }
            Msg::ShellOpen { req, tab } => match self.shell.open(&tab) {
                Ok(()) => {
                    for m in shell::opened_msgs(&tab, self.shell.replay(&tab).unwrap_or("")) {
                        self.send(m);
                    }
                    self.reply(req, true, "");
                }
                Err(e) => self.reply(req, false, e),
            },
            Msg::ShellInput { req, tab, data } => match self.shell.input(&tab, &data) {
                Ok(text) => {
                    for m in shell::output_msgs(&tab, &text) {
                        self.send(m);
                    }
                    self.reply(req, true, "");
                }
                Err(e) => self.reply(req, false, e),
            },
            Msg::ShellComplete { req, tab, line, cursor } => {
                if self.shell.has_tab(&tab) {
                    let (start, candidates) = shell::complete(&line, cursor as usize);
                    self.send(Msg::ShellCompletion { req, tab, start, candidates });
                    self.reply(req, true, "");
                } else {
                    self.reply(req, false, "no such tab");
                }
            }
            Msg::ShellClose { req, tab } => match self.shell.close(&tab) {
                Ok(()) => {
                    self.send(Msg::ShellClosed { tab, reason: "closed".into() });
                    self.reply(req, true, "");
                }
                Err(e) => self.reply(req, false, e),
            },
            Msg::ShellRename { req, tab, new_name } => match self.shell.rename(&tab, &new_name) {
                Ok(()) => self.reply(req, true, ""),
                Err(e) => self.reply(req, false, e),
            },
            _ => {}
        }
    }

    // -------------------------------------------------------------- periodic
    fn tick(&mut self) {
        let now = self.now();
        let dt = (now - self.last_advance) as f64 / 1000.0;
        self.last_advance = now;
        let ev = self.sim.advance(dt);
        self.events(ev);
        self.drive(now);
        if self.sess.as_ref().is_some_and(|s| now.saturating_sub(s.last_rx) > OPERATOR_TIMEOUT_MS) {
            self.sess = None;
            self.link_lost();
        }
        let Some(s) = &self.sess else { return };
        let (id, addr) = (s.id, s.addr);
        if now >= self.next[0] {
            self.next[0] = now + STATE_MS;
            self.send_state();
            if self.sim.memory() != &self.sent_memory {
                self.sent_memory = self.sim.memory().clone();
                let m = self.memory_msg();
                self.send(m);
            }
        }
        if now >= self.next[1] {
            self.next[1] = now + SCAN_MS;
            self.send_scan();
        }
        if now >= self.next[2] {
            self.next[2] = now + LOG_MS;
            self.send_log();
        }
        for (seq, msg) in self.out.poll(now) {
            self.tx(&Datagram::Rel { session: id, seq, msg }, addr);
        }
    }

    /// The deadman and the manual velocity (limits, field frame).
    fn drive(&mut self, now: u64) {
        let timed_out = self.last_control.is_none_or(|t| now.saturating_sub(t) > CONTROL_TIMEOUT_MS);
        if self.mode != Mode::Stopped && timed_out {
            self.enter_stop("link lost");
        }
        self.sim.cmd = match self.mode {
            Mode::Manual => {
                let [mut vx, mut vy, wz] = self.v;
                if self.field_frame {
                    let (s, c) = self.sim.pose[2].sin_cos();
                    (vx, vy) = (c * vx + s * vy, -s * vx + c * vy);
                }
                let n = vx.hypot(vy);
                if n > MAX_V {
                    (vx, vy) = (vx * MAX_V / n, vy * MAX_V / n);
                }
                [vx, vy, wz.clamp(-MAX_W, MAX_W)]
            }
            _ => [0.0; 3],
        };
    }

    fn send_state(&mut self) {
        let Some(s) = &self.sess else { return };
        self.state_seq += 1;
        let body = State {
            mode: self.mode,
            stop_reason: self.reason.clone(),
            pose: Some(self.sim.pose),
            vel: Some(self.sim.cmd),
            wheels: vec![Wheel { vdc: 24.0, ok: true }; 3],
            grippers: self.sim.grippers(),
            task: self.sim.task_state(),
            path: self.sim.path_state(),
            tracker_enabled: self.sim.tracker,
        };
        let d = Datagram::State { session: s.id, seq: self.state_seq, ack_seq: s.ack_seq, ack_t_ms: s.ack_t, body };
        self.tx(&d, s.addr);
    }

    fn send_scan(&mut self) {
        let Some(s) = &self.sess else { return };
        let points = self.sim.scan();
        self.scan_id += 1;
        let parts: Vec<&[(i16, i16)]> = points.chunks(MAX_POINTS_PER_SCAN).collect();
        for (i, part) in parts.iter().enumerate() {
            let d = Datagram::Scan {
                session: s.id,
                scan_id: self.scan_id,
                frag: i as u8,
                frag_count: parts.len() as u8,
                points: part.to_vec(),
            };
            self.tx(&d, s.addr);
        }
    }

    fn send_log(&mut self) {
        let text = LOG_LINES[self.log_n % LOG_LINES.len()];
        self.log_n += 1;
        if self.log_level > 20 || self.out.pending() > LOG_PENDING {
            return;
        }
        let stamp_ms = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64);
        self.send(Msg::Log { stamp_ms, level: 20, name: "demo".into(), text: text.into() });
    }
}
