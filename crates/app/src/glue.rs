//! Connects the Slint window to the link: callbacks in, a 33 ms timer out.

use crate::model::{AppModel, ConnState};
use crate::{Beacon, MainWindow, storage};
use link::{Client, ControlInput, Discovery, LinkEvent};
use proto::{Mode, Msg};
use slint::{ComponentHandle, Model, ModelRc, SharedString, Timer, TimerMode, VecModel};
use std::cell::RefCell;
use std::net::{SocketAddr, ToSocketAddrs};
use std::rc::Rc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const TICK: Duration = Duration::from_millis(33);
const NAME_FILE: &str = "client_name";
const TARGET_FILE: &str = "last_target";

/// Per-device unique id, generated once and then reused forever.
pub fn client_name() -> String {
    if let Some(n) = storage::load(NAME_FILE) {
        let n = n.trim().to_owned();
        if !n.is_empty() {
            return n;
        }
    }
    let name = new_client_name();
    storage::save(NAME_FILE, &name);
    name
}

fn new_client_name() -> String {
    use std::hash::{BuildHasher, Hasher};
    let mut h = std::collections::hash_map::RandomState::new().build_hasher();
    h.write_u128(SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_nanos()));
    h.write_u32(std::process::id());
    format!("phone-{:08x}", h.finish() as u32)
}

/// "host" or "host:port" (default port `proto::PORT`) to a socket address.
pub fn parse_target(text: &str) -> Result<SocketAddr, String> {
    let t = text.trim();
    if t.is_empty() {
        return Err("IP アドレスを入力してください".into());
    }
    let with_port = if t.parse::<SocketAddr>().is_ok() {
        t.to_owned()
    } else if t.parse::<std::net::Ipv6Addr>().is_ok() {
        format!("[{t}]:{}", proto::PORT)
    } else if t.rsplit_once(':').is_some_and(|(_, p)| p.parse::<u16>().is_ok()) {
        t.to_owned()
    } else {
        format!("{t}:{}", proto::PORT)
    };
    with_port
        .to_socket_addrs()
        .ok()
        .and_then(|mut a| a.next())
        .ok_or_else(|| format!("宛先を解釈できません: {t}"))
}

struct Glue {
    window: slint::Weak<MainWindow>,
    model: AppModel,
    client: Option<Client>,
    rx: Option<Receiver<LinkEvent>>,
    _discovery: Option<Discovery>,
    beacons_rx: Option<link::discovery::Beacons>,
    beacons: Vec<(String, SocketAddr)>,
    beacons_model: Rc<VecModel<Beacon>>,
    /// Phone-side emergency stop latch; sent as `ControlInput.estop`.
    control: ControlInput,
    /// Request id of the RELEASE_STOP we are waiting on.
    pending_release: Option<u32>,
    notice: String,
    status: String,
    name: String,
}

impl Glue {
    fn push_control(&self) {
        if let Some(c) = &self.client {
            c.set_control(self.control);
        }
    }

    fn connect(&mut self, target: &str, takeover: bool) {
        let addr = match parse_target(target) {
            Ok(a) => a,
            Err(e) => {
                self.status = e;
                return;
            }
        };
        storage::save(TARGET_FILE, target.trim());
        // Drop the old client first: its thread uses the same session socket space.
        self.client = None;
        self.model.begin_connect();
        self.notice.clear();
        self.pending_release = None;
        self.control = ControlInput::default();
        let (client, rx) = Client::start(addr, self.name.clone(), takeover);
        self.client = Some(client);
        self.rx = Some(rx);
        self.status = format!("{addr} に接続中…");
    }

    fn press_stop(&mut self) {
        self.control.estop = true;
        self.notice.clear();
        self.push_control();
    }

    fn press_release(&mut self) {
        self.control.estop = false;
        self.notice.clear();
        self.push_control();
        if let Some(c) = &self.client {
            let req = c.next_req();
            c.send(Msg::ReleaseStop { req });
            self.pending_release = Some(req);
        }
    }

    fn poll(&mut self) {
        if let Some(rx) = &self.beacons_rx {
            while let Ok((name, addr)) = rx.try_recv() {
                match self.beacons.iter_mut().find(|(n, a)| *n == name && *a == addr) {
                    Some(_) => {}
                    None => self.beacons.push((name, addr)),
                }
            }
        }
        let mut events = Vec::new();
        if let Some(rx) = &self.rx {
            while let Ok(ev) = rx.try_recv() {
                events.push(ev);
            }
        }
        for ev in events {
            if matches!(ev, LinkEvent::Rejected { .. } | LinkEvent::Bye { .. }) {
                // The client thread idles after these; release it so Connect starts a fresh one.
                self.client = None;
                self.rx = None;
                self.control.estop = false;
            }
            self.model.apply(ev);
        }
        if let Some(req) = self.pending_release
            && let Some((_, ok, msg)) = self.model.results.iter().find(|r| r.0 == req)
        {
            if !ok {
                self.notice = format!("解除できません: {msg}");
            }
            self.pending_release = None;
        }
        self.refresh();
    }

    fn refresh(&self) {
        let Some(w) = self.window.upgrade() else { return };
        let conn = self.model.connection();
        let (connected, text, color, rtt) = match &conn {
            ConnState::Searching => (false, "未接続", slint::Color::from_rgb_u8(0x8d, 0x99, 0xa8), -1),
            ConnState::Connecting => (false, "接続中", slint::Color::from_rgb_u8(0xd9, 0xa2, 0x1b), -1),
            ConnState::Connected { rtt_ms } => {
                (true, "接続", slint::Color::from_rgb_u8(0x3f, 0xb9, 0x50), rtt_ms.map_or(-1, |v| v as i32))
            }
            ConnState::Lost => (true, "切断", slint::Color::from_rgb_u8(0xd1, 0x34, 0x2f), -1),
            ConnState::Rejected(_) => (false, "拒否", slint::Color::from_rgb_u8(0xd1, 0x34, 0x2f), -1),
        };
        let mode = self.model.state().map_or(-1, |s| match s.mode {
            Mode::Manual => 0,
            Mode::Auto => 1,
            Mode::Stopped => 2,
        });
        w.set_connected(connected);
        w.set_conn_text(text.into());
        w.set_conn_color(color);
        w.set_rtt_ms(rtt);
        w.set_mode(mode);
        w.set_stopped(self.control.estop || mode == 2);
        w.set_notice(self.notice.as_str().into());
        let (status, status_color) = match &conn {
            ConnState::Rejected(r) => (format!("接続できません: {r}"), slint::Color::from_rgb_u8(0xd1, 0x34, 0x2f)),
            ConnState::Connecting => (self.status.clone(), slint::Color::from_rgb_u8(0xd9, 0xa2, 0x1b)),
            _ if self.beacons.is_empty() && self.status.is_empty() => {
                ("ロボットを探しています…".to_owned(), slint::Color::from_rgb_u8(0x8d, 0x99, 0xa8))
            }
            _ => (self.status.clone(), slint::Color::from_rgb_u8(0x8d, 0x99, 0xa8)),
        };
        w.set_connect_status(status.into());
        w.set_connect_status_color(status_color);
        if self.beacons_model.row_count() != self.beacons.len() {
            self.beacons_model.set_vec(
                self.beacons
                    .iter()
                    .map(|(n, a)| Beacon { name: n.as_str().into(), addr: a.to_string().into() })
                    .collect::<Vec<_>>(),
            );
        }
    }
}

/// Wires `window` up; keep the returned handle alive for as long as the window runs.
pub fn start(window: &MainWindow) -> Rc<RefCell<GlueHandle>> {
    let (discovery, beacons_rx, status) = match Discovery::start() {
        Ok((d, rx)) => (Some(d), Some(rx), String::new()),
        Err(e) => (None, None, format!("名乗りを聞けません: {e}")),
    };
    let beacons_model = Rc::new(VecModel::default());
    window.set_beacons(ModelRc::from(beacons_model.clone()));
    window.set_target(SharedString::from(storage::load(TARGET_FILE).unwrap_or_default().trim()));
    let glue = Rc::new(RefCell::new(Glue {
        window: window.as_weak(),
        model: AppModel::new(),
        client: None,
        rx: None,
        _discovery: discovery,
        beacons_rx,
        beacons: Vec::new(),
        beacons_model,
        control: ControlInput::default(),
        pending_release: None,
        notice: String::new(),
        status,
        name: client_name(),
    }));
    {
        let g = glue.clone();
        window.on_connect(move |t, takeover| {
            let mut g = g.borrow_mut();
            g.connect(&t, takeover);
            g.refresh();
        });
        let g = glue.clone();
        window.on_stop_pressed(move || {
            let mut g = g.borrow_mut();
            g.press_stop();
            g.refresh();
        });
        let g = glue.clone();
        window.on_release_pressed(move || {
            let mut g = g.borrow_mut();
            g.press_release();
            g.refresh();
        });
    }
    let timer = Timer::default();
    let g = glue.clone();
    timer.start(TimerMode::Repeated, TICK, move || g.borrow_mut().poll());
    glue.borrow().refresh();
    Rc::new(RefCell::new(GlueHandle { _glue: glue, _timer: timer }))
}

/// Keeps the glue state and its timer alive.
pub struct GlueHandle {
    _glue: Rc<RefCell<Glue>>,
    _timer: Timer,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn target_parsing() {
        assert_eq!(parse_target("127.0.0.1").unwrap(), "127.0.0.1:47800".parse().unwrap());
        assert_eq!(parse_target(" 10.0.0.2:5000 ").unwrap(), "10.0.0.2:5000".parse().unwrap());
        assert!(parse_target("").is_err());
        assert!(parse_target("not an address").is_err());
    }

    #[test]
    fn client_names_look_right_and_differ() {
        let a = new_client_name();
        assert!(a.starts_with("phone-") && a.len() == 14, "{a}");
        assert_ne!(a, new_client_name());
    }
}
