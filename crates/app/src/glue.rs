//! Connects the Slint window to the link: callbacks in, a 33 ms timer out.

use crate::manual::{self, Manual};
use crate::model::{AppModel, ConnState, StopLatch};
use crate::map::{self, PaneState, Scene};
use crate::{Beacon, MainWindow, PresetInfo, WheelInfo, storage};
use link::{Client, ControlInput, Discovery, LinkEvent};
use proto::{Mode, Msg};
use slint::{ComponentHandle, Model, ModelRc, SharedString, Timer, TimerMode, VecModel};
use std::cell::RefCell;
use std::collections::HashMap;
use std::net::{IpAddr, SocketAddr};
use std::rc::Rc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const TICK: Duration = Duration::from_millis(33);
/// The map is redrawn at most this often (20 Hz).
const MAP_INTERVAL: Duration = Duration::from_millis(50);
/// The pane id of the output page's map.
const OUTPUT_MAP: i32 = 0;
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

/// "ip" or "ip:port" (default port `proto::PORT`). No DNS lookup: this runs on the UI thread.
pub fn parse_target(text: &str) -> Result<SocketAddr, String> {
    let t = text.trim();
    if t.is_empty() {
        return Err("IP アドレスを入力してください".into());
    }
    if let Ok(a) = t.parse::<SocketAddr>() {
        return Ok(a);
    }
    let t6 = t.trim_start_matches('[').trim_end_matches(']');
    if let Ok(ip) = t6.parse::<IpAddr>() {
        return Ok(SocketAddr::new(ip, proto::PORT));
    }
    Err(format!("IP アドレスで入力してください (ホスト名は使えません): {t}"))
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
    latch: StopLatch,
    notice: String,
    status: String,
    name: String,
    /// Per-pane map views, keyed by pane id, so panes zoom and pan independently.
    maps: HashMap<i32, PaneState>,
    last_render: Option<Instant>,
    /// Something the map shows changed since the last draw.
    scene_dirty: bool,
    was_visible: bool,
    bounds: [f64; 4],
    /// Manual page inputs; copied into `control` by `sync_manual`.
    manual: Manual,
    /// Requests sent from the manual page whose RESULT we still show: `(req, is_reset)`.
    manual_reqs: Vec<(u32, bool)>,
    manual_message: String,
    /// Last preset list given to the window, to avoid rebuilding the model every tick.
    presets_shown: Vec<(String, bool)>,
}

impl Glue {
    fn scale_factor(&self) -> f32 {
        self.window.upgrade().map_or(1.0, |w| w.window().scale_factor())
    }

    fn pane(&mut self, id: i32) -> &mut PaneState {
        self.maps.entry(id).or_default()
    }

    fn map_geometry(&mut self, id: i32, w: f32, h: f32) {
        let sf = self.scale_factor();
        let bounds = self.bounds;
        let (pw, ph) = ((w * sf).round().max(1.0) as u32, (h * sf).round().max(1.0) as u32);
        self.pane(id).set_size(pw, ph, bounds);
    }

    fn map_panned(&mut self, id: i32, dx: f32, dy: f32) {
        let sf = self.scale_factor();
        self.pane(id).pan(dx * sf, dy * sf);
    }

    fn map_zoomed(&mut self, id: i32, factor: f32, x: f32, y: f32) {
        let sf = self.scale_factor();
        self.pane(id).zoom(factor, (x * sf, y * sf));
    }

    fn map_fit(&mut self, id: i32) {
        let bounds = self.bounds;
        self.pane(id).fit(bounds);
    }

    fn scene(&self, pane: &PaneState) -> Scene {
        let mut scene = Scene { pose_drag: pane.pose_drag, ..Scene::default() };
        if let Some(f) = self.model.field() {
            scene.bounds = Some(f.bounds);
            scene.obstacles = f.obstacles.clone();
            scene.shelf = Some(f.shelf);
            scene.robot_radius = f.robot_radius;
        }
        if let Some(s) = self.model.state() {
            scene.robot_pose = s.pose;
            if let Some((name, _)) = &s.path
                && let Some(pts) = self.model.path_shape(name)
            {
                scene.path = pts.iter().map(|&(x, y)| (f32::from(x) / 1000.0, f32::from(y) / 1000.0)).collect();
            }
        }
        scene.scan = self.model.scan().to_vec();
        scene
    }

    /// Redraws the visible map panes when something changed, at most at 20 Hz.
    fn render_maps(&mut self, events: bool) {
        let Some(w) = self.window.upgrade() else { return };
        let visible = w.get_connected() && w.get_page() == 0;
        if visible && !self.was_visible {
            self.scene_dirty = true;
        }
        self.was_visible = visible;
        if !visible {
            return;
        }
        let bounds = map::field_bounds(self.model.field().map(|f| f.bounds));
        if bounds != self.bounds {
            self.bounds = bounds;
            for p in self.maps.values_mut().filter(|p| !p.user_moved) {
                p.fit(bounds);
            }
        }
        self.scene_dirty |= events;
        let any_dirty = self.scene_dirty || self.maps.values().any(|p| p.dirty);
        if !any_dirty || self.last_render.is_some_and(|t| t.elapsed() < MAP_INTERVAL) {
            return;
        }
        let sf = w.window().scale_factor();
        if let Some(p) = self.maps.get(&OUTPUT_MAP)
            && p.size.0 > 0
            && p.size.1 > 0
        {
            let buf = map::render(&p.view, p.size.0, p.size.1, &self.scene(p));
            w.set_map_image(slint::Image::from_rgba8_premultiplied(buf));
            w.set_map_cx(p.view.center.0);
            w.set_map_cy(p.view.center.1);
            w.set_map_scale(p.view.scale / sf);
        }
        self.scene_dirty = false;
        self.last_render = Some(Instant::now());
        for p in self.maps.values_mut() {
            p.dirty = false;
        }
    }

    fn refresh_band(&self, w: &MainWindow) {
        let red = slint::Color::from_rgb_u8(0xd1, 0x34, 0x2f);
        let Some(s) = self.model.state() else {
            w.set_band_pose("位置 -".into());
            w.set_band_vel("速度 -".into());
            w.set_band_wheels(ModelRc::default());
            w.set_band_mode("".into());
            w.set_band_reason("".into());
            return;
        };
        w.set_band_pose(match s.pose {
            Some([x, y, yaw]) => format!("位置 x {x:.2}  y {y:.2} m   向き {:.0}°", yaw.to_degrees()),
            None => "位置 -".to_owned(),
        }.into());
        w.set_band_vel(match s.vel {
            Some([vx, vy, wz]) => format!("速度 {vx:.2}, {vy:.2} m/s   旋回 {wz:.2} rad/s"),
            None => "速度 -".to_owned(),
        }.into());
        let wheels: Vec<WheelInfo> = s
            .wheels
            .iter()
            .map(|wh| WheelInfo { text: format!("{:.1} V", wh.vdc).into(), ok: wh.ok })
            .collect();
        w.set_band_wheels(ModelRc::from(Rc::new(VecModel::from(wheels))));
        let (mode, color) = match s.mode {
            Mode::Manual => ("手動", slint::Color::from_rgb_u8(0x3f, 0xb9, 0x50)),
            Mode::Auto => ("自動", slint::Color::from_rgb_u8(0x3d, 0x8b, 0xfd)),
            Mode::Stopped => ("停止", red),
        };
        w.set_band_mode(mode.into());
        w.set_band_mode_color(color);
        w.set_band_reason(if s.mode == Mode::Stopped && !s.stop_reason.is_empty() {
            format!("理由: {}", s.stop_reason)
        } else {
            String::new()
        }.into());
    }

    fn levels(&self) -> [(f64, f64); 3] {
        self.model.config().map_or(manual::DEFAULT_LEVELS, |c| c.speed_levels)
    }

    /// Motion may be sent only while the manual page is showing on a live link and the robot
    /// is not running something automatic.
    fn manual_allowed(&self) -> bool {
        let Some(w) = self.window.upgrade() else { return false };
        w.get_connected()
            && self.client.is_some()
            && w.get_page() == 1
            && self.model.state().is_some_and(|s| s.mode != Mode::Auto)
    }

    /// Copies the manual inputs into `control` (never touching `estop`) and pushes on change.
    /// When the page may not drive the robot, the held inputs are dropped, so they cannot
    /// resume by themselves when it may again.
    fn sync_manual(&mut self) {
        let allowed = self.manual_allowed();
        if !allowed {
            self.manual.release_all();
        }
        let levels = self.levels();
        if self.manual.apply(&mut self.control, &levels, allowed) {
            self.push_control();
        }
    }

    fn manual_stick(&mut self, dx: f32, dy: f32) {
        if self.manual_allowed() {
            self.manual.set_stick(dx, dy);
        }
        self.sync_manual();
    }

    fn manual_slider(&mut self, s: f32) {
        if self.manual_allowed() {
            self.manual.set_slider(s);
        }
        self.sync_manual();
    }

    fn manual_jog(&mut self, side: i32, dir: i32) {
        if self.manual_allowed() {
            self.manual.jog_press(side.clamp(0, 1) as u8, dir.clamp(-1, 1) as i8);
        }
        self.sync_manual();
    }

    fn manual_jog_released(&mut self) {
        self.manual.jog_release();
        self.sync_manual();
    }

    fn manual_options(&mut self) {
        let Some(w) = self.window.upgrade() else { return };
        self.manual.level = w.get_manual_level().clamp(0, 2) as usize;
        self.manual.field_frame = w.get_manual_field_frame();
        self.manual.both = w.get_manual_both();
        self.sync_manual();
    }

    /// Sends a request from the manual page and remembers it to show its RESULT.
    fn manual_send(&mut self, is_reset: bool, build: impl FnOnce(u32) -> Msg) {
        let Some(c) = &self.client else { return };
        let req = c.next_req();
        c.send(build(req));
        self.manual_reqs.push((req, is_reset));
        self.manual_message.clear();
    }

    fn manual_preset(&mut self, side: i32, idx: i32) {
        if !self.manual_allowed() {
            return;
        }
        let Some(cfg) = self.model.config() else { return };
        let Some((_, h)) = usize::try_from(idx).ok().and_then(|i| cfg.lift_presets.get(i)) else { return };
        let h = *h;
        if !manual::preset_enabled(h, cfg.lift_max) {
            return;
        }
        let gripper = manual::gripper_id(side.clamp(0, 1) as u8, self.manual.both);
        self.manual_send(false, |req| Msg::LiftTo { req, gripper, height: h });
    }

    fn manual_suction(&mut self, side: i32) {
        if !self.manual_allowed() {
            return;
        }
        let side = side.clamp(0, 1) as u8;
        let gripper = manual::gripper_id(side, self.manual.both);
        let on_of = |i: usize| self.model.state().and_then(|s| s.grippers[i]).is_some_and(|g| g.suction);
        let on = match gripper {
            2 => !(on_of(0) && on_of(1)),
            i => !on_of(usize::from(i)),
        };
        self.manual_send(false, |req| Msg::SetSuction { req, gripper, on });
    }

    fn lift_moving(&self) -> bool {
        self.model.state().is_some_and(|s| s.grippers.iter().flatten().any(|g| g.moving))
    }

    fn manual_reset(&mut self) {
        // The button is held for a second, during which a lift may have started to move.
        if !self.manual_allowed() || self.lift_moving() {
            return;
        }
        self.manual_send(true, |req| Msg::LiftZero { req, gripper: 2 });
    }

    fn manual_cancel(&mut self) {
        self.manual_send(false, |req| Msg::Cancel { req });
    }

    /// Shows the RESULT of manual-page requests: failures, and the reset outcome.
    fn collect_manual_results(&mut self) {
        let results = &self.model.results;
        let message = &mut self.manual_message;
        self.manual_reqs.retain(|&(req, is_reset)| {
            let Some((_, ok, msg)) = results.iter().find(|r| r.0 == req) else { return true };
            if !ok {
                *message = if msg.is_empty() { "失敗しました".to_owned() } else { msg.clone() };
            } else if is_reset {
                *message = if msg.is_empty() { "0 点にしました".to_owned() } else { msg.clone() };
            }
            false
        });
    }

    fn refresh_manual(&mut self, w: &MainWindow) {
        let state = self.model.state();
        let grippers = state.map_or([None, None], |s| s.grippers);
        w.set_manual_left_info(manual::height_text(grippers[0].as_ref()).into());
        w.set_manual_right_info(manual::height_text(grippers[1].as_ref()).into());
        w.set_manual_left_suction(grippers[0].is_some_and(|g| g.suction));
        w.set_manual_right_suction(grippers[1].is_some_and(|g| g.suction));
        w.set_manual_reset_enabled(!self.lift_moving());
        let (lv, wv) = self.manual.level_pair(&self.levels());
        w.set_manual_speed_text(format!("並進 {lv:.1} m/s  旋回 {wv:.1} rad/s").into());
        w.set_manual_auto_label(
            manual::auto_label(state.and_then(|s| s.task.as_ref()), state.and_then(|s| s.path.as_ref())).into(),
        );
        w.set_manual_message(self.manual_message.as_str().into());
        let (presets, reason) = match self.model.config() {
            Some(c) => {
                let p: Vec<(String, bool)> =
                    c.lift_presets.iter().map(|(n, h)| (n.clone(), manual::preset_enabled(*h, c.lift_max))).collect();
                let reason = if p.iter().any(|(_, ok)| !ok) { manual::lift_limit_reason(c.lift_max) } else { String::new() };
                (p, reason)
            }
            None => (Vec::new(), String::new()),
        };
        w.set_manual_lift_reason(reason.into());
        if presets != self.presets_shown {
            let items: Vec<PresetInfo> =
                presets.iter().map(|(n, ok)| PresetInfo { name: n.as_str().into(), enabled: *ok }).collect();
            w.set_manual_presets(ModelRc::from(Rc::new(VecModel::from(items))));
            self.presets_shown = presets;
        }
    }

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
        self.latch.on_link_reset();
        // The emergency-stop latch survives reconnects; motion inputs restart from zero.
        self.control = ControlInput { estop: self.latch.estop(), ..ControlInput::default() };
        self.manual.release_all();
        self.manual_reqs.clear();
        self.manual_message.clear();
        let (client, rx) = Client::start(addr, self.name.clone(), takeover);
        self.client = Some(client);
        self.push_control();
        self.rx = Some(rx);
        self.status = format!("{addr} に接続中…");
    }

    fn press_stop(&mut self) {
        self.latch.press();
        self.control.estop = self.latch.estop();
        self.notice.clear();
        self.push_control();
    }

    fn press_release(&mut self) {
        let Some(c) = &self.client else { return };
        let req = c.next_req();
        self.latch.release_sent(req);
        self.control.estop = self.latch.estop();
        self.notice.clear();
        self.push_control();
        if let Some(c) = &self.client {
            c.send(Msg::ReleaseStop { req });
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
        let got_events = !events.is_empty();
        for ev in events {
            if matches!(ev, LinkEvent::Rejected { .. } | LinkEvent::Bye { .. }) {
                // The client thread idles after these; release it so Connect starts a fresh one.
                self.client = None;
                self.rx = None;
                self.latch.on_link_reset();
                self.control.estop = self.latch.estop();
            }
            self.model.apply(ev);
        }
        if let Some(req) = self.latch.pending()
            && let Some((_, ok, msg)) = self.model.results.iter().find(|r| r.0 == req).cloned()
        {
            self.latch.on_result(req, ok);
            self.control.estop = self.latch.estop();
            self.push_control();
            if !ok {
                self.notice = format!("解除できません: {msg}");
            }
        }
        self.collect_manual_results();
        self.sync_manual();
        self.refresh();
        if let Some(w) = self.window.upgrade()
            && w.get_connected()
            && w.get_page() == 1
        {
            self.refresh_manual(&w);
        }
        self.render_maps(got_events);
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
            ConnState::TakenOver(_) => (false, "乗っ取られました", slint::Color::from_rgb_u8(0xd1, 0x34, 0x2f), -1),
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
        if connected && w.get_page() == 0 {
            self.refresh_band(&w);
        }
        let (status, status_color) = match &conn {
            ConnState::TakenOver(r) => (format!("別のスマホに乗っ取られました: {r}"), slint::Color::from_rgb_u8(0xd1, 0x34, 0x2f)),
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
        latch: StopLatch::default(),
        notice: String::new(),
        status,
        name: client_name(),
        maps: HashMap::new(),
        last_render: None,
        scene_dirty: true,
        was_visible: false,
        bounds: map::field_bounds(None),
        manual: Manual::default(),
        manual_reqs: Vec::new(),
        manual_message: String::new(),
        presets_shown: Vec::new(),
    }));
    {
        let g = glue.clone();
        window.on_connect(move |t, takeover| {
            let mut g = g.borrow_mut();
            g.connect(&t, takeover);
            g.refresh();
        });
        let g = glue.clone();
        window.on_map_geometry(move |id, w, h| {
            let mut g = g.borrow_mut();
            g.map_geometry(id, w, h);
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_map_panned(move |id, dx, dy| {
            let mut g = g.borrow_mut();
            g.map_panned(id, dx, dy);
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_map_zoomed(move |id, f, x, y| {
            let mut g = g.borrow_mut();
            g.map_zoomed(id, f, x, y);
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_map_fit(move |id| {
            let mut g = g.borrow_mut();
            g.map_fit(id);
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_manual_stick(move |x, y| g.borrow_mut().manual_stick(x, y));
        let g = glue.clone();
        window.on_manual_slider(move |s| g.borrow_mut().manual_slider(s));
        let g = glue.clone();
        window.on_manual_jog(move |side, dir| g.borrow_mut().manual_jog(side, dir));
        let g = glue.clone();
        window.on_manual_jog_released(move || g.borrow_mut().manual_jog_released());
        let g = glue.clone();
        window.on_manual_options_changed(move || g.borrow_mut().manual_options());
        let g = glue.clone();
        window.on_manual_preset(move |side, i| g.borrow_mut().manual_preset(side, i));
        let g = glue.clone();
        window.on_manual_suction(move |side| g.borrow_mut().manual_suction(side));
        let g = glue.clone();
        window.on_manual_reset(move || g.borrow_mut().manual_reset());
        let g = glue.clone();
        window.on_manual_cancel_auto(move || g.borrow_mut().manual_cancel());
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
        assert!(parse_target("robot.local").is_err());
        assert!(parse_target("::1").unwrap().is_ipv6());
    }

    #[test]
    fn client_names_look_right_and_differ() {
        let a = new_client_name();
        assert!(a.starts_with("phone-") && a.len() == 14, "{a}");
        assert_ne!(a, new_client_name());
    }
}
