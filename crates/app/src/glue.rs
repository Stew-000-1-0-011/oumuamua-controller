//! Connects the Slint window to the link: callbacks in, a 33 ms timer out.

use crate::auto;
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
/// The pane id of the auto page's map (the one that picks poses).
const AUTO_MAP: i32 = 1;
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
    /// The pane drawn on the last render pass, `None` when no map is showing.
    shown_pane: Option<i32>,
    bounds: [f64; 4],
    /// Manual page inputs; copied into `control` by `sync_manual`.
    manual: Manual,
    /// Requests sent from the manual page whose RESULT we still show: `(req, is_reset, sent)`.
    manual_reqs: Vec<(u32, bool, Instant)>,
    /// When the reset button was last pressed down.
    reset_pressed_at: Option<Instant>,
    manual_message: String,
    /// Last preset list given to the window, to avoid rebuilding the model every tick.
    presets_shown: Vec<(String, bool)>,
    /// Requests sent from the auto page whose RESULT we show: `(req, label, sent)`.
    auto_reqs: Vec<(u32, String, Instant)>,
    /// When the 忘れさせる button was last pressed down.
    forget_pressed_at: Option<Instant>,
    /// The bottom line of the auto page and whether it is good news.
    auto_message: (String, bool),
    /// A picked pose awaiting 決定: `(x, y, yaw)`.
    pending_pose: Option<(f64, f64, f64)>,
    /// The RUN_TASK / START_PATH awaiting its RESULT or its appearance in STATE:
    /// `(req, sent, accepted_at)`.
    start_pending: Option<(u32, Instant, Option<Instant>)>,
    /// `(kind, args)` of the last RUN_TASK sent, for naming its TASK_DONE.
    last_sent_task: Option<(u8, [i32; 4])>,
    /// The task last seen running, to name its TASK_DONE.
    last_task: Option<(u8, [i32; 4])>,
    /// How many TASK_DONE have been shown.
    done_seen: u64,
    /// Last lists given to the window: blocks, cells, paths.
    auto_shown: (Vec<i32>, Vec<i32>, Vec<String>),
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

    /// Redraws the map that is showing (output page: pane 0, auto page: pane 1) when something
    /// changed, at most at 20 Hz.
    fn render_maps(&mut self, events: bool) {
        let Some(w) = self.window.upgrade() else { return };
        let pane_id = match (w.get_connected(), w.get_page()) {
            (true, 0) => Some(OUTPUT_MAP),
            (true, 2) => Some(AUTO_MAP),
            _ => None,
        };
        if pane_id != self.shown_pane {
            self.scene_dirty = true;
        }
        self.shown_pane = pane_id;
        let Some(pane_id) = pane_id else { return };
        let bounds = map::field_bounds(self.model.field().map(|f| f.bounds));
        if bounds != self.bounds {
            self.bounds = bounds;
            for p in self.maps.values_mut().filter(|p| !p.user_moved) {
                p.fit(bounds);
            }
        }
        self.scene_dirty |= events;
        let any_dirty = self.scene_dirty || self.maps.get(&pane_id).is_some_and(|p| p.dirty);
        if !any_dirty || self.last_render.is_some_and(|t| t.elapsed() < MAP_INTERVAL) {
            return;
        }
        let sf = w.window().scale_factor();
        if let Some(p) = self.maps.get(&pane_id)
            && p.size.0 > 0
            && p.size.1 > 0
        {
            let buf = map::render(&p.view, p.size.0, p.size.1, &self.scene(p));
            let image = slint::Image::from_rgba8_premultiplied(buf);
            let (cx, cy, scale) = (p.view.center.0, p.view.center.1, p.view.scale / sf);
            if pane_id == OUTPUT_MAP {
                w.set_map_image(image);
                w.set_map_cx(cx);
                w.set_map_cy(cy);
                w.set_map_scale(scale);
            } else {
                w.set_auto_map_image(image);
                w.set_auto_map_cx(cx);
                w.set_auto_map_cy(cy);
                w.set_auto_map_scale(scale);
            }
        }
        self.scene_dirty = false;
        self.last_render = Some(Instant::now());
        if let Some(p) = self.maps.get_mut(&pane_id) {
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
        manual::motion_allowed(
            w.get_connected(),
            self.client.is_some(),
            w.get_page(),
            self.model.state().map(|s| s.mode),
        )
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
        self.manual_reqs.push((req, is_reset, Instant::now()));
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

    fn manual_reset_pressed(&mut self) {
        self.reset_pressed_at = Some(Instant::now());
    }

    fn manual_reset(&mut self) {
        // Only a confirmation that follows a real press counts.
        let pressed = self.reset_pressed_at.take();
        if !pressed.is_some_and(|t| t.elapsed() <= Duration::from_millis(manual::RESET_CONFIRM_WINDOW_MS)) {
            return;
        }
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
        let keep = Duration::from_millis(manual::REQ_KEEP_MS);
        self.manual_reqs.retain(|&(req, is_reset, sent)| {
            let Some((_, ok, msg)) = results.iter().find(|r| r.0 == req) else { return sent.elapsed() < keep };
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

    // ---- auto page ----

    fn link_live(&self) -> bool {
        self.client.is_some() && matches!(self.model.connection(), ConnState::Connected { .. })
    }

    /// Why task buttons are unusable, or `None`.
    fn auto_block_reason(&self) -> Option<String> {
        if !self.link_live() {
            return Some("接続が切れています".to_owned());
        }
        let s = self.model.state();
        if self.start_blocks() {
            return Some("作業中のため押せません".to_owned());
        }
        auto::task_block_reason(s.map(|s| s.mode), s.and_then(|s| s.task.as_ref()))
    }

    /// A RUN_TASK / START_PATH we sent is still unanswered or not yet visible in STATE.
    fn start_blocks(&self) -> bool {
        let Some((_, _, accepted)) = self.start_pending else { return false };
        let shown = self.model.state().is_some_and(|s| s.task.is_some() || s.path.is_some());
        auto::start_pending(accepted.is_none(), accepted.map(|t| t.elapsed().as_millis() as u64), shown)
    }

    /// Sends a request from the auto page and remembers it to show its RESULT.
    fn auto_send(&mut self, label: String, build: impl FnOnce(u32) -> Msg) -> Option<u32> {
        let c = self.client.as_ref()?;
        let req = c.next_req();
        c.send(build(req));
        self.auto_reqs.push((req, label, Instant::now()));
        Some(req)
    }

    fn auto_run(&mut self, kind: u8, args: [i32; 4]) {
        let [gripper, block, column, row] = args;
        let label = auto::task_label(kind, args);
        let req = self.auto_send(label, |req| Msg::RunTask { req, kind, gripper, block, column, row });
        if let Some(req) = req {
            self.start_pending = Some((req, Instant::now(), None));
            self.last_sent_task = Some((kind, args));
        }
    }

    fn auto_pick(&mut self, gripper: i32, block: i32) {
        let Some(b) = u8::try_from(block).ok() else { return };
        let mem = self.model.task_memory().cloned().unwrap_or_default();
        let count = self.model.config().map_or(auto::DEFAULT_BLOCKS, |c| c.blocks);
        if self.auto_block_reason().is_some() || b >= count || !auto::can_pick(&mem, b) || !(0..=1).contains(&gripper) {
            return;
        }
        self.auto_run(auto::TaskKind::Pick as u8, [gripper, block, 0, 0]);
    }

    fn auto_place(&mut self, gripper: i32, column: i32, row: i32) {
        let mem = self.model.task_memory().cloned().unwrap_or_default();
        let (cols, rows) = self.model.config().map_or((auto::DEFAULT_COLUMNS, auto::DEFAULT_ROWS), |c| (c.columns, c.rows));
        let holds = u8::try_from(gripper).ok().is_some_and(|g| auto::holding_gripper(&mem).contains(&g));
        if self.auto_block_reason().is_some()
            || !holds
            || !(0..i32::from(cols)).contains(&column)
            || !(0..i32::from(rows)).contains(&row)
        {
            return;
        }
        self.auto_run(auto::TaskKind::Place as u8, [gripper, 0, column, row]);
    }

    fn auto_move(&mut self, kind: i32) {
        use auto::TaskKind::*;
        let ok = [ToSlalomEntrance, ToStart, SlalomForward, SlalomBackward, ToSlalomExit].iter().any(|&k| i32::from(k as u8) == kind);
        if !ok || self.auto_block_reason().is_some() {
            return;
        }
        self.auto_run(kind as u8, [0; 4]);
    }

    fn auto_cancel(&mut self) {
        if self.link_live() {
            self.auto_send("取り消し".to_owned(), |req| Msg::Cancel { req });
        }
    }

    fn auto_start_path(&mut self, name: &str) {
        if self.auto_block_reason().is_some() || !self.model.config().is_some_and(|c| c.paths.iter().any(|p| p == name)) {
            return;
        }
        let name = name.to_owned();
        if let Some(req) = self.auto_send(format!("経路 {name}"), |req| Msg::StartPath { req, name }) {
            self.start_pending = Some((req, Instant::now(), None));
        }
    }

    fn auto_toggle_tracker(&mut self) {
        if !self.link_live() {
            return;
        }
        let on = !self.model.state().is_some_and(|s| s.tracker_enabled);
        self.auto_send(if on { "追従 入".to_owned() } else { "追従 切".to_owned() }, |req| Msg::SetTracker { req, on });
    }

    fn auto_forget_pressed(&mut self) {
        self.forget_pressed_at = Some(Instant::now());
    }

    fn auto_forget(&mut self) {
        let pressed = self.forget_pressed_at.take();
        if !pressed.is_some_and(|t| t.elapsed() <= Duration::from_millis(manual::RESET_CONFIRM_WINDOW_MS)) {
            return;
        }
        if self.auto_block_reason().is_some() {
            return;
        }
        self.auto_send("忘れさせる".to_owned(), |req| Msg::TaskReset { req });
    }

    fn auto_pose_dragging(&mut self, x: f32, y: f32, yaw: f32) {
        self.pending_pose = None;
        let p = self.pane(AUTO_MAP);
        p.pose_drag = Some(([f64::from(x), f64::from(y)], f64::from(yaw)));
        p.dirty = true;
    }

    fn auto_pose_picked(&mut self, x: f32, y: f32, yaw: f32) {
        self.auto_pose_dragging(x, y, yaw);
        self.pending_pose = Some((f64::from(x), f64::from(y), f64::from(yaw)));
    }

    fn auto_pose_clear(&mut self) {
        self.pending_pose = None;
        let p = self.pane(AUTO_MAP);
        p.pose_drag = None;
        p.dirty = true;
    }

    fn auto_pose_confirm(&mut self) {
        if let Some((x, y, yaw)) = self.pending_pose
            && self.link_live()
        {
            self.auto_send("自己位置".to_owned(), |req| Msg::SetPose { req, x, y, yaw });
        }
        self.auto_pose_clear();
    }

    /// Shows RESULTs of auto-page requests and TASK_DONE on the bottom line.
    fn collect_auto_results(&mut self) {
        if let Some((req, sent, accepted)) = &mut self.start_pending {
            match self.model.results.iter().find(|r| r.0 == *req) {
                Some((_, true, _)) => {
                    accepted.get_or_insert_with(Instant::now);
                }
                Some(_) => self.start_pending = None,
                None if sent.elapsed() >= Duration::from_millis(manual::REQ_KEEP_MS) => self.start_pending = None,
                None => {}
            }
        }
        if self.start_pending.is_some() && !self.start_blocks() && self.start_pending.is_some_and(|p| p.2.is_some()) {
            self.start_pending = None;
        }
        let results = &self.model.results;
        let message = &mut self.auto_message;
        let keep = Duration::from_millis(manual::REQ_KEEP_MS);
        self.auto_reqs.retain(|(req, label, sent)| {
            let Some((_, ok, msg)) = results.iter().find(|r| r.0 == *req) else { return sent.elapsed() < keep };
            *message = if *ok {
                (if msg.is_empty() { format!("{label}: 受け付けました") } else { format!("{label}: {msg}") }, true)
            } else {
                (auto::done_text(label, false, msg), false)
            };
            false
        });
        if let Some(t) = self.model.state().and_then(|s| s.task.as_ref()) {
            self.last_task = Some((t.kind, t.args));
        }
        let (count, done) = self.model.task_done();
        if count != self.done_seen {
            self.done_seen = count;
            if let Some((kind, ok, msg)) = done {
                let label = match (self.last_task, self.last_sent_task) {
                    (Some((k, args)), _) | (None, Some((k, args))) if k == *kind => auto::task_label(k, args),
                    (_, Some((k, args))) if k == *kind => auto::task_label(k, args),
                    _ => auto::task_label(*kind, [0; 4]),
                };
                self.auto_message = (auto::done_text(&label, *ok, msg), *ok);
                self.last_task = None;
            }
        }
    }

    fn refresh_auto(&mut self, w: &MainWindow) {
        let state = self.model.state();
        let mem = self.model.task_memory().cloned().unwrap_or_default();
        let (blocks, cols, rows, paths) = match self.model.config() {
            Some(c) => (c.blocks, c.columns, c.rows, c.paths.clone()),
            None => (auto::DEFAULT_BLOCKS, auto::DEFAULT_COLUMNS, auto::DEFAULT_ROWS, Vec::new()),
        };
        let reason = self.auto_block_reason();
        let task = state.and_then(|s| s.task.as_ref());
        w.set_auto_block_reason(reason.clone().unwrap_or_default().into());
        w.set_auto_task_active(task.is_some());
        w.set_auto_task_label(task.map_or(String::new(), |t| auto::task_label(t.kind, t.args)).into());
        w.set_auto_task_step(task.map_or("", |t| t.step.as_str()).into());
        w.set_auto_columns(i32::from(cols));
        w.set_auto_rows(i32::from(rows));
        w.set_auto_hold_left(mem.holding[0]);
        w.set_auto_hold_right(mem.holding[1]);
        if let Some(g) = auto::place_gripper(&mem, w.get_auto_place_gripper().clamp(0, 1) as u8)
            && i32::from(g) != w.get_auto_place_gripper()
        {
            w.set_auto_place_gripper(i32::from(g));
        }
        let (col, row) = (w.get_auto_column(), w.get_auto_row());
        let warning = match (u8::try_from(col), u8::try_from(row)) {
            (Ok(c), Ok(r)) => auto::place_warning(&mem, c, r),
            _ => None,
        };
        w.set_auto_place_warning(warning.unwrap_or_default().into());
        w.set_auto_memory_text(auto::memory_text(&mem).into());
        w.set_auto_forget_enabled(reason.is_none());
        w.set_auto_active_path(state.and_then(|s| s.path.as_ref()).map_or("", |(n, _)| n.as_str()).into());
        w.set_auto_tracker(state.is_some_and(|s| s.tracker_enabled));
        w.set_auto_message(self.auto_message.0.as_str().into());
        w.set_auto_message_ok(self.auto_message.1);
        w.set_auto_confirm_active(self.pending_pose.is_some());
        if let Some((x, y, yaw)) = self.pending_pose {
            w.set_auto_confirm_text(auto::pose_text(x, y, yaw).into());
        }
        let block_states: Vec<i32> = (0..blocks).map(|b| i32::from(!auto::can_pick(&mem, b))).collect();
        let mut cells = vec![0; usize::from(cols) * usize::from(rows)];
        for &(c, r) in &mem.placed {
            if let Some(i) = usize::from(r).checked_mul(usize::from(cols)).map(|v| v + usize::from(c)).filter(|_| c < cols && r < rows) {
                cells[i] = 1;
            }
        }
        if block_states != self.auto_shown.0 {
            w.set_auto_blocks(ModelRc::from(Rc::new(VecModel::from(block_states.clone()))));
            self.auto_shown.0 = block_states;
        }
        if cells != self.auto_shown.1 {
            w.set_auto_cells(ModelRc::from(Rc::new(VecModel::from(cells.clone()))));
            self.auto_shown.1 = cells;
        }
        if paths != self.auto_shown.2 {
            let items: Vec<SharedString> = paths.iter().map(|p| p.as_str().into()).collect();
            w.set_auto_paths(ModelRc::from(Rc::new(VecModel::from(items))));
            self.auto_shown.2 = paths;
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
        self.auto_reqs.clear();
        self.auto_message = (String::new(), true);
        self.pending_pose = None;
        self.pane(AUTO_MAP).pose_drag = None;
        self.last_task = None;
        self.start_pending = None;
        self.last_sent_task = None;
        self.done_seen = 0;
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
        self.collect_auto_results();
        self.sync_manual();
        self.refresh();
        if let Some(w) = self.window.upgrade()
            && w.get_connected()
            && w.get_page() == 1
        {
            self.refresh_manual(&w);
        }
        if let Some(w) = self.window.upgrade()
            && w.get_connected()
            && w.get_page() == 2
        {
            self.refresh_auto(&w);
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
        shown_pane: None,
        bounds: map::field_bounds(None),
        manual: Manual::default(),
        manual_reqs: Vec::new(),
        reset_pressed_at: None,
        manual_message: String::new(),
        presets_shown: Vec::new(),
        auto_reqs: Vec::new(),
        forget_pressed_at: None,
        auto_message: (String::new(), true),
        pending_pose: None,
        start_pending: None,
        last_sent_task: None,
        last_task: None,
        done_seen: 0,
        auto_shown: (Vec::new(), Vec::new(), Vec::new()),
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
        window.on_manual_reset_pressed(move || g.borrow_mut().manual_reset_pressed());
        let g = glue.clone();
        window.on_manual_reset(move || g.borrow_mut().manual_reset());
        let g = glue.clone();
        window.on_manual_cancel_auto(move || g.borrow_mut().manual_cancel());
        let g = glue.clone();
        window.on_auto_run_pick(move |gr, b| g.borrow_mut().auto_pick(gr, b));
        let g = glue.clone();
        window.on_auto_run_place(move |gr, c, r| g.borrow_mut().auto_place(gr, c, r));
        let g = glue.clone();
        window.on_auto_run_move(move |k| g.borrow_mut().auto_move(k));
        let g = glue.clone();
        window.on_auto_cancel(move || g.borrow_mut().auto_cancel());
        let g = glue.clone();
        window.on_auto_forget_pressed(move || g.borrow_mut().auto_forget_pressed());
        let g = glue.clone();
        window.on_auto_forget(move || g.borrow_mut().auto_forget());
        let g = glue.clone();
        window.on_auto_start_path(move |n| g.borrow_mut().auto_start_path(&n));
        let g = glue.clone();
        window.on_auto_stop_path(move || g.borrow_mut().auto_cancel());
        let g = glue.clone();
        window.on_auto_toggle_tracker(move || g.borrow_mut().auto_toggle_tracker());
        let g = glue.clone();
        window.on_auto_pose_dragging(move |x, y, yaw| {
            let mut g = g.borrow_mut();
            g.auto_pose_dragging(x, y, yaw);
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_auto_pose_picked(move |x, y, yaw| {
            let mut g = g.borrow_mut();
            g.auto_pose_picked(x, y, yaw);
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_auto_pose_cancelled(move || {
            let mut g = g.borrow_mut();
            g.auto_pose_clear();
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_auto_pose_dismiss(move || {
            let mut g = g.borrow_mut();
            g.auto_pose_clear();
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_auto_pose_confirm(move || {
            let mut g = g.borrow_mut();
            g.auto_pose_confirm();
            g.render_maps(false);
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
