//! Connects the Slint window to the link: callbacks in, a 33 ms timer out.

use crate::auto;
use crate::console::{self, Console, PromptKind};
use crate::manual::{self, Manual};
use crate::model::{end_session, AppModel, ConnState, DemoSlot, StopLatch};
use crate::map::{self, PaneState, Scene};
use crate::{Beacon, ConsoleRow, ConsoleSpan, ConsoleState, ConsoleTabInfo, MainWindow, PresetInfo, WheelInfo, storage};
use link::{Client, ControlInput, Discovery, LinkEvent};
use proto::{Mode, Msg};
use term::{ModState, Modifier, Row};
use slint::{ComponentHandle, Model, ModelRc, SharedString, Timer, TimerMode, VecModel};
use std::cell::RefCell;
use std::collections::HashMap;
use std::net::{IpAddr, SocketAddr};
use std::rc::Rc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const TICK: Duration = Duration::from_millis(33);
/// A robot whose beacon has not been heard for this long leaves the list (it sends 1 per second).
const BEACON_TTL: Duration = Duration::from_secs(5);
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
    /// The in-app demo robot, alive only while we are connected to it.
    demo: DemoSlot<demo::DemoRobot>,
    /// Address of the robot the console state belongs to.
    link_addr: Option<SocketAddr>,
    rx: Option<Receiver<LinkEvent>>,
    _discovery: Option<Discovery>,
    beacons_rx: Option<link::discovery::Beacons>,
    beacons: Vec<(String, SocketAddr, Instant)>,
    beacons_dirty: std::cell::Cell<bool>,
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
    console: Console,
    console_rows: Rc<VecModel<ConsoleRow>>,
    /// The rows now in `console_rows`, to update only what changed.
    console_shown: Vec<Row>,
    /// `(active tab, revision)` of the rows shown, and when they were last rebuilt.
    console_rev: Option<(usize, u64)>,
    console_built: Option<Instant>,
    /// Characters per output row, from the width of the output area.
    console_cols: usize,
    console_tabs_shown: Vec<(String, bool)>,
    console_quick_shown: Vec<String>,
    console_cands_shown: Vec<String>,
    /// Shell requests whose RESULT may undo a local change: `(req, what, sent)`.
    shell_reqs: Vec<(u32, ShellReq, Instant)>,
    /// When the tabs not reported by the robot after a (re)connect are dropped.
    sync_deadline: Option<Instant>,
    /// When the histories are written (debounce).
    persist_at: Option<Instant>,
}

/// How long after CONNECTED the robot's SHELL_OPENED are awaited.
const SYNC_WAIT: Duration = Duration::from_millis(1500);
/// Console rows are rebuilt at most this often.
const CONSOLE_INTERVAL: Duration = Duration::from_millis(100);
const PERSIST_DELAY: Duration = Duration::from_secs(1);
/// Width of one output cell in logical pixels at 14 px text (DejaVu Sans Mono is 8.43).
const CELL_PX: f32 = 8.5;
const LOG_LEVELS: [u8; 4] = [10, 20, 30, 40];

#[derive(Debug, Clone, PartialEq)]
enum ShellReq {
    Open(String),
    Rename { old: String, new: String },
    /// Input, completion or close: only a failure is reported.
    Other(&'static str),
}

fn slint_color(c: Option<term::Color>, default: (u8, u8, u8)) -> slint::Color {
    let (r, g, b) = c.map_or(default, console::palette);
    slint::Color::from_rgb_u8(r, g, b)
}

fn console_row(r: &Row) -> ConsoleRow {
    match r {
        Row::Folded { block, hidden } => ConsoleRow {
            folded: true,
            hidden: *hidden as i32,
            block: *block as i32,
            spans: ModelRc::default(),
        },
        Row::Line(l) => {
            let spans: Vec<ConsoleSpan> = l
                .spans
                .iter()
                .map(|s| ConsoleSpan {
                    text: s.text.as_str().into(),
                    fg: slint_color(s.fg, (0xe8, 0xed, 0xf3)),
                    bg: slint_color(s.bg, (0, 0, 0)),
                    has_bg: s.bg.is_some(),
                    bold: s.bold,
                })
                .collect();
            ConsoleRow { folded: false, hidden: 0, block: 0, spans: ModelRc::from(Rc::new(VecModel::from(spans))) }
        }
    }
}

/// Drops beacons not heard within BEACON_TTL; true if any was dropped.
fn expire_beacons(list: &mut Vec<(String, SocketAddr, Instant)>, now: Instant) -> bool {
    let before = list.len();
    list.retain(|b| now.saturating_duration_since(b.2) < BEACON_TTL);
    list.len() != before
}

fn show_keyboard(g: &Glue) {
    if let Some(w) = g.window.upgrade() {
        w.global::<ConsoleState>().set_kbd_shown(true);
    }
}

fn mod_int(s: ModState) -> i32 {
    match s {
        ModState::Off => 0,
        ModState::Once => 1,
        ModState::Locked => 2,
    }
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

    /// Redraws the output map (pane 0, always under the manual and auto overlays too) when
    /// something changed, at most at 20 Hz.
    fn render_maps(&mut self, events: bool) {
        let Some(w) = self.window.upgrade() else { return };
        let pane_id = w.get_connected().then_some(OUTPUT_MAP);
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
            w.set_map_image(image);
            w.set_map_cx(cx);
            w.set_map_cy(cy);
            w.set_map_scale(scale);
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
            Some([x, y, yaw]) => format!("位置 {x:.2}, {y:.2} m  {:.0}°", yaw.to_degrees()),
            None => "位置 -".to_owned(),
        }.into());
        w.set_band_vel(match s.vel {
            Some([vx, vy, wz]) => format!("速度 {vx:.2}, {vy:.2}  旋回 {wz:.2}"),
            None => "速度 -".to_owned(),
        }.into());
        let wheels: Vec<WheelInfo> = s
            .wheels
            .iter()
            .map(|wh| WheelInfo { text: format!("{:.1}V", wh.vdc).into(), ok: wh.ok })
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

    /// The robot refuses lift motion, suction-on, reset and tracker-on while stopped.
    fn actuation_blocked(&self) -> bool {
        manual::actuation_blocked(self.model.state().map(|s| s.mode), self.control.estop)
    }

    /// The window lost focus (or the app was paused): nothing may stay held.
    fn window_deactivated(&mut self) {
        self.manual.release_all();
        self.sync_manual();
        self.push_control();
    }

    fn manual_jog(&mut self, side: i32, dir: i32) {
        if self.manual_allowed() && !self.actuation_blocked() {
            self.manual.jog_press(side.clamp(0, 1) as u8, dir.clamp(-1, 1) as i8);
        }
        self.sync_manual();
    }

    fn manual_jog_released(&mut self, side: i32) {
        self.manual.jog_release(side.clamp(0, 1) as u8);
        self.sync_manual();
    }

    fn manual_options(&mut self) {
        let Some(w) = self.window.upgrade() else { return };
        self.manual.level = w.get_manual_level().clamp(0, 2) as usize;
        self.manual.field_frame = w.get_manual_field_frame();
        self.manual.set_both(w.get_manual_both());
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
        if !self.manual_allowed() || self.actuation_blocked() {
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
        if on && self.actuation_blocked() {
            return;
        }
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
        if !self.manual_allowed() || self.lift_moving() || self.actuation_blocked() {
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
        let mut logs: Vec<(u8, String)> = Vec::new();
        self.manual_reqs.retain(|&(req, is_reset, sent)| {
            let Some((_, ok, msg)) = results.iter().find(|r| r.0 == req) else { return sent.elapsed() < keep };
            if !ok {
                *message = if msg.is_empty() { "失敗しました".to_owned() } else { msg.clone() };
                logs.push((40, message.clone()));
            } else if is_reset {
                *message = if msg.is_empty() { "0 点にしました".to_owned() } else { msg.clone() };
                logs.push((20, message.clone()));
            }
            false
        });
        for (level, text) in logs {
            self.console.log_local(level, "manual", &text);
        }
    }

    fn refresh_manual(&mut self, w: &MainWindow) {
        let state = self.model.state();
        let grippers = state.map_or([None, None], |s| s.grippers);
        w.set_manual_left_info(manual::height_text(grippers[0].as_ref()).into());
        w.set_manual_right_info(manual::height_text(grippers[1].as_ref()).into());
        w.set_manual_left_suction(grippers[0].is_some_and(|g| g.suction));
        w.set_manual_right_suction(grippers[1].is_some_and(|g| g.suction));
        w.set_manual_reset_enabled(!self.lift_moving() && !self.actuation_blocked());
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
        if on && self.actuation_blocked() {
            return;
        }
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
        let p = self.pane(OUTPUT_MAP);
        p.pose_drag = Some(([f64::from(x), f64::from(y)], f64::from(yaw)));
        p.dirty = true;
    }

    fn auto_pose_picked(&mut self, x: f32, y: f32, yaw: f32) {
        self.auto_pose_dragging(x, y, yaw);
        self.pending_pose = Some((f64::from(x), f64::from(y), f64::from(yaw)));
    }

    fn auto_pose_clear(&mut self) {
        self.pending_pose = None;
        let p = self.pane(OUTPUT_MAP);
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

    /// Shows the picked pose awaiting 決定 on the output screen (it is not tied to the auto page).
    fn sync_pose_confirm(&self) {
        let Some(w) = self.window.upgrade() else { return };
        w.set_auto_confirm_active(self.pending_pose.is_some());
        if let Some((x, y, yaw)) = self.pending_pose {
            w.set_auto_confirm_text(auto::pose_text(x, y, yaw).into());
        }
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
        let mut logs: Vec<(u8, String)> = Vec::new();
        self.auto_reqs.retain(|(req, label, sent)| {
            let Some((_, ok, msg)) = results.iter().find(|r| r.0 == *req) else { return sent.elapsed() < keep };
            *message = if *ok {
                (if msg.is_empty() { format!("{label}: 受け付けました") } else { format!("{label}: {msg}") }, true)
            } else {
                (auto::done_text(label, false, msg), false)
            };
            logs.push((if message.1 { 20 } else { 40 }, message.0.clone()));
            false
        });
        for (level, text) in logs {
            self.console.log_local(level, "auto", &text);
        }
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
                self.console.log_local(if *ok { 20 } else { 40 }, "auto", &self.auto_message.0);
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

    // ---- console ----

    /// Stamps request ids on console messages and sends them.
    fn console_send(&mut self, msgs: Vec<Msg>) {
        let Some(c) = &self.client else { return };
        for m in msgs {
            let req = c.next_req();
            let m = console::stamp(m, req);
            match &m {
                Msg::ShellOpen { tab, .. } => self.shell_reqs.push((req, ShellReq::Open(tab.clone()), Instant::now())),
                Msg::ShellRename { tab, new_name, .. } => self
                    .shell_reqs
                    .push((req, ShellReq::Rename { old: tab.clone(), new: new_name.clone() }, Instant::now())),
                Msg::ShellComplete { tab, .. } => {
                    self.console.note_complete_req(tab, req);
                    self.shell_reqs.push((req, ShellReq::Other("補完"), Instant::now()));
                }
                Msg::ShellInput { .. } => self.shell_reqs.push((req, ShellReq::Other("入力"), Instant::now())),
                Msg::ShellClose { .. } => self.shell_reqs.push((req, ShellReq::Other("閉じる"), Instant::now())),
                _ => {}
            }
            c.send(m);
        }
    }

    fn console_do(&mut self, f: impl FnOnce(&mut Console) -> Vec<Msg>) {
        let msgs = f(&mut self.console);
        self.console_send(msgs);
        self.refresh_console();
    }

    fn console_set_level(&mut self, idx: i32) {
        let Some(&level) = usize::try_from(idx).ok().and_then(|i| LOG_LEVELS.get(i)) else { return };
        let m = self.console.set_log_filter(level);
        self.console_send(vec![m]);
        self.refresh_console();
    }

    /// Shell events from the robot, the end of the reconnect wait, RESULTs of tab requests and
    /// debounced saving.
    fn poll_console(&mut self, connected_now: bool) {
        if connected_now {
            self.console.begin_sync();
            self.sync_deadline = Some(Instant::now() + SYNC_WAIT);
            let m = self.console.set_log_filter(self.console.log_filter());
            self.console_send(vec![m]);
        }
        for ev in self.model.take_shell_events() {
            self.console.on_event(ev);
        }
        if self.sync_deadline.is_some_and(|t| Instant::now() >= t) {
            self.sync_deadline = None;
            self.console.end_sync();
        }
        let results = &self.model.results;
        let console = &mut self.console;
        let keep = Duration::from_millis(manual::REQ_KEEP_MS);
        self.shell_reqs.retain(|(req, what, sent)| {
            let Some((_, ok, msg)) = results.iter().find(|r| r.0 == *req) else { return sent.elapsed() < keep };
            if !ok {
                let why = if msg.is_empty() { "失敗しました" } else { msg.as_str() };
                match what {
                    ShellReq::Open(tab) => {
                        console.forget_tab(tab);
                        console.log_local(40, "console", &format!("タブ {tab} を開けません: {why}"));
                    }
                    ShellReq::Rename { old, new } => {
                        console.undo_rename(new, old);
                        console.log_local(40, "console", &format!("名前を変えられません: {why}"));
                    }
                    ShellReq::Other(what) => console.log_local(40, "console", &format!("{what}に失敗しました: {why}")),
                }
            }
            false
        });
        if self.console.take_dirty() {
            self.persist_at.get_or_insert(Instant::now() + PERSIST_DELAY);
        }
        if self.persist_at.is_some_and(|t| Instant::now() >= t) {
            self.persist_at = None;
            self.console.save();
        }
    }

    /// Pushes the console state into the window. The row model is rebuilt only when the
    /// visible tab changed, at most every 100 ms (at once after a tab switch).
    fn refresh_console(&mut self) {
        let Some(w) = self.window.upgrade() else { return };
        if !w.get_connected() {
            return;
        }
        let cs = w.global::<ConsoleState>();
        let active = self.console.active();
        let tabs: Vec<(String, bool)> =
            self.console.tabs().into_iter().enumerate().map(|(i, n)| (n, i == active)).collect();
        if tabs != self.console_tabs_shown {
            let items: Vec<ConsoleTabInfo> =
                tabs.iter().map(|(n, a)| ConsoleTabInfo { name: n.as_str().into(), active: *a }).collect();
            cs.set_tabs(ModelRc::from(Rc::new(VecModel::from(items))));
            self.console_tabs_shown = tabs;
        }
        let (text, cur) = self.console.input_display();
        let split = text.char_indices().nth(cur).map_or(text.len(), |(i, _)| i);
        cs.set_input_before(text[..split].into());
        cs.set_input_after(text[split..].into());
        let cands = self.console.candidates().to_vec();
        if cands != self.console_cands_shown {
            let items: Vec<SharedString> = cands.iter().map(|c| c.as_str().into()).collect();
            cs.set_candidates(ModelRc::from(Rc::new(VecModel::from(items))));
            self.console_cands_shown = cands;
        }
        let quick = self.console.quick_commands().to_vec();
        if quick != self.console_quick_shown {
            let items: Vec<SharedString> = quick.iter().map(|c| c.as_str().into()).collect();
            cs.set_quick(ModelRc::from(Rc::new(VecModel::from(items))));
            self.console_quick_shown = quick;
        }
        cs.set_is_shell(self.console.active_is_shell());
        cs.set_log_level(LOG_LEVELS.iter().position(|&l| l == self.console.log_filter()).unwrap_or(1) as i32);
        cs.set_ctrl(mod_int(self.console.mod_state(Modifier::Ctrl)));
        cs.set_alt(mod_int(self.console.mod_state(Modifier::Alt)));
        cs.set_shift(mod_int(self.console.mod_state(Modifier::Shift)));
        cs.set_sym(self.console.sym());
        cs.set_bottom_seq(self.console.submit_seq() as i32);
        match self.console.prompt() {
            Some(p) => {
                cs.set_prompt_title(
                    match &p.kind {
                        PromptKind::New => "新しいタブの名前".to_owned(),
                        PromptKind::Rename(i) => {
                            format!("「{}」の新しい名前", self.console.tabs().get(*i).cloned().unwrap_or_default())
                        }
                    }
                    .into(),
                );
                cs.set_prompt_text(p.text.as_str().into());
                cs.set_prompt_error(p.error.clone().unwrap_or_default().into());
                cs.set_prompt_active(true);
            }
            None => cs.set_prompt_active(false),
        }
        let rev = (active, self.console.view_rev());
        if self.console_rev == Some(rev) {
            return;
        }
        let switched = self.console_rev.is_none_or(|(a, _)| a != active);
        if !switched && self.console_built.is_some_and(|t| t.elapsed() < CONSOLE_INTERVAL) {
            return;
        }
        let cols = self.console_cols;
        let new: Vec<Row> = self.console.rows().iter().flat_map(|r| console::wrap_row(r, cols)).collect();
        let m = &self.console_rows;
        let mut common = self.console_shown.iter().zip(&new).take_while(|(a, b)| a == b).count();
        if common == 0
            && let Some(first) = new.first()
        {
            // A full scrollback drops lines at the front: remove them instead of rewriting.
            let old = &self.console_shown;
            if let Some(k) = old.iter().position(|r| r == first).filter(|&k| k > 0) {
                let n = (old.len() - k).min(new.len());
                if old[k..k + n] == new[..n] {
                    for _ in 0..k {
                        m.remove(0);
                    }
                    self.console_shown.drain(..k);
                    common = n;
                }
            }
        }
        let old_len = m.row_count();
        for (i, r) in new.iter().enumerate().take(old_len).skip(common) {
            m.set_row_data(i, console_row(r));
        }
        for r in new.iter().skip(old_len) {
            m.push(console_row(r));
        }
        while m.row_count() > new.len() {
            m.remove(m.row_count() - 1);
        }
        self.console_shown = new;
        self.console_rev = Some(rev);
        self.console_built = Some(Instant::now());
    }

    /// Collects beacons; one that has not been heard for BEACON_TTL leaves the list.
    fn poll_beacons(&mut self) {
        let now = Instant::now();
        let mut changed = false;
        if let Some(rx) = &self.beacons_rx {
            while let Ok((name, addr)) = rx.try_recv() {
                match self.beacons.iter_mut().find(|(n, a, _)| *n == name && *a == addr) {
                    Some(b) => b.2 = now,
                    None => {
                        self.beacons.push((name, addr, now));
                        changed = true;
                    }
                }
            }
        }
        changed |= expire_beacons(&mut self.beacons, now);
        if changed {
            self.beacons_dirty.set(true);
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
        // A real robot: the demo robot (if any) stops.
        self.client = None;
        self.demo.release();
        self.connect_addr(addr, takeover);
    }

    /// The デモ button: start the in-app robot and connect to it like any other (no takeover,
    /// not remembered as the last target).
    fn connect_demo(&mut self) {
        self.client = None;
        self.demo.release();
        match demo::DemoRobot::start() {
            Ok(robot) => {
                let addr = robot.addr();
                self.demo.start(robot);
                self.connect_addr(addr, false);
                self.status = "デモに接続中…".to_owned();
            }
            Err(e) => self.status = format!("デモを始められません: {e}"),
        }
    }

    /// 接続先: BYE (by dropping the client), stop the demo, back to the connection screen.
    fn leave_robot(&mut self) {
        self.client = None;
        self.rx = None;
        end_session(&mut self.model, &mut self.demo, &mut self.latch);
        self.control = ControlInput { estop: self.latch.estop(), ..ControlInput::default() };
        self.reset_session_state(true);
        self.link_addr = None;
        self.notice.clear();
        self.status.clear();
    }

    /// Forget what the previous link left on the phone side. `new_robot` also clears the console
    /// (log tab, shell tabs, pending shell requests); a reconnect to the same robot keeps them.
    fn reset_session_state(&mut self, new_robot: bool) {
        self.manual.release_all();
        self.manual_reqs.clear();
        self.manual_message.clear();
        self.auto_reqs.clear();
        self.auto_message = (String::new(), true);
        self.pending_pose = None;
        self.pane(OUTPUT_MAP).pose_drag = None;
        self.last_task = None;
        self.start_pending = None;
        self.last_sent_task = None;
        self.done_seen = 0;
        if new_robot {
            self.console.reset_session();
            self.shell_reqs.clear();
            self.sync_deadline = None;
            self.console_tabs_shown.clear();
            self.console_cands_shown.clear();
            self.console_rev = None;
        }
    }

    fn connect_addr(&mut self, addr: SocketAddr, takeover: bool) {
        // Drop the old client first: its thread uses the same session socket space.
        self.client = None;
        self.model.begin_connect();
        self.notice.clear();
        self.latch.on_link_reset();
        // The emergency-stop latch survives reconnects; motion inputs restart from zero.
        self.control = ControlInput { estop: self.latch.estop(), ..ControlInput::default() };
        let new_robot = self.link_addr != Some(addr);
        self.reset_session_state(new_robot);
        self.link_addr = Some(addr);
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
        // UI heartbeat: the link thread zeroes the motion if this stops for 250 ms.
        self.push_control();
        self.poll_beacons();
        let mut events = Vec::new();
        if let Some(rx) = &self.rx {
            while let Ok(ev) = rx.try_recv() {
                events.push(ev);
            }
        }
        let got_events = !events.is_empty();
        let mut connected_now = false;
        for ev in events {
            connected_now |= matches!(ev, LinkEvent::Connected { .. });
            if matches!(ev, LinkEvent::Rejected { .. } | LinkEvent::Bye { .. } | LinkEvent::Failed { .. }) {
                // The client thread idles after these; release it so Connect starts a fresh one.
                self.client = None;
                self.rx = None;
                self.demo.release();
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
        self.poll_console(connected_now);
        self.sync_manual();
        self.sync_pose_confirm();
        self.refresh();
        self.refresh_console();
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
        w.set_demo(connected && self.demo.active());
        w.set_conn_text(text.into());
        w.set_conn_color(color);
        w.set_rtt_ms(rtt);
        w.set_mode(mode);
        w.set_stopped(self.control.estop || mode == 2);
        w.set_notice(self.notice.as_str().into());
        if connected {
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
        if self.beacons_dirty.replace(false) {
            self.beacons_model.set_vec(
                self.beacons
                    .iter()
                    .map(|(n, a, _)| Beacon { name: n.as_str().into(), addr: a.to_string().into() })
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
    let console_rows = Rc::new(VecModel::default());
    window.global::<ConsoleState>().set_rows(ModelRc::from(console_rows.clone()));
    let glue = Rc::new(RefCell::new(Glue {
        window: window.as_weak(),
        model: AppModel::new(),
        client: None,
        demo: DemoSlot::default(),
        link_addr: None,
        rx: None,
        _discovery: discovery,
        beacons_rx,
        beacons: Vec::new(),
        beacons_dirty: std::cell::Cell::new(true),
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
        console: {
            let mut c = Console::new();
            c.load();
            c
        },
        console_rows: console_rows.clone(),
        console_shown: Vec::new(),
        console_rev: None,
        console_built: None,
        console_cols: 60,
        console_tabs_shown: Vec::new(),
        console_quick_shown: Vec::new(),
        console_cands_shown: Vec::new(),
        shell_reqs: Vec::new(),
        sync_deadline: None,
        persist_at: None,
    }));
    {
        let g = glue.clone();
        window.on_connect(move |t, takeover| {
            let mut g = g.borrow_mut();
            g.connect(&t, takeover);
            g.refresh();
        });
        let g = glue.clone();
        window.on_connect_demo(move || {
            let mut g = g.borrow_mut();
            g.connect_demo();
            g.refresh();
        });
        let g = glue.clone();
        window.on_disconnect(move || {
            let mut g = g.borrow_mut();
            g.leave_robot();
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
        window.on_manual_jog_released(move |side| g.borrow_mut().manual_jog_released(side));
        let g = glue.clone();
        window.on_window_deactivated(move || {
            if let Ok(mut g) = g.try_borrow_mut() {
                g.window_deactivated();
            }
        });
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
            g.sync_pose_confirm();
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_auto_pose_picked(move |x, y, yaw| {
            let mut g = g.borrow_mut();
            g.auto_pose_picked(x, y, yaw);
            g.sync_pose_confirm();
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_auto_pose_cancelled(move || {
            let mut g = g.borrow_mut();
            g.auto_pose_clear();
            g.sync_pose_confirm();
            g.render_maps(false);
        });
        let g = glue.clone();
        window.on_auto_pose_confirm(move || {
            let mut g = g.borrow_mut();
            g.auto_pose_confirm();
            g.sync_pose_confirm();
            g.render_maps(false);
        });
        let cs = window.global::<ConsoleState>();
        let g = glue.clone();
        cs.on_key(move |c| g.borrow_mut().console_do(|k| k.key(&c)));
        let g = glue.clone();
        cs.on_select_tab(move |i| {
            let mut g = g.borrow_mut();
            g.console.set_active(usize::try_from(i).unwrap_or(0));
            g.refresh_console();
        });
        let g = glue.clone();
        cs.on_swipe(move |d| {
            let mut g = g.borrow_mut();
            g.console.step_active(d);
            g.refresh_console();
        });
        let g = glue.clone();
        cs.on_new_tab(move || {
            let mut g = g.borrow_mut();
            g.console.begin_new_tab();
            show_keyboard(&g);
            g.refresh_console();
        });
        let g = glue.clone();
        cs.on_rename_tab(move |i| {
            let mut g = g.borrow_mut();
            g.console.begin_rename(usize::try_from(i).unwrap_or(0));
            show_keyboard(&g);
            g.refresh_console();
        });
        let g = glue.clone();
        cs.on_close_tab(move |i| {
            g.borrow_mut().console_do(|k| k.close_tab(usize::try_from(i).unwrap_or(0)).into_iter().collect());
        });
        let g = glue.clone();
        cs.on_pick_candidate(move |i| {
            let mut g = g.borrow_mut();
            g.console.pick_candidate(usize::try_from(i).unwrap_or(usize::MAX));
            g.refresh_console();
        });
        let g = glue.clone();
        cs.on_quick_insert(move |i| {
            let mut g = g.borrow_mut();
            if let Some(t) = usize::try_from(i).ok().and_then(|i| g.console.quick_commands().get(i).cloned()) {
                g.console.insert_text(&t);
            }
            g.refresh_console();
        });
        let g = glue.clone();
        cs.on_quick_add(move || {
            let mut g = g.borrow_mut();
            g.console.add_quick_from_line();
            g.refresh_console();
        });
        let g = glue.clone();
        cs.on_quick_remove(move |i| {
            let mut g = g.borrow_mut();
            g.console.remove_quick(usize::try_from(i).unwrap_or(usize::MAX));
            g.refresh_console();
        });
        let g = glue.clone();
        cs.on_fold(move |b| {
            let mut g = g.borrow_mut();
            g.console.toggle_fold(usize::try_from(b).unwrap_or(usize::MAX));
            g.console_rev = None;
            g.refresh_console();
        });
        let g = glue.clone();
        cs.on_output_width(move |w| {
            let mut g = g.borrow_mut();
            let cols = (((w - 10.0) / CELL_PX) as usize).max(8);
            if cols != g.console_cols {
                g.console_cols = cols;
                g.console_rev = None;
                g.refresh_console();
            }
        });
        let g = glue.clone();
        cs.on_set_level(move |i| g.borrow_mut().console_set_level(i));
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

impl Drop for GlueHandle {
    /// Histories and quick commands are written on exit too (saving is otherwise debounced).
    fn drop(&mut self) {
        if let Ok(mut g) = self._glue.try_borrow_mut() {
            g.console.save();
            g.client = None;
            g.demo.release();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn silent_beacons_expire_after_five_seconds() {
        let t0 = Instant::now();
        let addr: SocketAddr = "10.0.0.2:47800".parse().unwrap();
        let mut list = vec![("old".to_owned(), addr, t0), ("new".to_owned(), addr, t0 + Duration::from_secs(4))];
        assert!(!expire_beacons(&mut list, t0 + Duration::from_millis(4900)));
        assert!(expire_beacons(&mut list, t0 + Duration::from_secs(6)));
        assert_eq!(list.len(), 1);
        assert_eq!(list[0].0, "new");
    }

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

    fn touch(w: &slint::Window, x: f32, y: f32, phase: i_slint_core::input::TouchPhase) {
        use i_slint_core::{lengths::LogicalPoint, platform::{InternalEvent, WindowEvent}};
        w.dispatch_event(WindowEvent::internal(InternalEvent::Touch { id: 0, position: LogicalPoint::new(x, y), phase }));
    }

    fn pump(ms: u64) {
        i_slint_backend_testing::mock_elapsed_time(Duration::from_millis(ms));
        slint::platform::update_timers_and_animations();
    }

    /// The whole 地図で姿勢を指定 flow on the real window and the in-app demo robot: one tap on
    /// 決定 sends exactly one SET_POSE, and the robot ends up at the picked pose.
    #[test]
    fn pose_pick_flow_sends_one_set_pose() {
        use i_slint_core::input::TouchPhase::{Ended, Moved, Started};
        let dir = tempfile::tempdir().unwrap();
        storage::set_data_dir(dir.path().to_path_buf());
        i_slint_backend_testing::init_no_event_loop();
        let win = MainWindow::new().unwrap();
        win.window().set_size(slint::LogicalSize::new(800., 360.));
        let handle = start(&win);
        win.show().unwrap();
        win.invoke_connect_demo();
        let glue = handle.borrow()._glue.clone();
        let t0 = Instant::now();
        while !glue.borrow().link_live() {
            assert!(t0.elapsed() < Duration::from_secs(10), "demo never connected");
            pump(50);
            std::thread::sleep(Duration::from_millis(20));
        }
        pump(100);
        win.set_page(2);
        pump(50);
        // 地図で姿勢を指定 (bottom row, right end).
        touch(win.window(), 630., 332., Started);
        touch(win.window(), 630., 332., Ended);
        pump(50);
        assert_eq!(win.get_page(), 0, "the button shows the output screen");
        assert!(win.get_pose_pick());
        // Long press on the map, drag towards upper right, lift.
        touch(win.window(), 200., 176., Started);
        pump(600);
        touch(win.window(), 240., 160., Moved);
        touch(win.window(), 260., 140., Moved);
        touch(win.window(), 260., 140., Ended);
        pump(50);
        assert!(win.get_auto_confirm_active(), "the confirmation is shown");
        let (px, py, _) = glue.borrow().pending_pose.expect("a picked pose");
        assert_eq!(glue.borrow().auto_reqs.len(), 0, "nothing is sent before 決定");
        // 決定
        touch(win.window(), 100., 108., Started);
        touch(win.window(), 100., 108., Ended);
        assert_eq!(glue.borrow().auto_reqs.len(), 1, "exactly one SET_POSE");
        assert!(!win.get_pose_pick() && !win.get_auto_confirm_active());
        // A second tap where 決定 was now lands on the map and sends nothing.
        touch(win.window(), 100., 108., Started);
        touch(win.window(), 100., 108., Ended);
        pump(50);
        // (The demo may already have answered the first one, so only "not more" can be checked.)
        assert!(glue.borrow().auto_reqs.len() <= 1);
        // The demo robot took the pose.
        let t0 = Instant::now();
        loop {
            pump(50);
            std::thread::sleep(Duration::from_millis(20));
            let pose = glue.borrow().model.state().and_then(|s| s.pose);
            if pose.is_some_and(|p| (p[0] - px).abs() < 0.05 && (p[1] - py).abs() < 0.05) {
                break;
            }
            assert!(t0.elapsed() < Duration::from_secs(10), "the robot never took the pose {px},{py}");
        }
    }
}
