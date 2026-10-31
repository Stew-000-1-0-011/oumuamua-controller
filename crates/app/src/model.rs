//! Slint-free application state fed by `LinkEvent`s.

use link::LinkEvent;
use proto::{Msg, State};
use std::collections::{HashMap, VecDeque};

#[derive(Debug, Clone, PartialEq)]
pub enum ConnState {
    Searching,
    Connecting,
    Connected { rtt_ms: Option<u32> },
    Lost,
    Rejected(String),
    /// Another phone took over (BYE).
    TakenOver(String),
}

/// Shell/log traffic collected for the console (Task 16).
#[derive(Debug, Clone, PartialEq)]
pub enum ShellEvent {
    Opened { tab: String, replay: Vec<u8> },
    Output { tab: String, data: Vec<u8> },
    Completion { req: u32, tab: String, start: u32, candidates: Vec<String> },
    Closed { tab: String, reason: String },
    Log { stamp_ms: u64, level: u8, name: String, text: String },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    pub robot_name: String,
    pub paths: Vec<String>,
    pub lift_presets: Vec<(String, f64)>,
    pub lift_max: f64,
    pub speed_levels: [(f64, f64); 3],
    pub blocks: u8,
    pub columns: u8,
    pub rows: u8,
    /// The bridge drives harite; false from an older bridge (the controls are then hidden).
    pub harite: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub bounds: [f64; 4],
    pub obstacles: Vec<[f64; 4]>,
    pub shelf: [f64; 4],
    pub start: [f64; 3],
    pub robot_radius: f64,
    /// The robot's outline in its own frame (x forward, y left); empty = draw a `robot_radius` circle.
    pub footprint: Vec<(f64, f64)>,
    /// Suction centres in the robot frame, gripper0 (left) first; may be empty.
    pub suction: Vec<(f64, f64)>,
}

/// What task_commander remembers. It no longer remembers placed cells: the TASK_MEMORY `placed` list is
/// kept on the wire for compatibility, but it is ignored here.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TaskMemory {
    pub holding: [bool; 2],
    pub picked: Vec<u8>,
}

/// Nothing drains shell events until the console exists; keep memory bounded.
pub const SHELL_EVENTS_KEPT: usize = 10_000;

/// The phone-side emergency stop. It survives reconnects, rejections and BYE; only a
/// RELEASE_STOP that the robot confirms clears it.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct StopLatch {
    estop: bool,
    pending: Option<u32>,
}

impl StopLatch {
    pub fn estop(&self) -> bool {
        self.estop
    }
    /// Request id of the RELEASE_STOP awaiting its RESULT.
    pub fn pending(&self) -> Option<u32> {
        self.pending
    }
    pub fn press(&mut self) {
        self.estop = true;
        self.pending = None;
    }
    /// 解除 pressed and RELEASE_STOP `req` sent: estop is tentatively off.
    pub fn release_sent(&mut self, req: u32) {
        self.estop = false;
        self.pending = Some(req);
    }
    /// RESULT for `req`; a failed release restores the latch.
    pub fn on_result(&mut self, req: u32, ok: bool) {
        if self.pending == Some(req) {
            self.pending = None;
            if !ok {
                self.estop = true;
            }
        }
    }
    /// New client, Rejected or Bye: the estop stays; an unanswered release counts as failed.
    pub fn on_link_reset(&mut self) {
        if self.pending.take().is_some() {
            self.estop = true;
        }
    }
}

pub const RESULTS_KEPT: usize = 20;

/// Holds the in-app demo robot (any guard that stops on drop) while the app is connected to
/// it. The "is demo" decision for the screen lives here, not in the UI.
#[derive(Debug)]
pub struct DemoSlot<T> {
    robot: Option<T>,
}

impl<T> Default for DemoSlot<T> {
    fn default() -> Self {
        DemoSlot { robot: None }
    }
}

impl<T> DemoSlot<T> {
    /// The demo robot started; any earlier one is dropped (stopped) first.
    pub fn start(&mut self, robot: T) {
        // None first: the old robot must stop before the new one is stored.
        self.robot = None;
        self.robot = Some(robot);
    }

    /// Stops the demo robot, if any (connecting elsewhere, link ended, app exit).
    pub fn release(&mut self) {
        self.robot = None;
    }

    /// True while the demo robot exists: the demo indicator is on.
    pub fn active(&self) -> bool {
        self.robot.is_some()
    }
}

/// The user left the robot (接続先): the demo robot stops, the model returns to the connection
/// screen, and the phone's stop latch is kept (it only forgets a RELEASE_STOP in flight).
pub fn end_session<T>(model: &mut AppModel, demo: &mut DemoSlot<T>, latch: &mut StopLatch) {
    demo.release();
    model.reset();
    latch.on_link_reset();
}

#[derive(Debug)]
pub struct AppModel {
    conn: ConnState,
    robot_name: Option<String>,
    state: Option<State>,
    config: Option<Config>,
    field: Option<Field>,
    path_shapes: HashMap<String, Vec<(i16, i16)>>,
    task_memory: Option<TaskMemory>,
    /// Latest TASK_DONE `(kind, ok, message)` and how many have arrived.
    task_done: Option<(u8, bool, String)>,
    task_done_count: u64,
    scan: Vec<(f32, f32)>,
    /// `(req, ok, message)`, oldest first, last 20.
    pub results: VecDeque<(u32, bool, String)>,
    shell: Vec<ShellEvent>,
}

impl Default for AppModel {
    fn default() -> Self {
        Self::new()
    }
}

impl AppModel {
    pub fn new() -> Self {
        AppModel {
            conn: ConnState::Searching,
            robot_name: None,
            state: None,
            config: None,
            field: None,
            path_shapes: HashMap::new(),
            task_memory: None,
            task_done: None,
            task_done_count: 0,
            scan: Vec::new(),
            results: VecDeque::new(),
            shell: Vec::new(),
        }
    }

    /// A new `Client` was started: forget everything about the previous link.
    pub fn begin_connect(&mut self) {
        *self = AppModel { conn: ConnState::Connecting, ..AppModel::new() };
    }

    /// Back to the connection screen.
    pub fn reset(&mut self) {
        *self = AppModel::new();
    }

    pub fn apply(&mut self, ev: LinkEvent) {
        match ev {
            LinkEvent::Connected { robot_name } => {
                self.robot_name = Some(robot_name);
                self.conn = ConnState::Connected { rtt_ms: None };
            }
            LinkEvent::Rejected { reason } => self.conn = ConnState::Rejected(reason),
            LinkEvent::Bye { reason } => self.conn = ConnState::TakenOver(reason),
            LinkEvent::Failed { message } => self.conn = ConnState::Rejected(message),
            LinkEvent::State { body, rtt_ms } => {
                let rtt = rtt_ms.or(match self.conn {
                    ConnState::Connected { rtt_ms } => rtt_ms,
                    _ => None,
                });
                self.conn = ConnState::Connected { rtt_ms: rtt };
                self.state = Some(body);
            }
            LinkEvent::Scan(points) => self.scan = points,
            LinkEvent::Lost => self.conn = ConnState::Lost,
            LinkEvent::Msg(m) => self.apply_msg(m),
        }
    }

    fn apply_msg(&mut self, m: Msg) {
        match m {
            Msg::Result { req, ok, message } => {
                self.results.push_back((req, ok, message));
                while self.results.len() > RESULTS_KEPT {
                    self.results.pop_front();
                }
            }
            Msg::Config { robot_name, paths, lift_presets, lift_max, speed_levels, blocks, columns, rows, harite } => {
                self.config = Some(Config {
                    robot_name, paths, lift_presets, lift_max, speed_levels, blocks, columns, rows, harite,
                });
            }
            Msg::Field { bounds, obstacles, shelf, start, robot_radius, footprint, suction } => {
                self.field = Some(Field { bounds, obstacles, shelf, start, robot_radius, footprint, suction });
            }
            Msg::PathShape { name, points } => {
                self.path_shapes.insert(name, points);
            }
            Msg::TaskMemory { holding, picked, placed: _ } => {
                self.task_memory = Some(TaskMemory { holding, picked });
            }
            Msg::ShellOpened { tab, replay } => self.push_shell(ShellEvent::Opened { tab, replay }),
            Msg::ShellOutput { tab, data } => self.push_shell(ShellEvent::Output { tab, data }),
            Msg::ShellCompletion { req, tab, start, candidates } => {
                self.push_shell(ShellEvent::Completion { req, tab, start, candidates });
            }
            Msg::ShellClosed { tab, reason } => self.push_shell(ShellEvent::Closed { tab, reason }),
            Msg::Log { stamp_ms, level, name, text } => {
                self.push_shell(ShellEvent::Log { stamp_ms, level, name, text });
            }
            Msg::TaskDone { kind, ok, message } => {
                self.task_done = Some((kind, ok, message));
                self.task_done_count += 1;
            }
            // Phone-to-robot kinds: nothing to keep here yet.
            _ => {}
        }
    }

    fn push_shell(&mut self, e: ShellEvent) {
        self.shell.push(e);
        if self.shell.len() > SHELL_EVENTS_KEPT {
            let drop = self.shell.len() - SHELL_EVENTS_KEPT;
            self.shell.drain(..drop);
        }
    }

    pub fn connection(&self) -> ConnState {
        self.conn.clone()
    }
    /// Replaces the CONFIG (tests: an older bridge's flag).
    #[cfg(test)]
    pub fn set_config_for_test(&mut self, c: Config) {
        self.config = Some(c);
    }
    pub fn robot_name(&self) -> Option<&str> {
        self.robot_name.as_deref()
    }
    pub fn state(&self) -> Option<&State> {
        self.state.as_ref()
    }
    pub fn config(&self) -> Option<&Config> {
        self.config.as_ref()
    }
    pub fn field(&self) -> Option<&Field> {
        self.field.as_ref()
    }
    pub fn path_shape(&self, name: &str) -> Option<&[(i16, i16)]> {
        self.path_shapes.get(name).map(Vec::as_slice)
    }
    pub fn path_shapes(&self) -> &HashMap<String, Vec<(i16, i16)>> {
        &self.path_shapes
    }
    pub fn task_memory(&self) -> Option<&TaskMemory> {
        self.task_memory.as_ref()
    }
    /// The latest TASK_DONE and a counter that grows with each one.
    pub fn task_done(&self) -> (u64, Option<&(u8, bool, String)>) {
        (self.task_done_count, self.task_done.as_ref())
    }
    pub fn scan(&self) -> &[(f32, f32)] {
        &self.scan
    }
    pub fn take_shell_events(&mut self) -> Vec<ShellEvent> {
        std::mem::take(&mut self.shell)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proto::Mode;

    fn st(mode: Mode) -> State {
        State {
            mode,
            stop_reason: String::new(),
            pose: None,
            vel: None,
            wheels: vec![],
            grippers: [None, None],
            task: None,
            path: None,
            tracker_enabled: false,
            harite_command: None,
        }
    }

    #[test]
    fn connected_then_state_updates_rtt() {
        let mut m = AppModel::new();
        assert_eq!(m.connection(), ConnState::Searching);
        m.apply(LinkEvent::Connected { robot_name: "r".into() });
        assert_eq!(m.connection(), ConnState::Connected { rtt_ms: None });
        m.apply(LinkEvent::State { body: st(Mode::Manual), rtt_ms: Some(12) });
        assert_eq!(m.connection(), ConnState::Connected { rtt_ms: Some(12) });
        assert_eq!(m.state().unwrap().mode, Mode::Manual);
        // A STATE without an rtt sample keeps the last one.
        m.apply(LinkEvent::State { body: st(Mode::Stopped), rtt_ms: None });
        assert_eq!(m.connection(), ConnState::Connected { rtt_ms: Some(12) });
        assert_eq!(m.state().unwrap().mode, Mode::Stopped);
    }

    #[test]
    fn lost_and_recovered() {
        let mut m = AppModel::new();
        m.apply(LinkEvent::Connected { robot_name: "r".into() });
        m.apply(LinkEvent::Lost);
        assert_eq!(m.connection(), ConnState::Lost);
        m.apply(LinkEvent::State { body: st(Mode::Manual), rtt_ms: Some(5) });
        assert_eq!(m.connection(), ConnState::Connected { rtt_ms: Some(5) });
    }

    #[test]
    fn rejected_shows_reason() {
        let mut m = AppModel::new();
        m.begin_connect();
        assert_eq!(m.connection(), ConnState::Connecting);
        m.apply(LinkEvent::Rejected { reason: "busy".into() });
        assert_eq!(m.connection(), ConnState::Rejected("busy".into()));
        m.apply(LinkEvent::Failed { message: "bind failed".into() });
        assert_eq!(m.connection(), ConnState::Rejected("bind failed".into()));
        m.apply(LinkEvent::Bye { reason: "taken over".into() });
        assert_eq!(m.connection(), ConnState::TakenOver("taken over".into()));
    }

    #[test]
    fn results_keep_last_20() {
        let mut m = AppModel::new();
        for i in 0..25u32 {
            m.apply(LinkEvent::Msg(Msg::Result { req: i, ok: i % 2 == 0, message: format!("m{i}") }));
        }
        assert_eq!(m.results.len(), 20);
        assert_eq!(m.results.front().unwrap(), &(5, false, "m5".to_string()));
        assert_eq!(m.results.back().unwrap(), &(24, true, "m24".to_string()));
    }

    #[test]
    fn config_field_path_task_memory_stored() {
        let mut m = AppModel::new();
        assert!(m.config().is_none() && m.field().is_none() && m.task_memory().is_none());
        m.apply(LinkEvent::Msg(Msg::Config {
            robot_name: "r".into(),
            paths: vec!["a".into()],
            lift_presets: vec![("low".into(), 0.1)],
            lift_max: 0.5,
            speed_levels: [(0.3, 1.0), (0.8, 1.8), (1.5, 2.5)],
            blocks: 6,
            columns: 3,
            rows: 3,
            harite: true,
        }));
        m.apply(LinkEvent::Msg(Msg::Field {
            bounds: [0.0, 0.0, 5.0, 4.0],
            obstacles: vec![[0.0, 0.0, 1.0, 1.0]],
            shelf: [1.0, 2.0, 3.0, 4.0],
            start: [0.0, 0.0, 0.0],
            robot_radius: 0.3,
            footprint: vec![(0.2, 0.1), (-0.3, 0.0)],
            suction: vec![(0.27, 0.12), (0.27, -0.12)],
        }));
        m.apply(LinkEvent::Msg(Msg::PathShape { name: "p".into(), points: vec![(1, 2), (3, 4)] }));
        m.apply(LinkEvent::Msg(Msg::TaskMemory {
            holding: [true, false],
            picked: vec![1, 2],
            placed: vec![(0, 1)],
        }));
        m.apply(LinkEvent::Scan(vec![(1.0, 2.0)]));
        assert_eq!(m.config().unwrap().paths, vec!["a".to_string()]);
        assert_eq!(m.field().unwrap().robot_radius, 0.3);
        assert_eq!(m.path_shape("p").unwrap(), &[(1, 2), (3, 4)]);
        assert!(m.path_shape("q").is_none());
        assert_eq!(m.field().unwrap().footprint.len(), 2);
        assert_eq!(m.field().unwrap().suction, vec![(0.27, 0.12), (0.27, -0.12)]);
        // The wire still carries `placed`; the model ignores it.
        assert_eq!(m.task_memory().unwrap(), &TaskMemory { holding: [true, false], picked: vec![1, 2] });
        assert_eq!(m.scan(), &[(1.0, 2.0)]);
    }

    #[test]
    fn latch_survives_reconnect_bye_and_rejected() {
        let mut l = StopLatch::default();
        l.press();
        l.on_link_reset(); // new Client / Rejected / Bye
        assert!(l.estop());
        l.on_link_reset();
        assert!(l.estop());
    }

    #[test]
    fn latch_release_ok_clears_and_fail_restores() {
        let mut l = StopLatch::default();
        l.press();
        l.release_sent(7);
        assert!(!l.estop());
        l.on_result(8, false); // unrelated
        assert!(!l.estop());
        l.on_result(7, true);
        assert!(!l.estop() && l.pending().is_none());
        l.press();
        l.release_sent(9);
        l.on_result(9, false);
        assert!(l.estop());
        l.release_sent(10);
        l.on_link_reset(); // link dropped before the RESULT
        assert!(l.estop());
    }

    #[test]
    fn shell_events_are_capped() {
        let mut m = AppModel::new();
        for i in 0..(SHELL_EVENTS_KEPT + 5) {
            m.apply(LinkEvent::Msg(Msg::ShellClosed { tab: i.to_string(), reason: String::new() }));
        }
        let ev = m.take_shell_events();
        assert_eq!(ev.len(), SHELL_EVENTS_KEPT);
        assert!(matches!(&ev[0], ShellEvent::Closed { tab, .. } if tab == "5"));
    }

    #[test]
    fn shell_events_are_taken_once() {
        let mut m = AppModel::new();
        m.apply(LinkEvent::Msg(Msg::ShellOutput { tab: "t".into(), data: b"x".to_vec() }));
        m.apply(LinkEvent::Msg(Msg::Log { stamp_ms: 1, level: 2, name: "n".into(), text: "x".into() }));
        let ev = m.take_shell_events();
        assert_eq!(ev.len(), 2);
        assert!(matches!(ev[1], ShellEvent::Log { level: 2, .. }));
        assert!(m.take_shell_events().is_empty());
    }

    #[test]
    fn task_done_is_kept_and_counted() {
        let mut m = AppModel::new();
        assert_eq!(m.task_done(), (0, None));
        m.apply(LinkEvent::Msg(Msg::TaskDone { kind: 2, ok: false, message: "blocked".into() }));
        let (n, d) = m.task_done();
        assert_eq!((n, d.cloned()), (1, Some((2, false, "blocked".to_owned()))));
        m.begin_connect();
        assert_eq!(m.task_done().0, 0);
    }

    #[test]
    fn demo_slot_indicator_and_drop() {
        use std::cell::Cell;
        use std::rc::Rc;
        struct Guard(Rc<Cell<u32>>);
        impl Drop for Guard {
            fn drop(&mut self) {
                self.0.set(self.0.get() + 1);
            }
        }
        let stopped = Rc::new(Cell::new(0));
        let mut slot: DemoSlot<Guard> = DemoSlot::default();
        assert!(!slot.active());
        slot.start(Guard(stopped.clone()));
        assert!(slot.active() && stopped.get() == 0);
        // Switching to a real target releases it: indicator off, robot dropped.
        slot.release();
        assert!(!slot.active() && stopped.get() == 1);
        slot.release();
        assert_eq!(stopped.get(), 1);
        // Pressing デモ twice stops the first.
        slot.start(Guard(stopped.clone()));
        slot.start(Guard(stopped.clone()));
        assert_eq!(stopped.get(), 2);
        drop(slot); // app exit
        assert_eq!(stopped.get(), 3);
    }

    #[test]
    fn leaving_the_demo_drops_it_and_keeps_the_latch() {
        let mut model = AppModel::new();
        let mut demo: DemoSlot<u8> = DemoSlot::default();
        let mut latch = StopLatch::default();
        model.begin_connect();
        model.apply(LinkEvent::Connected { robot_name: "demo".into() });
        demo.start(1);
        latch.press();
        latch.release_sent(5); // a RELEASE_STOP in flight when the user leaves
        end_session(&mut model, &mut demo, &mut latch);
        assert!(!demo.active());
        assert_eq!(model.connection(), ConnState::Searching);
        assert!(latch.estop() && latch.pending().is_none());
    }
}
