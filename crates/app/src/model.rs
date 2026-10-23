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
}

#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub walls: Vec<[f64; 4]>,
    pub shelf: [f64; 4],
    pub start: [f64; 3],
    pub robot_radius: f64,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct TaskMemory {
    pub holding: [bool; 2],
    pub picked: Vec<u8>,
    pub placed: Vec<(u8, u8)>,
}

pub const RESULTS_KEPT: usize = 20;

#[derive(Debug)]
pub struct AppModel {
    conn: ConnState,
    robot_name: Option<String>,
    state: Option<State>,
    config: Option<Config>,
    field: Option<Field>,
    path_shapes: HashMap<String, Vec<(i16, i16)>>,
    task_memory: Option<TaskMemory>,
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
            LinkEvent::Bye { reason } => {
                self.conn = ConnState::Rejected(reason);
            }
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
            Msg::Config { robot_name, paths, lift_presets, lift_max, speed_levels, blocks, columns, rows } => {
                self.config = Some(Config {
                    robot_name, paths, lift_presets, lift_max, speed_levels, blocks, columns, rows,
                });
            }
            Msg::Field { walls, shelf, start, robot_radius } => {
                self.field = Some(Field { walls, shelf, start, robot_radius });
            }
            Msg::PathShape { name, points } => {
                self.path_shapes.insert(name, points);
            }
            Msg::TaskMemory { holding, picked, placed } => {
                self.task_memory = Some(TaskMemory { holding, picked, placed });
            }
            Msg::ShellOpened { tab, replay } => self.shell.push(ShellEvent::Opened { tab, replay }),
            Msg::ShellOutput { tab, data } => self.shell.push(ShellEvent::Output { tab, data }),
            Msg::ShellCompletion { req, tab, start, candidates } => {
                self.shell.push(ShellEvent::Completion { req, tab, start, candidates });
            }
            Msg::ShellClosed { tab, reason } => self.shell.push(ShellEvent::Closed { tab, reason }),
            Msg::Log { stamp_ms, level, name, text } => {
                self.shell.push(ShellEvent::Log { stamp_ms, level, name, text });
            }
            // TaskDone and phone-to-robot kinds: nothing to keep here yet.
            _ => {}
        }
    }

    pub fn connection(&self) -> ConnState {
        self.conn.clone()
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
        m.apply(LinkEvent::Bye { reason: "taken over".into() });
        assert_eq!(m.connection(), ConnState::Rejected("taken over".into()));
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
        }));
        m.apply(LinkEvent::Msg(Msg::Field {
            walls: vec![[0.0, 0.0, 1.0, 0.0]],
            shelf: [1.0, 2.0, 3.0, 4.0],
            start: [0.0, 0.0, 0.0],
            robot_radius: 0.3,
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
        assert_eq!(m.task_memory().unwrap().placed, vec![(0, 1)]);
        assert_eq!(m.scan(), &[(1.0, 2.0)]);
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
}
