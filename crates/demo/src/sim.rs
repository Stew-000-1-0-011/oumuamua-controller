//! The simulated robot: a planar pose driven by body-frame velocity, two lifts, suction,
//! a staged task, a timed path run and a 1-degree LiDAR. Time advances only through `advance`.

use proto::{Gripper, TaskState};

pub const ROBOT_NAME: &str = "oumuamua (demo)";
pub const PATHS: [&str; 2] = ["slalom_forward", "slalom_backward"];
// Field and lift values are config.py DEFAULTS (ros/oumuamua_controller_bridge); a test keeps them equal.
// The presets above lift_max stay: the phone disables those buttons, as with the real robot.
pub const LIFT_PRESETS: [(&str, f64); 4] = [("下段", 0.01), ("中段", 0.35), ("上段", 0.69), ("0 点", 0.0)];
pub const LIFT_MAX: f64 = 0.85;
pub const SPEED_LEVELS: [(f64, f64); 3] = [(0.3, 1.0), (0.8, 1.8), (1.5, 2.5)];
pub const BLOCKS: u8 = 6;
pub const COLUMNS: u8 = 3;
pub const ROWS: u8 = 3;
pub const BOUNDS: [f64; 4] = [-2.786, -1.655, 2.786, 1.655];
pub const SHELF: [f64; 4] = [1.5625, -0.30, 2.5025, 0.30];
pub const START: [f64; 3] = [-2.2, 1.3, 0.0];
pub const ROBOT_RADIUS: f64 = 0.32;
pub const OBSTACLES: [[f64; 4]; 13] = [
    [-2.8025, -0.0165, 1.5625, 0.0165],
    [-0.8375, 0.019, -0.8045, 0.8905],
    [-0.8375, -0.8905, -0.8045, -0.019],
    [-1.5955, 1.069, -1.5625, 1.669],
    [-1.5955, -1.669, -1.5625, -1.069],
    [-1.5955, 0.0165, -1.5625, 0.2665],
    [-1.5955, -0.2665, -1.5625, -0.0165],
    [0.0045, 0.8715, 0.0375, 1.6715],
    [0.0045, -1.6715, 0.0375, -0.8715],
    [0.8045, 0.019, 0.8375, 0.8905],
    [0.8045, -0.8905, 0.8375, -0.019],
    [2.5025, -0.0165, 2.8025, 0.0165],
    [1.5625, -0.30, 2.5025, 0.30],
];

const LIFT_SPEED: f64 = 0.05; // m/s
const STAGE_S: f64 = 1.0;
const PATH_RUN_S: f64 = 3.0;
const MAX_RANGE: f64 = 30.0;
/// RunTask kinds with their own stages: 0 = pick up a block, 5 = place on the shelf.
const PICK: u8 = 0;
const PLACE: u8 = 5;

fn stages(kind: u8) -> &'static [&'static str] {
    match kind {
        PICK => &["ブロックへ移動", "吸着", "持ち上げ"],
        PLACE => &["棚へ移動", "差し込み", "解放"],
        _ => &["移動中", "到着"],
    }
}

#[derive(Debug, Clone)]
pub enum Event {
    /// A lift request (`req`) ended on gripper `g`.
    Lift { req: u32, g: usize, ok: bool, msg: &'static str },
    TaskDone { kind: u8 },
    PathDone,
}

#[derive(Debug, Default, Clone)]
struct Lift {
    h: f64,
    goal: f64,
    moving: bool,
    req: Option<u32>,
}

struct Task {
    kind: u8,
    args: [i32; 4], // gripper, block, column, row
    stage: usize,
    t: f64,
}

struct PathRun {
    name: String,
    t: f64,
    waypoints: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Memory {
    pub holding: [bool; 2],
    pub picked: Vec<u8>,
    pub placed: Vec<(u8, u8)>,
}

pub struct Sim {
    pub pose: [f64; 3],
    /// Body-frame velocity; integrated into the pose.
    pub cmd: [f64; 3],
    pub tracker: bool,
    pub suction: [bool; 2],
    lifts: [Lift; 2],
    task: Option<Task>,
    path: Option<PathRun>,
    memory: Memory,
}

impl Default for Sim {
    fn default() -> Self {
        Sim {
            pose: START,
            cmd: [0.0; 3],
            tracker: false,
            suction: [false; 2],
            lifts: Default::default(),
            task: None,
            path: None,
            memory: Memory::default(),
        }
    }
}

impl Sim {
    pub fn busy(&self) -> bool {
        self.task.is_some() || self.path.is_some()
    }

    pub fn advance(&mut self, dt: f64) -> Vec<Event> {
        let mut ev = Vec::new();
        let [vx, vy, wz] = self.cmd;
        let (s, c) = self.pose[2].sin_cos();
        self.pose[0] += (vx * c - vy * s) * dt;
        self.pose[1] += (vx * s + vy * c) * dt;
        self.pose[2] += wz * dt;
        for (g, l) in self.lifts.iter_mut().enumerate() {
            if !l.moving {
                continue;
            }
            let (err, step) = (l.goal - l.h, LIFT_SPEED * dt);
            if err.abs() <= step {
                (l.h, l.moving) = (l.goal, false);
                if let Some(req) = l.req.take() {
                    ev.push(Event::Lift { req, g, ok: true, msg: "" });
                }
            } else {
                l.h += step.copysign(err);
            }
        }
        if let Some(t) = &mut self.task {
            t.t += dt;
            while t.t >= STAGE_S && t.stage < stages(t.kind).len() {
                t.t -= STAGE_S;
                t.stage += 1;
            }
            if t.stage >= stages(t.kind).len() {
                let (kind, args) = (t.kind, t.args);
                self.task = None;
                self.finish_task(kind, args);
                ev.push(Event::TaskDone { kind });
            }
        }
        if let Some(p) = &mut self.path {
            p.t += dt;
            let (x, y) = along(&p.waypoints, p.t / PATH_RUN_S);
            (self.pose[0], self.pose[1]) = (x, y);
            if p.t >= PATH_RUN_S {
                self.path = None;
                ev.push(Event::PathDone);
            }
        }
        ev
    }

    fn finish_task(&mut self, kind: u8, [gripper, block, column, row]: [i32; 4]) {
        let g = usize::from(gripper == 1);
        let byte = |v: i32| v.clamp(0, 255) as u8;
        match kind {
            PICK => {
                self.memory.holding[g] = true;
                self.memory.picked.push(byte(block));
            }
            PLACE => {
                self.memory.holding[g] = false;
                self.memory.placed.push((byte(column), byte(row)));
            }
            _ => {}
        }
    }

    // ------------------------------------------------------------- lifts
    pub fn lift_to(&mut self, g: usize, height: f64, req: Option<u32>) -> Vec<Event> {
        let l = &mut self.lifts[g];
        let old = std::mem::replace(l, Lift { h: l.h, goal: height, moving: true, req });
        superseded(g, old.req, "superseded")
    }

    pub fn lift_cancel(&mut self, g: usize) -> Vec<Event> {
        let l = &mut self.lifts[g];
        let req = l.req.take();
        (l.moving, l.goal) = (false, l.h);
        superseded(g, req, "canceled")
    }

    pub fn lift_zero(&mut self, g: usize) -> Vec<Event> {
        let req = self.lifts[g].req.take();
        self.lifts[g] = Lift::default();
        superseded(g, req, "zeroed")
    }

    pub fn grippers(&self) -> [Option<Gripper>; 2] {
        std::array::from_fn(|i| {
            let l = &self.lifts[i];
            Some(Gripper {
                height: l.h,
                goal: l.goal,
                moving: l.moving,
                arrived: !l.moving,
                suction: self.suction[i],
                feedback: true,
            })
        })
    }

    // -------------------------------------------------------- task / path
    pub fn start_task(&mut self, kind: u8, args: [i32; 4]) {
        self.task = Some(Task { kind, args, stage: 0, t: 0.0 });
    }

    pub fn cancel_task(&mut self) {
        self.task = None;
    }

    pub fn task_state(&self) -> Option<TaskState> {
        let t = self.task.as_ref()?;
        let step = stages(t.kind)[t.stage.min(stages(t.kind).len() - 1)];
        Some(TaskState { kind: t.kind, step: step.to_owned(), args: t.args })
    }

    pub fn memory(&self) -> &Memory {
        &self.memory
    }

    pub fn reset_memory(&mut self) {
        self.memory = Memory::default();
    }

    /// A polyline from the current position to the front of the shelf.
    pub fn start_path(&mut self, name: &str) {
        let (x0, y0) = (self.pose[0], self.pose[1]);
        let goal = (SHELF[0] - 0.5, 0.0);
        let waypoints = vec![(x0, y0), (x0 + (goal.0 - x0) * 0.5, y0), (goal.0, y0 * 0.2), goal];
        self.path = Some(PathRun { name: name.to_owned(), t: 0.0, waypoints });
    }

    /// True if a path run was active.
    pub fn cancel_path(&mut self) -> bool {
        self.path.take().is_some()
    }

    pub fn path_state(&self) -> Option<(String, bool)> {
        self.path.as_ref().map(|p| (p.name.clone(), true))
    }

    /// The running path as dense points in mm (empty without one).
    pub fn path_points(&self) -> Vec<(i16, i16)> {
        let Some(p) = &self.path else { return Vec::new() };
        (0..=24).map(|i| to_mm(along(&p.waypoints, f64::from(i) / 24.0))).collect()
    }

    // -------------------------------------------------------------- scan
    /// One ray per degree against the outer walls and the shelf box, in mm (field frame).
    pub fn scan(&self) -> Vec<(i16, i16)> {
        let [x, y, yaw] = self.pose;
        if !(BOUNDS[0] < x && x < BOUNDS[2] && BOUNDS[1] < y && y < BOUNDS[3]) {
            return Vec::new();
        }
        (0..360)
            .filter_map(|deg| {
                let a = yaw + f64::from(deg).to_radians();
                let (dy, dx) = a.sin_cos();
                let wall = ray_exit(x, y, dx, dy, &BOUNDS);
                let d = match ray_entry(x, y, dx, dy, &SHELF) {
                    Some(h) => wall.map_or(h, |w| w.min(h)),
                    None => wall?,
                };
                (d <= MAX_RANGE).then(|| to_mm((x + d * dx, y + d * dy)))
            })
            .collect()
    }
}

fn superseded(g: usize, req: Option<u32>, msg: &'static str) -> Vec<Event> {
    req.map(|req| Event::Lift { req, g, ok: false, msg }).into_iter().collect()
}

fn to_mm((x, y): (f64, f64)) -> (i16, i16) {
    ((x * 1000.0).round() as i16, (y * 1000.0).round() as i16)
}

/// The point at fraction `f` (0..=1) of a polyline's length.
fn along(w: &[(f64, f64)], f: f64) -> (f64, f64) {
    let len: Vec<f64> = w.windows(2).map(|s| (s[1].0 - s[0].0).hypot(s[1].1 - s[0].1)).collect();
    let mut left = f.clamp(0.0, 1.0) * len.iter().sum::<f64>();
    for (s, l) in w.windows(2).zip(&len) {
        if left <= *l {
            let u = if *l > 0.0 { (left / l).min(1.0) } else { 0.0 };
            return (s[0].0 + (s[1].0 - s[0].0) * u, s[0].1 + (s[1].1 - s[0].1) * u);
        }
        left -= l;
    }
    w[w.len() - 1]
}

/// Distance from an interior point to the box boundary along (dx, dy).
fn ray_exit(ox: f64, oy: f64, dx: f64, dy: f64, b: &[f64; 4]) -> Option<f64> {
    [(ox, dx, b[0], b[2]), (oy, dy, b[1], b[3])]
        .into_iter()
        .filter(|(_, d, _, _)| d.abs() >= 1e-12)
        .map(|(o, d, lo, hi)| ((if d > 0.0 { hi } else { lo }) - o) / d)
        .filter(|t| *t >= 0.0)
        .min_by(f64::total_cmp)
}

/// Distance from an outside point to where the ray enters the box.
fn ray_entry(ox: f64, oy: f64, dx: f64, dy: f64, b: &[f64; 4]) -> Option<f64> {
    let (mut near, mut far) = (f64::NEG_INFINITY, f64::INFINITY);
    for (o, d, lo, hi) in [(ox, dx, b[0], b[2]), (oy, dy, b[1], b[3])] {
        if d.abs() < 1e-12 {
            if o < lo || o > hi {
                return None;
            }
        } else {
            let (t1, t2) = ((lo - o) / d, (hi - o) / d);
            near = near.max(t1.min(t2));
            far = far.min(t1.max(t2));
        }
    }
    (near > 0.0 && near <= far).then_some(near)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scan_hits_walls_and_shelf() {
        let mut s = Sim::default();
        assert_eq!(s.scan().len(), 360);
        s.pose = [0.0, 1.0, 0.0];
        let pts = s.scan();
        // Ray 0 is straight +x: it meets the right wall at x = 2.786 (the shelf is at y <= 0.3).
        assert_eq!(pts[0], (2786, 1000));
        s.pose = [0.0, 0.0, 0.0];
        assert_eq!(s.scan()[0].0, 1563); // shelf front face (1.5625 m, rounded)
    }

    #[test]
    fn lift_moves_at_fixed_speed_and_reports() {
        let mut s = Sim::default();
        assert!(s.lift_to(0, 0.05, Some(7)).is_empty());
        assert!(s.advance(0.5).is_empty());
        let ev = s.advance(0.6);
        assert!(matches!(ev[..], [Event::Lift { req: 7, g: 0, ok: true, .. }]));
        assert!(s.lift_to(0, 0.1, Some(8)).is_empty());
        assert!(matches!(s.lift_to(0, 0.0, None)[..], [Event::Lift { req: 8, ok: false, .. }]));
    }

    #[test]
    fn path_finishes_after_three_seconds() {
        let mut s = Sim::default();
        s.start_path("slalom_forward");
        assert!(s.advance(2.9).is_empty() && s.path_points().len() == 25);
        assert!(matches!(s.advance(0.2)[..], [Event::PathDone]));
        assert!(!s.busy());
    }

    /// Numbers of `"key": ...` in config.py DEFAULTS (quoted strings skipped).
    fn defaults(key: &str) -> Vec<f64> {
        const PY: &str = include_str!("../../../ros/oumuamua_controller_bridge/oumuamua_controller_bridge/config.py");
        let from = PY.find(&format!("    \"{key}\":")).expect(key) + key.len() + 7;
        let rest = &PY[from..];
        let body = &rest[..rest.find("\n    \"").or(rest.find("\n}")).unwrap_or(rest.len())];
        let (mut plain, mut quoted) = (String::new(), false);
        for c in body.chars() {
            match c {
                '"' => quoted = !quoted,
                _ if quoted => {}
                '0'..='9' | '.' | '-' => plain.push(c),
                _ => plain.push(' '),
            }
        }
        plain.split_whitespace().filter_map(|t| t.parse().ok()).collect()
    }

    #[test]
    fn config_matches_python_defaults() {
        let flat = |v: &[[f64; 4]]| v.concat();
        assert_eq!(defaults("lift_presets"), LIFT_PRESETS.iter().map(|p| p.1).collect::<Vec<_>>());
        assert_eq!(defaults("lift_max"), [LIFT_MAX]);
        assert_eq!(defaults("speed_levels"), SPEED_LEVELS.iter().flat_map(|s| [s.0, s.1]).collect::<Vec<_>>());
        assert_eq!(defaults("blocks"), [f64::from(BLOCKS), f64::from(COLUMNS), f64::from(ROWS)]); // one line in config.py
        assert_eq!(defaults("field_bounds"), BOUNDS);
        assert_eq!(defaults("field_obstacles"), flat(&OBSTACLES));
        assert_eq!(defaults("shelf"), SHELF);
        assert_eq!(defaults("start_pose"), START);
        assert_eq!(defaults("robot_radius"), [ROBOT_RADIUS]);
    }
}
