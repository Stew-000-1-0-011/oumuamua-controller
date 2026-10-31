//! Wire format shared by the phone app and the ROS bridge (see docs/protocol.md).
//!
//! Every datagram is one MessagePack array whose first element is the kind.
//! Reals are always f64, integers use the shortest form (rmpv does this).

use rmpv::Value;

pub const PORT: u16 = 47800;
pub const BEACON_PORT: u16 = 47801;
pub const MAX_POINTS_PER_SCAN: usize = 280;

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum DecodeError {
    #[error("invalid msgpack: {0}")]
    Msgpack(String),
    #[error("datagram is not a non-empty array")]
    NotArray,
    #[error("unknown kind {0}")]
    UnknownKind(u64),
    #[error("element {0} is missing")]
    Missing(usize),
    #[error("element {index}: expected {expected}")]
    Type { index: usize, expected: &'static str },
}

type Res<T> = Result<T, DecodeError>;

// ---------------------------------------------------------------- Wire trait

/// Conversion between a Rust value and a msgpack `Value`. A decode failure
/// reports only the expected type; `Reader::get` adds the element index.
trait Wire: Sized {
    fn to_value(&self) -> Value;
    fn from_value(v: &Value) -> Result<Self, &'static str>;
}

macro_rules! wire_uint {
    ($($t:ty),*) => {$(
        impl Wire for $t {
            fn to_value(&self) -> Value { Value::from(u64::from(*self)) }
            fn from_value(v: &Value) -> Result<Self, &'static str> {
                v.as_u64().and_then(|n| Self::try_from(n).ok()).ok_or(stringify!($t))
            }
        }
    )*};
}
macro_rules! wire_int {
    ($($t:ty),*) => {$(
        impl Wire for $t {
            fn to_value(&self) -> Value { Value::from(i64::from(*self)) }
            fn from_value(v: &Value) -> Result<Self, &'static str> {
                v.as_i64().and_then(|n| Self::try_from(n).ok()).ok_or(stringify!($t))
            }
        }
    )*};
}
wire_uint!(u8, u16, u32, u64);
wire_int!(i8, i16, i32);

impl Wire for f64 {
    fn to_value(&self) -> Value {
        Value::F64(*self)
    }
    fn from_value(v: &Value) -> Result<Self, &'static str> {
        match v {
            Value::F64(x) => Ok(*x),
            Value::F32(x) => Ok(f64::from(*x)),
            Value::Integer(i) => i.as_f64().ok_or("f64"),
            _ => Err("f64"),
        }
    }
}
impl Wire for bool {
    fn to_value(&self) -> Value {
        Value::Boolean(*self)
    }
    fn from_value(v: &Value) -> Result<Self, &'static str> {
        v.as_bool().ok_or("bool")
    }
}
impl Wire for String {
    fn to_value(&self) -> Value {
        Value::from(self.as_str())
    }
    fn from_value(v: &Value) -> Result<Self, &'static str> {
        v.as_str().map(str::to_owned).ok_or("str")
    }
}
impl<T: Wire> Wire for Option<T> {
    fn to_value(&self) -> Value {
        self.as_ref().map_or(Value::Nil, T::to_value)
    }
    fn from_value(v: &Value) -> Result<Self, &'static str> {
        if v.is_nil() { Ok(None) } else { T::from_value(v).map(Some) }
    }
}
impl<T: Wire> Wire for Vec<T> {
    fn to_value(&self) -> Value {
        Value::Array(self.iter().map(T::to_value).collect())
    }
    fn from_value(v: &Value) -> Result<Self, &'static str> {
        v.as_array().ok_or("array")?.iter().map(T::from_value).collect()
    }
}
impl<T: Wire, const N: usize> Wire for [T; N] {
    fn to_value(&self) -> Value {
        Value::Array(self.iter().map(T::to_value).collect())
    }
    fn from_value(v: &Value) -> Result<Self, &'static str> {
        let items: Vec<T> = v
            .as_array()
            .ok_or("array")?
            .iter()
            .take(N)
            .map(T::from_value)
            .collect::<Result<_, _>>()?;
        items.try_into().map_err(|_| "array too short")
    }
}
impl<A: Wire, B: Wire> Wire for (A, B) {
    fn to_value(&self) -> Value {
        Value::Array(vec![self.0.to_value(), self.1.to_value()])
    }
    fn from_value(v: &Value) -> Result<Self, &'static str> {
        match v.as_array().map(Vec::as_slice) {
            Some([a, b, ..]) => Ok((A::from_value(a)?, B::from_value(b)?)),
            _ => Err("pair"),
        }
    }
}

/// Reads typed elements from an array by index.
struct Reader<'a> {
    items: &'a [Value],
}

impl Reader<'_> {
    fn raw(&self, i: usize) -> Res<&Value> {
        self.items.get(i).ok_or(DecodeError::Missing(i))
    }
    fn get<T: Wire>(&self, i: usize) -> Res<T> {
        T::from_value(self.raw(i)?).map_err(|expected| DecodeError::Type { index: i, expected })
    }
    /// An optional trailing field: `T::default()` when the sender (an older peer) left it out or sent nil.
    fn opt<T: Wire + Default>(&self, i: usize) -> Res<T> {
        match self.items.get(i) {
            None | Some(Value::Nil) => Ok(T::default()),
            Some(_) => self.get(i),
        }
    }
    fn bin(&self, i: usize) -> Res<Vec<u8>> {
        match self.raw(i)? {
            Value::Binary(b) => Ok(b.clone()),
            _ => Err(DecodeError::Type { index: i, expected: "bin" }),
        }
    }
    fn points(&self, i: usize) -> Res<Vec<(i16, i16)>> {
        let b = self.bin(i)?;
        if b.len() % 4 != 0 {
            return Err(DecodeError::Type { index: i, expected: "points (multiple of 4 bytes)" });
        }
        Ok(b.chunks_exact(4)
            .map(|c| (i16::from_le_bytes([c[0], c[1]]), i16::from_le_bytes([c[2], c[3]])))
            .collect())
    }
}

fn bin(b: &[u8]) -> Value {
    Value::Binary(b.to_vec())
}
fn points_bin(points: &[(i16, i16)]) -> Value {
    Value::Binary(points.iter().flat_map(|(x, y)| [x.to_le_bytes(), y.to_le_bytes()]).flatten().collect())
}
fn w<T: Wire>(x: &T) -> Value {
    x.to_value()
}

/// `row!(kind; a, b)` builds `[kind, a, b]`.
macro_rules! row {
    ($kind:expr; $($f:expr),*) => { Value::Array(vec![Value::from($kind as u64), $(w($f)),*]) };
}

/// Plain array-encoded struct: fields in order, starting at array index 0.
macro_rules! wire_struct {
    ($name:ident { $($f:ident),* $(,)? }) => {
        impl Wire for $name {
            fn to_value(&self) -> Value { Value::Array(vec![$(w(&self.$f)),*]) }
            fn from_value(v: &Value) -> Result<Self, &'static str> {
                let items = v.as_array().ok_or("array")?;
                let r = Reader { items };
                let mut i = 0;
                Ok($name { $($f: { i += 1; r.get(i - 1).map_err(|_| stringify!($name))? }),* })
            }
        }
    };
}

/// Like `wire_struct!`, then trailing fields added by a later protocol version
/// (`name = default`). On decode, an absent or nil trailing element takes its default, so a peer
/// that sends only the leading fields still connects. Encoding always writes every field, in
/// order, so an older decoder (which reads the leading fields and ignores the rest) still works.
macro_rules! wire_struct_ext {
    ($name:ident { $($f:ident),* ; $($g:ident = $d:expr),* $(,)? }) => {
        impl Wire for $name {
            fn to_value(&self) -> Value { Value::Array(vec![$(w(&self.$f),)* $(w(&self.$g)),*]) }
            fn from_value(v: &Value) -> Result<Self, &'static str> {
                let items = v.as_array().ok_or("array")?;
                let r = Reader { items };
                let mut i = 0;
                $(let $f = { i += 1; r.get(i - 1).map_err(|_| stringify!($name))? };)*
                $(let $g = {
                    i += 1;
                    match items.get(i - 1) {
                        None | Some(Value::Nil) => $d,
                        Some(_) => r.get(i - 1).map_err(|_| stringify!($name))?,
                    }
                };)*
                Ok($name { $($f,)* $($g),* })
            }
        }
    };
}

// ------------------------------------------------------------------- State

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Manual = 0,
    Auto = 1,
    Stopped = 2,
}
impl Wire for Mode {
    fn to_value(&self) -> Value {
        Value::from(*self as u64)
    }
    fn from_value(v: &Value) -> Result<Self, &'static str> {
        match v.as_u64() {
            Some(0) => Ok(Mode::Manual),
            Some(1) => Ok(Mode::Auto),
            Some(2) => Ok(Mode::Stopped),
            _ => Err("mode 0..=2"),
        }
    }
}

/// Motor `mode` values (mini_shirasu_ros/msg/Status), plus `MOTOR_MODE_UNKNOWN` for a peer that
/// sent no detail.
pub const MOTOR_MODE_DISABLED: u8 = 0;
pub const MOTOR_MODE_CURRENT: u8 = 1;
pub const MOTOR_MODE_VELOCITY: u8 = 2;
pub const MOTOR_MODE_POSITION: u8 = 3;
pub const MOTOR_MODE_FAULT: u8 = 4;
pub const MOTOR_MODE_UNKNOWN: u8 = 255;
/// Bits of `Wheel::flags`.
pub const MOTOR_SATURATED_UPPER: u8 = 1;
pub const MOTOR_SATURATED_LOWER: u8 = 2;
pub const MOTOR_VDC_LOW: u8 = 4;

/// One mini-shirasu motor board: the three wheels, then the two suction motors.
/// Wire: `[vdc, ok, current, velocity, mode, flags, name]`; only the first two are required.
#[derive(Debug, Clone, PartialEq)]
pub struct Wheel {
    /// Bus voltage [V].
    pub vdc: f64,
    /// Status is arriving and the board is not in fault.
    pub ok: bool,
    /// Motor current [A].
    pub current: f64,
    /// Angular velocity [rad/s].
    pub velocity: f64,
    /// `MOTOR_MODE_*`.
    pub mode: u8,
    /// `MOTOR_*` flag bits (saturation, low bus voltage).
    pub flags: u8,
    /// Label for the UI ("車輪0", "吸引 左"); empty from an older peer.
    pub name: String,
}
wire_struct_ext!(Wheel { vdc, ok; current = 0.0, velocity = 0.0, mode = MOTOR_MODE_UNKNOWN, flags = 0, name = String::new() });

impl Wheel {
    /// A board with only the legacy fields (voltage and ok), as an older peer sends it.
    pub fn legacy(vdc: f64, ok: bool) -> Wheel {
        Wheel { vdc, ok, current: 0.0, velocity: 0.0, mode: MOTOR_MODE_UNKNOWN, flags: 0, name: String::new() }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gripper {
    pub height: f64,
    pub goal: f64,
    pub moving: bool,
    pub arrived: bool,
    pub suction: bool,
    pub feedback: bool,
}
wire_struct!(Gripper { height, goal, moving, arrived, suction, feedback });

#[derive(Debug, Clone, PartialEq)]
pub struct TaskState {
    pub kind: u8,
    pub step: String,
    pub args: [i32; 4],
}
wire_struct!(TaskState { kind, step, args });

#[derive(Debug, Clone, PartialEq)]
pub struct State {
    pub mode: Mode,
    pub stop_reason: String,
    pub pose: Option<[f64; 3]>,
    pub vel: Option<[f64; 3]>,
    pub wheels: Vec<Wheel>,
    pub grippers: [Option<Gripper>; 2],
    pub task: Option<TaskState>,
    pub path: Option<(String, bool)>,
    pub tracker_enabled: bool,
    /// The last `/harite/command` [rad/s] the bridge saw; `None` when unknown or from an older bridge.
    pub harite_command: Option<f64>,
}
wire_struct_ext!(State { mode, stop_reason, pose, vel, wheels, grippers, task, path, tracker_enabled; harite_command = None });

#[derive(Debug, Clone, PartialEq)]
pub struct Control {
    pub session: u32,
    pub seq: u32,
    pub t_ms: u32,
    pub vx: f64,
    pub vy: f64,
    pub wz: f64,
    pub field_frame: bool,
    pub jog_left: i8,
    pub jog_right: i8,
    pub estop: bool,
    /// Harite (lift without suction): +1 up, -1 down, 0 released. Trailing and optional: an older
    /// phone does not send it (0), an older bridge ignores it.
    pub jog_harite: i8,
}

// --------------------------------------------------------------------- Msg

#[derive(Debug, Clone, PartialEq)]
pub enum Msg {
    ReleaseStop { req: u32 },
    SetSuction { req: u32, gripper: u8, on: bool },
    LiftTo { req: u32, gripper: u8, height: f64 },
    LiftZero { req: u32, gripper: u8 },
    RunTask { req: u32, kind: u8, gripper: i32, block: i32, column: i32, row: i32 },
    Cancel { req: u32 },
    StartPath { req: u32, name: String },
    SetTracker { req: u32, on: bool },
    SetPose { req: u32, x: f64, y: f64, yaw: f64 },
    TaskReset { req: u32 },
    LogLevel { req: u32, min_level: u8 },
    ShellOpen { req: u32, tab: String },
    ShellInput { req: u32, tab: String, data: Vec<u8> },
    ShellComplete { req: u32, tab: String, line: String, cursor: u32 },
    ShellClose { req: u32, tab: String },
    ShellRename { req: u32, tab: String, new_name: String },
    Result { req: u32, ok: bool, message: String },
    Config {
        robot_name: String,
        paths: Vec<String>,
        lift_presets: Vec<(String, f64)>,
        lift_max: f64,
        speed_levels: [(f64, f64); 3],
        blocks: u8,
        columns: u8,
        rows: u8,
        /// The bridge drives harite (jog controls shown). Trailing and optional: false from an older bridge.
        harite: bool,
    },
    /// `footprint` (robot-frame polygon, x forward, y left) and `suction` (the two suction centres, gripper0
    /// first) are empty from a peer that does not send them: the map then draws a `robot_radius` circle.
    Field {
        bounds: [f64; 4],
        obstacles: Vec<[f64; 4]>,
        shelf: [f64; 4],
        start: [f64; 3],
        robot_radius: f64,
        footprint: Vec<(f64, f64)>,
        suction: Vec<(f64, f64)>,
    },
    PathShape { name: String, points: Vec<(i16, i16)> },
    TaskMemory { holding: [bool; 2], picked: Vec<u8>, placed: Vec<(u8, u8)> },
    TaskDone { kind: u8, ok: bool, message: String },
    Log { stamp_ms: u64, level: u8, name: String, text: String },
    ShellOpened { tab: String, replay: Vec<u8> },
    ShellOutput { tab: String, data: Vec<u8> },
    ShellCompletion { req: u32, tab: String, start: u32, candidates: Vec<String> },
    ShellClosed { tab: String, reason: String },
    /// Unrecognised inner kind (decode only; protocol.md 1: unknown kinds are dropped,
    /// but a REL carrying one must still be ACKed). Encodes as `[kind]`.
    Unknown { kind: u8 },
}

impl Msg {
    fn to_value(&self) -> Value {
        use Msg::*;
        match self {
            ReleaseStop { req } => row!(10; req),
            SetSuction { req, gripper, on } => row!(11; req, gripper, on),
            LiftTo { req, gripper, height } => row!(12; req, gripper, height),
            LiftZero { req, gripper } => row!(13; req, gripper),
            RunTask { req, kind, gripper, block, column, row } => {
                row!(14; req, kind, gripper, block, column, row)
            }
            Cancel { req } => row!(15; req),
            StartPath { req, name } => row!(16; req, name),
            SetTracker { req, on } => row!(17; req, on),
            SetPose { req, x, y, yaw } => row!(18; req, x, y, yaw),
            TaskReset { req } => row!(19; req),
            LogLevel { req, min_level } => row!(20; req, min_level),
            ShellOpen { req, tab } => row!(30; req, tab),
            ShellInput { req, tab, data } => row!(31; req, tab, &Raw(bin(data))),
            ShellComplete { req, tab, line, cursor } => row!(32; req, tab, line, cursor),
            ShellClose { req, tab } => row!(33; req, tab),
            ShellRename { req, tab, new_name } => row!(34; req, tab, new_name),
            Result { req, ok, message } => row!(50; req, ok, message),
            Config { robot_name, paths, lift_presets, lift_max, speed_levels, blocks, columns, rows, harite } => {
                row!(51; robot_name, paths, lift_presets, lift_max, speed_levels, blocks, columns, rows, harite)
            }
            Field { bounds, obstacles, shelf, start, robot_radius, footprint, suction } => {
                row!(52; bounds, obstacles, shelf, start, robot_radius, footprint, suction)
            }
            PathShape { name, points } => row!(53; name, &Raw(points_bin(points))),
            TaskMemory { holding, picked, placed } => row!(54; holding, picked, placed),
            TaskDone { kind, ok, message } => row!(55; kind, ok, message),
            Log { stamp_ms, level, name, text } => row!(56; stamp_ms, level, name, text),
            ShellOpened { tab, replay } => row!(60; tab, &Raw(bin(replay))),
            ShellOutput { tab, data } => row!(61; tab, &Raw(bin(data))),
            ShellCompletion { req, tab, start, candidates } => row!(62; req, tab, start, candidates),
            ShellClosed { tab, reason } => row!(63; tab, reason),
            Unknown { kind } => row!(*kind;),
        }
    }

    fn from_value(v: &Value) -> Res<Msg> {
        use Msg::*;
        let items = v.as_array().ok_or(DecodeError::NotArray)?;
        let r = Reader { items };
        let kind: u64 = r.get(0)?;
        Ok(match kind {
            10 => ReleaseStop { req: r.get(1)? },
            11 => SetSuction { req: r.get(1)?, gripper: r.get(2)?, on: r.get(3)? },
            12 => LiftTo { req: r.get(1)?, gripper: r.get(2)?, height: r.get(3)? },
            13 => LiftZero { req: r.get(1)?, gripper: r.get(2)? },
            14 => RunTask {
                req: r.get(1)?,
                kind: r.get(2)?,
                gripper: r.get(3)?,
                block: r.get(4)?,
                column: r.get(5)?,
                row: r.get(6)?,
            },
            15 => Cancel { req: r.get(1)? },
            16 => StartPath { req: r.get(1)?, name: r.get(2)? },
            17 => SetTracker { req: r.get(1)?, on: r.get(2)? },
            18 => SetPose { req: r.get(1)?, x: r.get(2)?, y: r.get(3)?, yaw: r.get(4)? },
            19 => TaskReset { req: r.get(1)? },
            20 => LogLevel { req: r.get(1)?, min_level: r.get(2)? },
            30 => ShellOpen { req: r.get(1)?, tab: r.get(2)? },
            31 => ShellInput { req: r.get(1)?, tab: r.get(2)?, data: r.bin(3)? },
            32 => ShellComplete { req: r.get(1)?, tab: r.get(2)?, line: r.get(3)?, cursor: r.get(4)? },
            33 => ShellClose { req: r.get(1)?, tab: r.get(2)? },
            34 => ShellRename { req: r.get(1)?, tab: r.get(2)?, new_name: r.get(3)? },
            50 => Result { req: r.get(1)?, ok: r.get(2)?, message: r.get(3)? },
            51 => Config {
                robot_name: r.get(1)?,
                paths: r.get(2)?,
                lift_presets: r.get(3)?,
                lift_max: r.get(4)?,
                speed_levels: r.get(5)?,
                blocks: r.get(6)?,
                columns: r.get(7)?,
                rows: r.get(8)?,
                harite: r.opt(9)?,
            },
            52 => Field {
                bounds: r.get(1)?,
                obstacles: r.get(2)?,
                shelf: r.get(3)?,
                start: r.get(4)?,
                robot_radius: r.get(5)?,
                footprint: r.opt(6)?,
                suction: r.opt(7)?,
            },
            53 => PathShape { name: r.get(1)?, points: r.points(2)? },
            54 => TaskMemory { holding: r.get(1)?, picked: r.get(2)?, placed: r.get(3)? },
            55 => TaskDone { kind: r.get(1)?, ok: r.get(2)?, message: r.get(3)? },
            56 => Log { stamp_ms: r.get(1)?, level: r.get(2)?, name: r.get(3)?, text: r.get(4)? },
            60 => ShellOpened { tab: r.get(1)?, replay: r.bin(2)? },
            61 => ShellOutput { tab: r.get(1)?, data: r.bin(2)? },
            62 => ShellCompletion {
                req: r.get(1)?,
                tab: r.get(2)?,
                start: r.get(3)?,
                candidates: r.get(4)?,
            },
            63 => ShellClosed { tab: r.get(1)?, reason: r.get(2)? },
            k => Unknown { kind: u8::try_from(k).map_err(|_| DecodeError::UnknownKind(k))? },
        })
    }
}

/// An already-built `Value` (used for bin fields) passed through `row!`.
struct Raw(Value);
impl Wire for Raw {
    fn to_value(&self) -> Value {
        self.0.clone()
    }
    fn from_value(v: &Value) -> Result<Self, &'static str> {
        Ok(Raw(v.clone()))
    }
}

// ---------------------------------------------------------------- Datagram

#[derive(Debug, Clone, PartialEq)]
pub enum Datagram {
    Hello { session: u32, client_name: String, takeover: bool },
    Welcome { session: u32, robot_name: String, accepted: bool, reason: String },
    Control(Control),
    State { session: u32, seq: u32, ack_seq: u32, ack_t_ms: u32, body: State },
    Scan { session: u32, scan_id: u32, frag: u8, frag_count: u8, points: Vec<(i16, i16)> },
    Rel { session: u32, seq: u32, msg: Msg },
    Ack { session: u32, cum: u32 },
    Beacon { robot_name: String, port: u16 },
    Bye { session: u32, reason: String },
}

impl Datagram {
    pub fn encode(&self) -> Vec<u8> {
        let v = match self {
            Datagram::Hello { session, client_name, takeover } => {
                row!(0; session, client_name, takeover, &Raw(Value::Nil))
            }
            Datagram::Welcome { session, robot_name, accepted, reason } => {
                row!(1; session, robot_name, accepted, reason)
            }
            Datagram::Control(c) => row!(2; &c.session, &c.seq, &c.t_ms, &c.vx, &c.vy, &c.wz,
                &c.field_frame, &c.jog_left, &c.jog_right, &c.estop, &c.jog_harite),
            Datagram::State { session, seq, ack_seq, ack_t_ms, body } => {
                row!(3; session, seq, ack_seq, ack_t_ms, body)
            }
            Datagram::Scan { session, scan_id, frag, frag_count, points } => {
                row!(4; session, scan_id, frag, frag_count, &Raw(points_bin(points)))
            }
            Datagram::Rel { session, seq, msg } => row!(5; session, seq, &Raw(msg.to_value())),
            Datagram::Ack { session, cum } => row!(6; session, cum),
            Datagram::Beacon { robot_name, port } => row!(7; robot_name, port),
            Datagram::Bye { session, reason } => row!(8; session, reason),
        };
        let mut out = Vec::new();
        rmpv::encode::write_value(&mut out, &v).expect("writing to a Vec cannot fail");
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Datagram, DecodeError> {
        let v = rmpv::decode::read_value(&mut &bytes[..])
            .map_err(|e| DecodeError::Msgpack(e.to_string()))?;
        let items = v.as_array().filter(|a| !a.is_empty()).ok_or(DecodeError::NotArray)?;
        let r = Reader { items };
        let kind: u64 = r.get(0)?;
        Ok(match kind {
            0 => Datagram::Hello { session: r.get(1)?, client_name: r.get(2)?, takeover: r.get(3)? },
            1 => Datagram::Welcome {
                session: r.get(1)?,
                robot_name: r.get(2)?,
                accepted: r.get(3)?,
                reason: r.get(4)?,
            },
            2 => Datagram::Control(Control {
                session: r.get(1)?,
                seq: r.get(2)?,
                t_ms: r.get(3)?,
                vx: r.get(4)?,
                vy: r.get(5)?,
                wz: r.get(6)?,
                field_frame: r.get(7)?,
                jog_left: r.get(8)?,
                jog_right: r.get(9)?,
                estop: r.get(10)?,
                jog_harite: r.opt(11)?,
            }),
            3 => Datagram::State {
                session: r.get(1)?,
                seq: r.get(2)?,
                ack_seq: r.get(3)?,
                ack_t_ms: r.get(4)?,
                body: r.get(5)?,
            },
            4 => Datagram::Scan {
                session: r.get(1)?,
                scan_id: r.get(2)?,
                frag: r.get(3)?,
                frag_count: r.get(4)?,
                points: r.points(5)?,
            },
            5 => Datagram::Rel {
                session: r.get(1)?,
                seq: r.get(2)?,
                msg: Msg::from_value(r.raw(3)?).map_err(|e| match e {
                    DecodeError::Type { expected, .. } => DecodeError::Type { index: 3, expected },
                    e => e,
                })?,
            },
            6 => Datagram::Ack { session: r.get(1)?, cum: r.get(2)? },
            7 => Datagram::Beacon { robot_name: r.get(1)?, port: r.get(2)? },
            8 => Datagram::Bye { session: r.get(1)?, reason: r.get(2)? },
            k => return Err(DecodeError::UnknownKind(k)),
        })
    }
}

/// Convert scan/path points from millimetres to metres.
pub fn points_to_m(points: &[(i16, i16)]) -> Vec<(f32, f32)> {
    points.iter().map(|&(x, y)| (f32::from(x) / 1000.0, f32::from(y) / 1000.0)).collect()
}
