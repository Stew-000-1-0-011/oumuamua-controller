//! The motor (wheel / suction) entry of STATE: round trip, size, and compatibility with a peer
//! one version apart in either direction.

use proto::*;
use rmpv::Value;

fn motor(i: usize) -> Wheel {
    Wheel {
        vdc: 24.0 - i as f64 * 0.37,
        ok: i != 3,
        current: -1.2345 * i as f64,
        velocity: 12.5 + i as f64,
        mode: [MOTOR_MODE_VELOCITY, MOTOR_MODE_CURRENT, MOTOR_MODE_FAULT, MOTOR_MODE_DISABLED, MOTOR_MODE_POSITION][i],
        flags: [0, MOTOR_SATURATED_UPPER, MOTOR_SATURATED_LOWER, MOTOR_VDC_LOW, 7][i],
        name: ["車輪0", "車輪1", "車輪2", "吸引 左", "吸引 右"][i].to_owned(),
    }
}

fn state(wheels: Vec<Wheel>) -> State {
    State {
        mode: Mode::Manual,
        stop_reason: String::new(),
        pose: Some([1.5, -0.25, 3.0]),
        vel: Some([0.5, -0.5, 0.125]),
        wheels,
        grippers: [None, None],
        task: None,
        path: None,
        tracker_enabled: false,
    }
}

fn dg(body: State) -> Datagram {
    Datagram::State { session: 1, seq: 2, ack_seq: 3, ack_t_ms: 4, body }
}

fn raw(v: &Value) -> Vec<u8> {
    let mut out = Vec::new();
    rmpv::encode::write_value(&mut out, v).unwrap();
    out
}

#[test]
fn five_motors_roundtrip() {
    let d = dg(state((0..5).map(motor).collect()));
    assert_eq!(Datagram::decode(&d.encode()).unwrap(), d);
}

#[test]
fn state_with_five_motors_fits_one_datagram_even_when_full() {
    let mut body = state((0..5).map(motor).collect());
    let g = Gripper { height: 0.123456789, goal: 0.35, moving: true, arrived: false, suction: true, feedback: true };
    body.grippers = [Some(g), Some(g)];
    body.stop_reason = "link lost".repeat(4);
    // Generous maxima for the free-form strings (multi-byte on purpose).
    body.task = Some(TaskState { kind: 5, step: "差し込み".repeat(25), args: [-1, -1, -1, -1] });
    body.path = Some(("slalom_forward_".repeat(4), true));
    body.tracker_enabled = true;
    let n = dg(body).encode().len();
    assert!(n <= 1200, "STATE is {n} bytes");
    // The motors alone stay small, so a few more would still fit.
    let with = dg(state((0..5).map(motor).collect())).encode().len();
    let without = dg(state(vec![])).encode().len();
    assert!(with - without <= 300, "five motors take {} bytes", with - without);
}

#[test]
fn a_two_element_entry_from_an_older_peer_gets_defaults() {
    let old = Value::Array(vec![
        Value::from(3u64), Value::from(1u64), Value::from(2u64), Value::from(3u64), Value::from(4u64),
        Value::Array(vec![
            Value::from(2u64), Value::from(""), Value::Nil, Value::Nil,
            Value::Array(vec![
                Value::Array(vec![Value::F64(12.5), Value::Boolean(true)]),
                Value::Array(vec![Value::F64(-3.75), Value::Boolean(false)]),
            ]),
            Value::Array(vec![Value::Nil, Value::Nil]), Value::Nil, Value::Nil, Value::Boolean(false),
        ]),
    ]);
    let Datagram::State { body, .. } = Datagram::decode(&raw(&old)).unwrap() else { panic!() };
    assert_eq!(body.wheels, vec![Wheel::legacy(12.5, true), Wheel::legacy(-3.75, false)]);
    assert_eq!(body.wheels[0].mode, MOTOR_MODE_UNKNOWN, "no detail: the UI shows dashes");
    assert_eq!(body.wheels[0].name, "");
}

#[test]
fn a_partly_extended_entry_fills_only_the_missing_tail() {
    let v = Value::Array(vec![Value::F64(24.0), Value::Boolean(true), Value::F64(1.5), Value::F64(-2.0), Value::from(2u64)]);
    let w = <Wheel as TestWire>::decode(&v).unwrap();
    assert_eq!((w.current, w.velocity, w.mode, w.flags, w.name.as_str()), (1.5, -2.0, 2, 0, ""));
    // A nil tail element also means "not sent".
    let v = Value::Array(vec![Value::F64(24.0), Value::Boolean(true), Value::Nil, Value::Nil, Value::Nil, Value::Nil, Value::Nil]);
    assert_eq!(<Wheel as TestWire>::decode(&v).unwrap(), Wheel::legacy(24.0, true));
}

#[test]
fn a_wrong_type_in_the_required_or_extended_part_is_an_error() {
    for bad in [
        vec![Value::from("x"), Value::Boolean(true)],
        vec![Value::F64(1.0), Value::from(1u64)],
        vec![Value::F64(1.0), Value::Boolean(true), Value::from("A")],
        vec![Value::F64(1.0), Value::Boolean(true), Value::F64(0.0), Value::F64(0.0), Value::from(300u64)],
    ] {
        assert!(<Wheel as TestWire>::decode(&Value::Array(bad)).is_err());
    }
    assert!(<Wheel as TestWire>::decode(&Value::Array(vec![Value::F64(1.0)])).is_err());
}

#[test]
fn the_new_entry_starts_with_the_legacy_fields_so_an_older_decoder_still_reads_it() {
    // An older decoder reads elements 0 and 1 of each entry and ignores the rest.
    let d = dg(state((0..5).map(motor).collect()));
    let v = rmpv::decode::read_value(&mut &d.encode()[..]).unwrap();
    let body = v.as_array().unwrap()[5].as_array().unwrap();
    let wheels = body[4].as_array().unwrap();
    assert_eq!(wheels.len(), 5);
    for (i, w) in wheels.iter().enumerate() {
        let w = w.as_array().unwrap();
        assert_eq!(w.len(), 7);
        assert_eq!(w[0], Value::F64(motor(i).vdc));
        assert_eq!(w[1], Value::Boolean(motor(i).ok));
    }
}

/// Decodes a lone `Wheel` through a STATE datagram (the real path), without exposing `Wire`.
trait TestWire: Sized {
    fn decode(v: &Value) -> Result<Self, DecodeError>;
}
impl TestWire for Wheel {
    fn decode(v: &Value) -> Result<Wheel, DecodeError> {
        let msg = Value::Array(vec![
            Value::from(3u64), Value::from(1u64), Value::from(1u64), Value::from(0u64), Value::from(0u64),
            Value::Array(vec![
                Value::from(2u64), Value::from(""), Value::Nil, Value::Nil, Value::Array(vec![v.clone()]),
                Value::Array(vec![Value::Nil, Value::Nil]), Value::Nil, Value::Nil, Value::Boolean(false),
            ]),
        ]);
        match Datagram::decode(&raw(&msg))? {
            Datagram::State { mut body, .. } => Ok(body.wheels.remove(0)),
            _ => unreachable!(),
        }
    }
}
