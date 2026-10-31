//! The harite fields are trailing and optional: CONTROL.jog_harite, STATE.harite_command and the
//! CONFIG harite flag. A peer one version apart in either direction must still interoperate.

use proto::*;
use rmpv::Value;

fn golden(name: &str) -> Vec<u8> {
    std::fs::read(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/golden").join(format!("{name}.msgpack"))).unwrap()
}

fn items(bytes: &[u8]) -> Vec<Value> {
    match rmpv::decode::read_value(&mut &bytes[..]).unwrap() {
        Value::Array(a) => a,
        v => panic!("{v:?}"),
    }
}

fn bytes(items: Vec<Value>) -> Vec<u8> {
    let mut out = Vec::new();
    rmpv::encode::write_value(&mut out, &Value::Array(items)).unwrap();
    out
}

fn control_of(bytes: &[u8]) -> Control {
    match Datagram::decode(bytes).unwrap() {
        Datagram::Control(c) => c,
        d => panic!("{d:?}"),
    }
}

fn sample_control(jog_harite: i8) -> Control {
    Control { session: 9, seq: 3, t_ms: 60, vx: 0.5, vy: 0.0, wz: -1.0, field_frame: true, jog_left: 1, jog_right: -1, estop: false, jog_harite }
}

// ----------------------------------------------------------------------------------------- CONTROL
#[test]
fn control_carries_jog_harite_and_roundtrips() {
    for j in [-1, 0, 1] {
        let c = sample_control(j);
        assert_eq!(control_of(&Datagram::Control(c.clone()).encode()), c);
    }
    assert_eq!(control_of(&golden("control")).jog_harite, 1, "the golden sample holds ▲");
}

#[test]
fn control_from_an_older_phone_has_no_jog_harite_and_reads_as_released() {
    let mut v = items(&Datagram::Control(sample_control(1)).encode());
    assert_eq!(v.len(), 12);
    v.truncate(11);
    let c = control_of(&bytes(v.clone()));
    assert_eq!(c.jog_harite, 0);
    assert_eq!((c.jog_left, c.jog_right, c.vx), (1, -1, 0.5), "the rest is unchanged");
    v.push(Value::Nil);
    assert_eq!(control_of(&bytes(v)).jog_harite, 0, "nil is the same as absent");
}

#[test]
fn control_to_an_older_bridge_keeps_the_leading_eleven_elements_unchanged() {
    // An older decoder reads elements 0..=10 and ignores the rest.
    let new = items(&Datagram::Control(sample_control(1)).encode());
    let old = items(&Datagram::Control(sample_control(0)).encode());
    assert_eq!(new[..11], old[..11]);
    assert_eq!(new[11], Value::from(1));
    // A future field after jog_harite is ignored too.
    let mut v = new;
    v.push(Value::from("future"));
    assert_eq!(control_of(&bytes(v)).jog_harite, 1);
}

#[test]
fn a_wrongly_typed_jog_harite_is_an_error() {
    let mut v = items(&Datagram::Control(sample_control(1)).encode());
    v[11] = Value::from("up");
    assert!(Datagram::decode(&bytes(v)).is_err());
}

// ------------------------------------------------------------------------------------------ STATE
fn state_of(bytes: &[u8]) -> State {
    match Datagram::decode(bytes).unwrap() {
        Datagram::State { body, .. } => body,
        d => panic!("{d:?}"),
    }
}

#[test]
fn state_roundtrips_with_and_without_a_harite_command() {
    let full = state_of(&golden("state_full"));
    assert_eq!(full.harite_command, Some(-12.5));
    let empty = state_of(&golden("state_empty"));
    assert_eq!(empty.harite_command, None, "nil = unknown");
}

#[test]
fn state_from_an_older_bridge_has_no_harite_command() {
    let mut v = items(&golden("state_full"));
    let Value::Array(body) = &mut v[5] else { panic!() };
    assert_eq!(body.len(), 10);
    body.truncate(9);
    let st = state_of(&bytes(v));
    assert_eq!(st.harite_command, None);
    assert!(st.tracker_enabled, "the leading fields are read as before");
}

#[test]
fn state_to_an_older_phone_keeps_the_leading_nine_elements() {
    let mut st = state_of(&golden("state_full"));
    let with = Datagram::State { session: 1, seq: 1, ack_seq: 0, ack_t_ms: 0, body: st.clone() }.encode();
    st.harite_command = None;
    let without = Datagram::State { session: 1, seq: 1, ack_seq: 0, ack_t_ms: 0, body: st }.encode();
    let (a, b) = (items(&with), items(&without));
    let (Value::Array(a), Value::Array(b)) = (&a[5], &b[5]) else { panic!() };
    assert_eq!(a.len(), 10);
    assert_eq!(a[..9], b[..9]);
    assert_eq!(b[9], Value::Nil);
}

#[test]
fn state_with_a_wrongly_typed_harite_command_is_an_error() {
    let mut v = items(&golden("state_full"));
    let Value::Array(body) = &mut v[5] else { panic!() };
    body[9] = Value::from("fast");
    assert!(Datagram::decode(&bytes(v)).is_err());
}

// ------------------------------------------------------------------------------------------ CONFIG
fn config_items() -> Vec<Value> {
    let v = items(&golden("rel_config"));
    match &v[3] {
        Value::Array(m) => m.clone(),
        v => panic!("{v:?}"),
    }
}

fn config_with(msg: Vec<Value>) -> Msg {
    let mut rel = items(&golden("rel_config"));
    rel[3] = Value::Array(msg);
    match Datagram::decode(&bytes(rel)).unwrap() {
        Datagram::Rel { msg, .. } => msg,
        d => panic!("{d:?}"),
    }
}

#[test]
fn config_flags_harite_and_an_older_bridge_means_hidden() {
    let mut m = config_items();
    assert_eq!(m.len(), 10);
    let Msg::Config { harite, rows, .. } = config_with(m.clone()) else { panic!() };
    assert!(harite);
    m.truncate(9);
    let Msg::Config { harite, rows: rows_old, .. } = config_with(m.clone()) else { panic!() };
    assert!(!harite, "no element: the controls stay hidden");
    assert_eq!(rows, rows_old);
    m.push(Value::Nil);
    let Msg::Config { harite, .. } = config_with(m.clone()) else { panic!() };
    assert!(!harite);
    m[9] = Value::Boolean(false);
    let Msg::Config { harite, .. } = config_with(m) else { panic!() };
    assert!(!harite);
}

#[test]
fn config_to_an_older_phone_keeps_the_leading_nine_elements() {
    let rel = Datagram::decode(&golden("rel_config")).unwrap();
    let v = items(&rel.encode());
    let Value::Array(m) = &v[3] else { panic!() };
    assert_eq!(m.len(), 10);
    assert_eq!(m[..9], config_items()[..9]);
}
