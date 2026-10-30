use proto::Datagram;
use std::path::PathBuf;

fn golden_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/golden")
}

fn read(name: &str) -> Vec<u8> {
    std::fs::read(golden_dir().join(format!("{name}.msgpack"))).unwrap()
}

#[test]
fn every_golden_file_roundtrips() {
    let mut n = 0;
    for entry in std::fs::read_dir(golden_dir()).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().is_none_or(|e| e != "msgpack") {
            continue;
        }
        let bytes = std::fs::read(&path).unwrap();
        let d = Datagram::decode(&bytes).unwrap_or_else(|e| panic!("{path:?}: {e}"));
        assert_eq!(d.encode(), bytes, "{path:?}");
        n += 1;
    }
    assert!(n > 0);
}

#[test]
fn golden_set_is_complete() {
    let n = std::fs::read_dir(golden_dir()).unwrap().count();
    assert!(n >= 9 + 27, "only {n} golden files");
}

#[test]
fn extra_trailing_fields_are_ignored() {
    let mut v = rmpv::decode::read_value(&mut &read("control")[..]).unwrap();
    let rmpv::Value::Array(a) = &mut v else { panic!() };
    a.push(rmpv::Value::from(99));
    let mut bytes = Vec::new();
    rmpv::encode::write_value(&mut bytes, &v).unwrap();
    assert_eq!(Datagram::decode(&bytes).unwrap(), Datagram::decode(&read("control")).unwrap());
}

#[test]
fn wrong_types_are_errors() {
    let mut v = rmpv::decode::read_value(&mut &read("control")[..]).unwrap();
    let rmpv::Value::Array(a) = &mut v else { panic!() };
    a[4] = rmpv::Value::from("fast");
    let mut bytes = Vec::new();
    rmpv::encode::write_value(&mut bytes, &v).unwrap();
    assert!(Datagram::decode(&bytes).is_err());
}

#[test]
fn integers_accept_any_width() {
    // ack with cum encoded as uint32 (0xce) instead of fixint
    let bytes = [0x93, 0x06, 0x05, 0xce, 0, 0, 0, 5];
    assert_eq!(Datagram::decode(&bytes).unwrap(), Datagram::Ack { session: 5, cum: 5 });
}

#[test]
fn points_convert_to_meters() {
    assert_eq!(proto::points_to_m(&[(1500, -250)]), vec![(1.5, -0.25)]);
}

fn field_items(bytes: &[u8]) -> Vec<rmpv::Value> {
    let v = rmpv::decode::read_value(&mut &bytes[..]).unwrap();
    let rmpv::Value::Array(rel) = v else { panic!() };
    let rmpv::Value::Array(msg) = rel[3].clone() else { panic!() };
    msg
}

fn encode_field(items: Vec<rmpv::Value>) -> Datagram {
    let mut rel = rmpv::decode::read_value(&mut &read("rel_field")[..]).unwrap();
    let rmpv::Value::Array(a) = &mut rel else { panic!() };
    a[3] = rmpv::Value::Array(items);
    let mut bytes = Vec::new();
    rmpv::encode::write_value(&mut bytes, &rel).unwrap();
    Datagram::decode(&bytes).unwrap()
}

fn field_of(d: Datagram) -> proto::Msg {
    let Datagram::Rel { msg, .. } = d else { panic!() };
    msg
}

#[test]
fn field_from_an_older_bridge_has_no_footprint_and_falls_back_to_the_circle() {
    // [52, bounds, obstacles, shelf, start, robot_radius]: the layout before the footprint was added.
    let mut items = field_items(&read("rel_field"));
    assert_eq!(items.len(), 8, "the golden FIELD carries footprint and suction");
    items.truncate(6);
    let proto::Msg::Field { robot_radius, footprint, suction, .. } = field_of(encode_field(items.clone())) else { panic!() };
    assert_eq!(robot_radius, 0.375);
    assert!(footprint.is_empty() && suction.is_empty());
    // nil is the same as absent.
    items.extend([rmpv::Value::Nil, rmpv::Value::Nil]);
    let proto::Msg::Field { footprint, suction, .. } = field_of(encode_field(items)) else { panic!() };
    assert!(footprint.is_empty() && suction.is_empty());
}

#[test]
fn field_with_a_footprint_roundtrips_and_extra_fields_after_it_are_ignored() {
    let proto::Msg::Field { footprint, suction, .. } = field_of(Datagram::decode(&read("rel_field")).unwrap()) else { panic!() };
    assert_eq!(footprint.len(), 4);
    assert_eq!(suction, vec![(0.25, 0.125), (0.25, -0.125)]);
    let mut items = field_items(&read("rel_field"));
    items.push(rmpv::Value::from("future"));
    let proto::Msg::Field { footprint: f2, .. } = field_of(encode_field(items)) else { panic!() };
    assert_eq!(f2, footprint);
}

#[test]
fn a_wrongly_typed_footprint_is_an_error() {
    let mut items = field_items(&read("rel_field"));
    items[6] = rmpv::Value::from("not a list");
    let mut rel = rmpv::decode::read_value(&mut &read("rel_field")[..]).unwrap();
    let rmpv::Value::Array(a) = &mut rel else { panic!() };
    a[3] = rmpv::Value::Array(items);
    let mut bytes = Vec::new();
    rmpv::encode::write_value(&mut bytes, &rel).unwrap();
    assert!(Datagram::decode(&bytes).is_err());
}

#[test]
fn task_memory_keeps_the_placed_field_on_the_wire() {
    let proto::Msg::TaskMemory { placed, .. } = field_of(Datagram::decode(&read("rel_task_memory")).unwrap()) else { panic!() };
    assert_eq!(placed, vec![(1, 2), (0, 0)], "still decoded; the phone ignores it");
}
