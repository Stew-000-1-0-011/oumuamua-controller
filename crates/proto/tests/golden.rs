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
