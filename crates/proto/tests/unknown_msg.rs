use proto::{Datagram, Msg};

#[test]
fn unknown_inner_kind_decodes_as_unknown() {
    // [5, session=7, seq=3, [99, 1, "x"]]
    let mut bytes = Vec::new();
    let v = rmpv::Value::Array(vec![
        5.into(),
        7.into(),
        3.into(),
        rmpv::Value::Array(vec![99.into(), 1.into(), "x".into()]),
    ]);
    rmpv::encode::write_value(&mut bytes, &v).unwrap();
    assert_eq!(
        Datagram::decode(&bytes).unwrap(),
        Datagram::Rel { session: 7, seq: 3, msg: Msg::Unknown { kind: 99 } }
    );
    // encode of Unknown is `[kind]` and decodes back
    let d = Datagram::Rel { session: 7, seq: 3, msg: Msg::Unknown { kind: 99 } };
    assert_eq!(Datagram::decode(&d.encode()).unwrap(), d);
}
