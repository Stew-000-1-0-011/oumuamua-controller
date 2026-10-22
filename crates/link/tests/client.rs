use link::{Client, ControlInput, LinkEvent};
use proto::{Datagram, Mode, Msg, State};
use std::net::{SocketAddr, UdpSocket};
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

struct Bridge {
    sock: UdpSocket,
    phone: Option<SocketAddr>,
    session: u32,
}

fn state_body() -> State {
    State {
        mode: Mode::Manual,
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

impl Bridge {
    fn new() -> (Bridge, SocketAddr) {
        let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        sock.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
        let addr = sock.local_addr().unwrap();
        (Bridge { sock, phone: None, session: 0 }, addr)
    }

    fn recv(&mut self, within: Duration) -> Option<Datagram> {
        let end = Instant::now() + within;
        let mut buf = [0u8; 2048];
        while Instant::now() < end {
            if let Ok((n, src)) = self.sock.recv_from(&mut buf) {
                self.phone = Some(src);
                return Some(Datagram::decode(&buf[..n]).unwrap());
            }
        }
        None
    }

    /// Receive until `pick` returns Some, ignoring other datagrams.
    fn wait<T>(&mut self, within: Duration, mut pick: impl FnMut(Datagram) -> Option<T>) -> T {
        let end = Instant::now() + within;
        while let Some(left) = end.checked_duration_since(Instant::now()) {
            if let Some(d) = self.recv(left)
                && let Some(x) = pick(d)
            {
                return x;
            }
        }
        panic!("timed out waiting for datagram");
    }

    fn send(&self, d: &Datagram) {
        self.sock.send_to(&d.encode(), self.phone.unwrap()).unwrap();
    }

    /// Wait for HELLO and answer with an accepting WELCOME.
    fn accept(&mut self) {
        let (session, name, takeover) = self.wait(Duration::from_secs(2), |d| match d {
            Datagram::Hello { session, client_name, takeover } => Some((session, client_name, takeover)),
            _ => None,
        });
        assert_eq!(name, "test-phone");
        assert!(!takeover);
        assert_ne!(session, 0);
        self.session = session;
        self.send(&Datagram::Welcome {
            session,
            robot_name: "oumuamua".into(),
            accepted: true,
            reason: String::new(),
        });
    }

    fn send_state(&self, ack_seq: u32, ack_t_ms: u32) {
        self.send(&Datagram::State {
            session: self.session,
            seq: 1,
            ack_seq,
            ack_t_ms,
            body: state_body(),
        });
    }
}

fn next_event(rx: &Receiver<LinkEvent>, within: Duration, mut pick: impl FnMut(&LinkEvent) -> bool) -> LinkEvent {
    let end = Instant::now() + within;
    while let Some(left) = end.checked_duration_since(Instant::now()) {
        if let Ok(ev) = rx.recv_timeout(left)
            && pick(&ev)
        {
            return ev;
        }
    }
    panic!("timed out waiting for event");
}

fn start() -> (Bridge, Client, Receiver<LinkEvent>) {
    let (bridge, addr) = Bridge::new();
    let (client, rx) = Client::start(addr, "test-phone".into(), false);
    (bridge, client, rx)
}

const T: Duration = Duration::from_secs(2);

#[test]
fn handshake_and_control_stream() {
    let (mut bridge, client, rx) = start();
    client.set_control(ControlInput { vx: 0.5, wz: -1.0, jog_left: 1, ..Default::default() });
    bridge.accept();
    match next_event(&rx, T, |e| matches!(e, LinkEvent::Connected { .. })) {
        LinkEvent::Connected { robot_name } => assert_eq!(robot_name, "oumuamua"),
        _ => unreachable!(),
    }
    let mut seqs = Vec::new();
    while seqs.len() < 4 {
        let c = bridge.wait(T, |d| if let Datagram::Control(c) = d { Some(c) } else { None });
        assert_eq!(c.session, bridge.session);
        assert_eq!((c.vx, c.wz, c.jog_left), (0.5, -1.0, 1));
        seqs.push(c.seq);
    }
    assert_eq!(seqs[0], 1);
    assert!(seqs.windows(2).all(|w| w[1] == w[0] + 1), "{seqs:?}");
    client.stop();
}

#[test]
fn hello_repeats_until_welcome() {
    let (mut bridge, _client, _rx) = start();
    let t0 = Instant::now();
    let mut hellos = 0;
    bridge.wait(Duration::from_secs(2), |d| {
        if matches!(d, Datagram::Hello { .. }) {
            hellos += 1;
        }
        (hellos == 2).then_some(())
    });
    assert!(t0.elapsed() >= Duration::from_millis(400));
}

#[test]
fn rejected_is_reported() {
    let (mut bridge, _client, rx) = start();
    let session = bridge.wait(T, |d| if let Datagram::Hello { session, .. } = d { Some(session) } else { None });
    bridge.send(&Datagram::Welcome { session, robot_name: "r".into(), accepted: false, reason: "busy".into() });
    match next_event(&rx, T, |e| matches!(e, LinkEvent::Rejected { .. })) {
        LinkEvent::Rejected { reason } => assert_eq!(reason, "busy"),
        _ => unreachable!(),
    }
}

#[test]
fn reliable_msgs_are_acked_and_delivered() {
    let (mut bridge, client, rx) = start();
    bridge.accept();

    // Phone -> bridge: retransmitted until acked.
    let req = client.next_req();
    assert_eq!(req, 1);
    assert_eq!(client.next_req(), 2);
    client.send(Msg::Cancel { req });
    let mut copies = 0;
    let expect_session = bridge.session;
    bridge.wait(T, |d| match d {
        Datagram::Rel { session, seq: 1, msg: Msg::Cancel { req: 1 } } => {
            assert_eq!(session, expect_session);
            copies += 1;
            (copies == 2).then_some(())
        }
        _ => None,
    });
    bridge.send(&Datagram::Ack { session: bridge.session, cum: 1 });
    // Drain, then make sure no more copies show up.
    bridge.recv(Duration::from_millis(60));
    let late = bridge.recv(Duration::from_millis(50));
    let end = Instant::now() + Duration::from_millis(300);
    let mut stray = matches!(late, Some(Datagram::Rel { .. }));
    while Instant::now() < end {
        stray |= matches!(bridge.recv(Duration::from_millis(50)), Some(Datagram::Rel { .. }));
    }
    assert!(!stray, "acked REL was retransmitted");

    // Bridge -> phone: out of order, a duplicate, and an unknown kind.
    let session = bridge.session;
    let rel = |seq, msg| Datagram::Rel { session, seq, msg };
    let res = |req| Msg::Result { req, ok: true, message: format!("r{req}") };
    bridge.send(&rel(2, res(2)));
    let cum = bridge.wait(T, |d| if let Datagram::Ack { cum, .. } = d { Some(cum) } else { None });
    assert_eq!(cum, 0);
    bridge.send(&rel(1, res(1)));
    bridge.send(&rel(3, Msg::Unknown { kind: 99 }));
    bridge.send(&rel(3, Msg::Unknown { kind: 99 }));
    bridge.send(&rel(4, res(4)));
    let mut got = Vec::new();
    while got.len() < 3 {
        if let LinkEvent::Msg(Msg::Result { req, .. }) =
            next_event(&rx, T, |e| matches!(e, LinkEvent::Msg(_)))
        {
            got.push(req);
        }
    }
    assert_eq!(got, vec![1, 2, 4]);
    // The unknown one was ACKed (cum reaches 4) and never emitted.
    bridge.wait(T, |d| matches!(d, Datagram::Ack { cum: 4, .. }).then_some(()));
    assert!(rx.recv_timeout(Duration::from_millis(100)).map_or(true, |e| !matches!(e, LinkEvent::Msg(_))));
    client.stop();
}

#[test]
fn state_rtt_is_reported() {
    let (mut bridge, client, rx) = start();
    bridge.accept();
    let c = bridge.wait(T, |d| if let Datagram::Control(c) = d { Some(c) } else { None });
    std::thread::sleep(Duration::from_millis(40));
    bridge.send_state(c.seq, c.t_ms);
    match next_event(&rx, T, |e| matches!(e, LinkEvent::State { .. })) {
        LinkEvent::State { body, rtt_ms } => {
            assert_eq!(body.mode, Mode::Manual);
            let rtt = rtt_ms.expect("rtt");
            assert!((40..400).contains(&rtt), "rtt {rtt}");
        }
        _ => unreachable!(),
    }
    // No CONTROL acked yet -> no rtt.
    bridge.send_state(0, 0);
    match next_event(&rx, T, |e| matches!(e, LinkEvent::State { .. })) {
        LinkEvent::State { rtt_ms, .. } => assert_eq!(rtt_ms, None),
        _ => unreachable!(),
    }
    client.stop();
}

#[test]
fn rehello_after_two_seconds_silence() {
    let (mut bridge, client, rx) = start();
    bridge.accept();
    next_event(&rx, T, |e| matches!(e, LinkEvent::Connected { .. }));
    bridge.send_state(0, 0);
    next_event(&rx, T, |e| matches!(e, LinkEvent::State { .. }));
    let silence = Instant::now();
    client.send(Msg::Cancel { req: 1 }); // pending REL must be dropped on re-HELLO
    next_event(&rx, Duration::from_millis(1800), |e| matches!(e, LinkEvent::Lost));
    assert!(silence.elapsed() >= Duration::from_millis(900));
    let old = bridge.session;
    let new = bridge.wait(Duration::from_millis(2500), |d| match d {
        Datagram::Hello { session, .. } => Some(session),
        _ => None,
    });
    assert_ne!(new, old);
    assert_ne!(new, 0);
    assert!(silence.elapsed() >= Duration::from_millis(1900));
    // Lost is emitted once only.
    assert!(!matches!(rx.try_recv(), Ok(LinkEvent::Lost)));
    // Fresh session restarts numbering; the old REL is gone.
    bridge.session = new;
    bridge.send(&Datagram::Welcome { session: new, robot_name: "oumuamua".into(), accepted: true, reason: String::new() });
    next_event(&rx, T, |e| matches!(e, LinkEvent::Connected { .. }));
    let c = bridge.wait(T, |d| if let Datagram::Control(c) = d { Some(c) } else { None });
    assert_eq!((c.session, c.seq), (new, 1));
    let mut rels = 0;
    let end = Instant::now() + Duration::from_millis(300);
    while Instant::now() < end {
        rels += usize::from(matches!(bridge.recv(Duration::from_millis(30)), Some(Datagram::Rel { .. })));
    }
    assert_eq!(rels, 0);
    client.stop();
}

#[test]
fn scan_fragments_are_joined() {
    let (mut bridge, client, rx) = start();
    bridge.accept();
    let session = bridge.session;
    let scan = |scan_id, frag, frag_count, points: Vec<(i16, i16)>| Datagram::Scan {
        session,
        scan_id,
        frag,
        frag_count,
        points,
    };
    // scan 5: fragments arrive out of order.
    bridge.send(&scan(5, 1, 2, vec![(3000, 4000)]));
    bridge.send(&scan(5, 0, 2, vec![(1000, -2000), (500, 0)]));
    // scan 6 is abandoned when 7 arrives.
    bridge.send(&scan(6, 0, 2, vec![(1, 1)]));
    bridge.send(&scan(7, 0, 1, vec![(-1500, 250)]));
    let mut scans = Vec::new();
    while scans.len() < 2 {
        if let LinkEvent::Scan(p) = next_event(&rx, T, |e| matches!(e, LinkEvent::Scan(_))) {
            scans.push(p);
        }
    }
    assert_eq!(scans[0], vec![(1.0, -2.0), (0.5, 0.0), (3.0, 4.0)]);
    assert_eq!(scans[1], vec![(-1.5, 0.25)]);
    client.stop();
}

#[test]
fn bye_is_reported_and_stop_sends_bye() {
    let (mut bridge, client, rx) = start();
    bridge.accept();
    bridge.recv(Duration::from_millis(50));
    bridge.send(&Datagram::Bye { session: bridge.session, reason: "taken over".into() });
    match next_event(&rx, T, |e| matches!(e, LinkEvent::Bye { .. })) {
        LinkEvent::Bye { reason } => assert_eq!(reason, "taken over"),
        _ => unreachable!(),
    }
    client.stop();
    let reason = bridge.wait(T, |d| if let Datagram::Bye { reason, .. } = d { Some(reason) } else { None });
    assert!(!reason.is_empty());
}
