use demo::DemoRobot;
use link::{Client, ControlInput, LinkEvent};
use proto::{Datagram, Mode, Msg, State};
use std::collections::VecDeque;
use std::net::UdpSocket;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

/// Events from the client, with the ones a wait skipped kept for the next wait.
struct Probe {
    rx: Receiver<LinkEvent>,
    buf: VecDeque<LinkEvent>,
}

impl Probe {
    fn wait<T>(&mut self, secs: f64, pick: impl Fn(&LinkEvent) -> Option<T>) -> T {
        if let Some(i) = self.buf.iter().position(|e| pick(e).is_some()) {
            return pick(&self.buf.remove(i).unwrap()).unwrap();
        }
        let end = Instant::now() + Duration::from_secs_f64(secs);
        while let Some(left) = end.checked_duration_since(Instant::now()) {
            if let Ok(e) = self.rx.recv_timeout(left) {
                match pick(&e) {
                    Some(x) => return x,
                    None => self.buf.push_back(e),
                }
            }
        }
        panic!("timed out waiting for an event");
    }

    fn msg<T>(&mut self, secs: f64, pick: impl Fn(&Msg) -> Option<T>) -> T {
        self.wait(secs, |e| match e {
            LinkEvent::Msg(m) => pick(m),
            _ => None,
        })
    }

    fn state(&mut self, secs: f64, pick: impl Fn(&State) -> bool) -> State {
        self.wait(secs, |e| match e {
            LinkEvent::State { body, .. } if pick(body) => Some(body.clone()),
            _ => None,
        })
    }

    fn result(&mut self, req: u32) -> (bool, String) {
        self.msg(3.0, |m| match m {
            Msg::Result { req: r, ok, message } if *r == req => Some((*ok, message.clone())),
            _ => None,
        })
    }
}

fn connect() -> (DemoRobot, Client, Probe) {
    let robot = DemoRobot::start().unwrap();
    let (client, rx) = Client::start(robot.addr(), "test-phone".into(), false);
    let mut p = Probe { rx, buf: VecDeque::new() };
    p.wait(2.0, |e| matches!(e, LinkEvent::Connected { .. }).then_some(()));
    (robot, client, p)
}

fn call(c: &Client, p: &mut Probe, build: impl FnOnce(u32) -> Msg) -> (bool, String) {
    let req = c.next_req();
    c.send(build(req));
    p.result(req)
}

fn release(c: &Client, p: &mut Probe) {
    assert_eq!(call(c, p, |req| Msg::ReleaseStop { req }), (true, String::new()));
    p.state(2.0, |s| s.mode == Mode::Manual);
}

#[test]
fn connect_delivers_config_field_and_starts_stopped() {
    let (_r, _c, mut p) = connect();
    let paths = p.msg(2.0, |m| match m {
        Msg::Config { paths, lift_max, .. } => {
            assert!(*lift_max > 0.0);
            Some(paths.clone())
        }
        _ => None,
    });
    assert!(!paths.is_empty());
    p.msg(2.0, |m| matches!(m, Msg::Field { .. }).then_some(()));
    p.msg(2.0, |m| matches!(m, Msg::PathShape { points, .. } if points.is_empty()).then_some(()));
    p.msg(2.0, |m| matches!(m, Msg::TaskMemory { .. }).then_some(()));
    let s = p.state(2.0, |_| true);
    assert_eq!((s.mode, s.stop_reason.as_str()), (Mode::Stopped, "link lost"));
    // SCAN at 10 Hz: 360 rays arrive reassembled.
    let n = p.wait(2.0, |e| match e {
        LinkEvent::Scan(pts) => Some(pts.len()),
        _ => None,
    });
    assert!(n >= 300, "scan has {n} points");
}

#[test]
fn release_then_drive_forward_moves_the_robot() {
    let (_r, c, mut p) = connect();
    release(&c, &mut p);
    let x0 = p.state(1.0, |_| true).pose.unwrap()[0];
    let end = Instant::now() + Duration::from_millis(600);
    while Instant::now() < end {
        c.set_control(ControlInput { vx: 1.0, ..Default::default() });
        std::thread::sleep(Duration::from_millis(30));
    }
    c.set_control(ControlInput::default());
    // The robot starts facing -x (its front, the gripper side, points to the start zone), so forward is -x.
    let x1 = p.state(1.0, |s| s.pose.is_some_and(|q| q[0] < x0 - 0.2)).pose.unwrap()[0];
    assert!(x1 < x0 - 0.2);
}

#[test]
fn silent_operator_is_stopped_within_a_moment() {
    // link::Client never stops sending CONTROL, so speak the protocol by hand.
    let robot = DemoRobot::start().unwrap();
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock.set_read_timeout(Some(Duration::from_millis(10))).unwrap();
    let send = |d: Datagram| sock.send_to(&d.encode(), robot.addr()).unwrap();
    let session = 77;
    send(Datagram::Hello { session, client_name: "raw".into(), takeover: false });
    let mut seq = 0;
    let control = |seq: &mut u32| {
        *seq += 1;
        send(Datagram::Control(proto::Control {
            session, seq: *seq, t_ms: 0, vx: 0.0, vy: 0.0, wz: 0.0,
            field_frame: false, jog_left: 0, jog_right: 0, estop: false,
        }));
    };
    let mut buf = [0u8; 2048];
    let mut released = false;
    let mut last_control = Instant::now();
    let mut stopped_at = None;
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(3) && stopped_at.is_none() {
        let talking = start.elapsed() < Duration::from_millis(600);
        if talking && last_control.elapsed() >= Duration::from_millis(20) {
            control(&mut seq);
            last_control = Instant::now();
            if seq == 5 {
                send(Datagram::Rel { session, seq: 1, msg: Msg::ReleaseStop { req: 1 } });
            }
        }
        let Ok((n, _)) = sock.recv_from(&mut buf) else { continue };
        if let Ok(Datagram::State { body, .. }) = Datagram::decode(&buf[..n]) {
            if body.mode == Mode::Manual {
                released = true;
            } else if released && body.mode == Mode::Stopped {
                assert_eq!(body.stop_reason, "link lost");
                stopped_at = Some(start.elapsed());
            }
        }
    }
    assert!(released, "never reached manual");
    let t = stopped_at.expect("never stopped after CONTROL ceased");
    assert!(t < Duration::from_millis(1400), "stopped after {t:?}");
}

#[test]
fn lift_is_refused_while_stopped_and_works_after_release() {
    let (_r, c, mut p) = connect();
    let (ok, msg) = call(&c, &mut p, |req| Msg::LiftTo { req, gripper: 0, height: 0.02 });
    assert!(!ok && msg.starts_with("stopped:"), "{ok} {msg}");
    release(&c, &mut p);
    assert_eq!(call(&c, &mut p, |req| Msg::LiftTo { req, gripper: 2, height: 0.02 }), (true, String::new()));
    let s = p.state(1.0, |s| s.grippers[0].is_some_and(|g| g.arrived && g.height > 0.019));
    assert!(s.grippers[1].is_some());
    let (ok, _) = call(&c, &mut p, |req| Msg::LiftTo { req, gripper: 0, height: 5.0 });
    assert!(!ok);
}

#[test]
fn run_task_walks_through_steps_then_reports_done() {
    let (_r, c, mut p) = connect();
    release(&c, &mut p);
    let (ok, _) = call(&c, &mut p, |req| Msg::RunTask { req, kind: 0, gripper: 0, block: 2, column: 0, row: 0 });
    assert!(ok);
    let (busy, msg) = call(&c, &mut p, |req| Msg::RunTask { req, kind: 0, gripper: 1, block: 3, column: 0, row: 0 });
    assert!(!busy && msg == "busy");
    let first = p.state(1.0, |s| s.task.is_some() && s.mode == Mode::Auto).task.unwrap().step;
    let later = p.state(2.0, |s| s.task.as_ref().is_some_and(|t| t.step != first)).task.unwrap().step;
    assert_ne!(first, later);
    p.msg(4.0, |m| matches!(m, Msg::TaskDone { kind: 0, ok: true, .. }).then_some(()));
    let holding = p.msg(2.0, |m| match m {
        Msg::TaskMemory { holding, picked, .. } if picked.contains(&2) => Some(*holding),
        _ => None,
    });
    assert!(holding[0]);
    p.state(1.0, |s| s.mode == Mode::Manual && s.task.is_none());
}

#[test]
fn start_path_runs_about_three_seconds_then_returns_to_manual() {
    let (_r, c, mut p) = connect();
    release(&c, &mut p);
    let (ok, _) = call(&c, &mut p, |req| Msg::StartPath { req, name: "slalom_forward".into() });
    assert!(ok);
    let s = p.state(1.0, |s| s.mode == Mode::Auto);
    assert_eq!(s.path, Some(("slalom_forward".to_string(), true)));
    p.msg(1.0, |m| matches!(m, Msg::PathShape { points, .. } if !points.is_empty()).then_some(()));
    let t0 = Instant::now();
    let s = p.state(4.0, |s| s.mode == Mode::Manual);
    assert!(s.path.is_none());
    assert!(t0.elapsed() > Duration::from_secs(1), "finished too early: {:?}", t0.elapsed());
}

#[test]
fn cancel_returns_to_manual() {
    let (_r, c, mut p) = connect();
    release(&c, &mut p);
    call(&c, &mut p, |req| Msg::StartPath { req, name: "slalom_forward".into() });
    p.state(1.0, |s| s.mode == Mode::Auto);
    assert!(call(&c, &mut p, |req| Msg::Cancel { req }).0);
    p.state(1.0, |s| s.mode == Mode::Manual && s.path.is_none());
}

#[test]
fn shell_echo_completion_and_ctrl_c() {
    let (_r, c, mut p) = connect();
    assert!(call(&c, &mut p, |req| Msg::ShellOpen { req, tab: "t1".into() }).0);
    let replay = p.msg(1.0, |m| match m {
        Msg::ShellOpened { tab, replay } if tab == "t1" => Some(String::from_utf8(replay.clone()).unwrap()),
        _ => None,
    });
    assert!(replay.ends_with("demo$ "), "{replay:?}");
    assert!(!call(&c, &mut p, |req| Msg::ShellOpen { req, tab: "t1".into() }).0);

    let data = b"echo hi\n".to_vec();
    assert!(call(&c, &mut p, |req| Msg::ShellInput { req, tab: "t1".into(), data }).0);
    let mut got = String::new();
    while !got.contains("hi\r\ndemo$ ") {
        let d = p.msg(2.0, |m| match m {
            Msg::ShellOutput { tab, data } if tab == "t1" => {
                assert!(data.len() <= 900);
                Some(String::from_utf8_lossy(data).into_owned())
            }
            _ => None,
        });
        got += &d;
    }

    let data = b"nosuch\n".to_vec();
    call(&c, &mut p, |req| Msg::ShellInput { req, tab: "t1".into(), data });
    p.msg(2.0, |m| match m {
        Msg::ShellOutput { data, .. } if String::from_utf8_lossy(data).contains("demo: nosuch:") => Some(()),
        _ => None,
    });

    let req = c.next_req();
    c.send(Msg::ShellComplete { req, tab: "t1".into(), line: "ec".into(), cursor: 2 });
    let (start, cands) = p.msg(2.0, |m| match m {
        Msg::ShellCompletion { req: r, start, candidates, .. } if *r == req => Some((*start, candidates.clone())),
        _ => None,
    });
    assert_eq!((start, cands), (0, vec!["echo".to_string()]));

    let data = vec![3u8];
    call(&c, &mut p, |req| Msg::ShellInput { req, tab: "t1".into(), data });
    p.msg(2.0, |m| match m {
        Msg::ShellOutput { data, .. } if String::from_utf8_lossy(data).starts_with("^C") => Some(()),
        _ => None,
    });
}

#[test]
fn stop_ends_the_thread_and_releases_the_port() {
    let robot = DemoRobot::start().unwrap();
    let addr = robot.addr();
    let t = Instant::now();
    robot.stop();
    assert!(t.elapsed() < Duration::from_secs(1));
    UdpSocket::bind(addr).expect("port is free after stop");
}

#[test]
fn estop_stops_and_turns_suction_off_and_release_is_refused_while_held() {
    let (_r, c, mut p) = connect();
    release(&c, &mut p);
    assert!(call(&c, &mut p, |req| Msg::SetSuction { req, gripper: 2, on: true }).0);
    p.state(1.0, |s| s.grippers.iter().all(|g| g.is_some_and(|g| g.suction)));
    c.set_control(ControlInput { estop: true, ..Default::default() });
    let s = p.state(1.0, |s| s.mode == Mode::Stopped && s.stop_reason == "estop");
    assert!(s.grippers.iter().all(|g| g.is_some_and(|g| !g.suction)));
    assert_eq!(call(&c, &mut p, |req| Msg::ReleaseStop { req }), (false, "estop is still pressed".into()));
    let (ok, msg) = call(&c, &mut p, |req| Msg::SetSuction { req, gripper: 0, on: true });
    assert!(!ok && msg == "stopped: estop", "{msg}");
    assert!(call(&c, &mut p, |req| Msg::SetSuction { req, gripper: 0, on: false }).0);
    c.set_control(ControlInput::default());
    release(&c, &mut p);
}

#[test]
fn release_without_a_control_is_refused() {
    let robot = DemoRobot::start().unwrap();
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock.set_read_timeout(Some(Duration::from_millis(50))).unwrap();
    let send = |d: Datagram| sock.send_to(&d.encode(), robot.addr()).unwrap();
    send(Datagram::Hello { session: 5, client_name: "raw".into(), takeover: false });
    send(Datagram::Rel { session: 5, seq: 1, msg: Msg::ReleaseStop { req: 1 } });
    let mut buf = [0u8; 2048];
    let end = Instant::now() + Duration::from_secs(2);
    while Instant::now() < end {
        let Ok((n, _)) = sock.recv_from(&mut buf) else { continue };
        if let Ok(Datagram::Rel { msg: Msg::Result { req: 1, ok, message }, .. }) = Datagram::decode(&buf[..n]) {
            assert!(!ok && message == "no control received");
            return;
        }
    }
    panic!("no RESULT");
}

#[test]
fn set_pose_rejects_non_finite_values() {
    let (_r, c, mut p) = connect();
    assert!(!call(&c, &mut p, |req| Msg::SetPose { req, x: f64::NAN, y: 0.0, yaw: 0.0 }).0);
    assert!(call(&c, &mut p, |req| Msg::SetPose { req, x: 1.0, y: 0.5, yaw: 0.0 }).0);
    p.state(1.0, |s| s.pose == Some([1.0, 0.5, 0.0]));
}

#[test]
fn state_reports_five_motors_with_current_and_speed() {
    let (_r, c, mut p) = connect();
    release(&c, &mut p);
    let s = p.state(1.0, |_| true);
    assert_eq!(s.wheels.iter().map(|w| w.name.as_str()).collect::<Vec<_>>(), ["車輪0", "車輪1", "車輪2", "吸引 左", "吸引 右"]);
    assert!(s.wheels.iter().all(|w| w.ok && w.vdc > 20.0));
    // Driving: the wheels turn and draw current in proportion to speed.
    let end = Instant::now() + Duration::from_millis(500);
    while Instant::now() < end {
        c.set_control(ControlInput { vx: 0.8, ..Default::default() });
        std::thread::sleep(Duration::from_millis(30));
    }
    let s = p.state(2.0, |s| s.wheels.iter().take(3).any(|w| w.velocity.abs() > 5.0));
    for w in s.wheels.iter().take(3) {
        assert!((w.current - 0.06 * w.velocity).abs() < 1e-6, "{w:?}");
    }
    // Suction: the right motor draws current while on, the left stays disabled.
    c.set_control(ControlInput::default());
    assert!(call(&c, &mut p, |req| Msg::SetSuction { req, gripper: 1, on: true }).0);
    let s = p.state(2.0, |s| s.wheels[4].current > 1.0);
    assert_eq!((s.wheels[3].current, s.wheels[3].mode), (0.0, proto::MOTOR_MODE_DISABLED));
    assert_eq!(s.wheels[4].mode, proto::MOTOR_MODE_VELOCITY);
    assert!(call(&c, &mut p, |req| Msg::SetSuction { req, gripper: 2, on: false }).0);
    p.state(2.0, |s| s.wheels[4].current == 0.0);
}
