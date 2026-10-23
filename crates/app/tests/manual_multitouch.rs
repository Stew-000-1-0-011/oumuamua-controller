//! Two fingers on the real ManualPage: the stick and the rotation slider are separate
//! TouchAreas and drive the Manual state at the same time.

use std::cell::RefCell;
use std::rc::Rc;

use app::manual::{DEFAULT_LEVELS, Manual};
use i_slint_core::input::TouchPhase::{self, Cancelled, Ended, Moved, Started};
use i_slint_core::lengths::LogicalPoint;
use i_slint_core::platform::{InternalEvent, WindowEvent};
use link::ControlInput;
use slint::ComponentHandle;

slint::slint! {
    import { ManualPage } from "../ui/manual.slint";
    export component Host inherits Window {
        width: 1000px;
        height: 500px;
        callback stick(float, float);
        callback slider(float);
        callback jog-pressed(int, int);
        callback jog-released(int);
        in-out property <bool> both <=> page.both;
        callback both-changed();
        page := ManualPage {
            changed both => { root.both-changed(); }
            stick-moved(x, y) => { root.stick(x, y); }
            slider-moved(s) => { root.slider(s); }
            jog-pressed(side, dir) => { root.jog-pressed(side, dir); }
            jog-released(side) => { root.jog-released(side); }
        }
    }
}

fn touch(w: &slint::Window, id: i32, x: f32, y: f32, phase: TouchPhase) {
    w.dispatch_event(WindowEvent::internal(InternalEvent::Touch { id, position: LogicalPoint::new(x, y), phase }));
}

#[test]
fn stick_and_slider_drive_manual_at_once() {
    i_slint_backend_testing::init_no_event_loop();
    let host = Host::new().unwrap();
    let manual = Rc::new(RefCell::new(Manual::default()));
    let m = manual.clone();
    host.on_stick(move |x, y| m.borrow_mut().set_stick(x, y));
    let m = manual.clone();
    host.on_slider(move |s| m.borrow_mut().set_slider(s));
    host.show().unwrap();
    let w = host.window();
    let read = || {
        let mut c = ControlInput::default();
        manual.borrow().apply(&mut c, &DEFAULT_LEVELS, true);
        c
    };

    // Finger 0 on the stick (well above its centre), finger 1 on the slider (right of its centre).
    touch(w, 0, 130., 60., Started);
    touch(w, 1, 800., 250., Started);
    let c = read();
    assert!(c.vx > 0.0, "stick drives vx: {c:?}");
    assert!(c.wz < 0.0, "slider drives wz at the same time: {c:?}");

    // Moving one finger leaves the other control alone.
    touch(w, 1, 760., 250., Moved);
    let c = read();
    assert!(c.vx > 0.0 && c.wz < 0.0, "{c:?}");

    // Lifting finger 0 zeroes only the stick.
    touch(w, 0, 130., 60., Ended);
    let c = read();
    assert_eq!((c.vx, c.vy), (0.0, 0.0));
    assert!(c.wz < 0.0, "slider still held by finger 1: {c:?}");

    touch(w, 1, 760., 250., Ended);
    let c = read();
    assert_eq!((c.vx, c.vy, c.wz), (0.0, 0.0, 0.0));
}

#[test]
fn jog_release_is_per_side() {
    i_slint_backend_testing::init_no_event_loop();
    let host = Host::new().unwrap();
    let manual = Rc::new(RefCell::new(Manual::default()));
    let m = manual.clone();
    host.on_jog_pressed(move |side, dir| m.borrow_mut().jog_press(side as u8, dir as i8));
    let m = manual.clone();
    host.on_jog_released(move |side| m.borrow_mut().jog_release(side as u8));
    host.show().unwrap();
    let w = host.window();
    let read = || {
        let mut c = ControlInput::default();
        manual.borrow().apply(&mut c, &DEFAULT_LEVELS, true);
        (c.jog_left, c.jog_right)
    };

    // Finger 0 on left 上, finger 1 on right 下.
    touch(w, 0, 310., 140., Started);
    touch(w, 1, 480., 140., Started);
    assert_eq!(read(), (1, -1));
    touch(w, 0, 310., 140., Ended);
    assert_eq!(read(), (0, -1), "right keeps jogging");
    touch(w, 0, 310., 140., Started);
    assert_eq!(read(), (1, -1));
    touch(w, 1, 480., 140., Ended);
    assert_eq!(read(), (1, 0), "left keeps jogging");
    touch(w, 0, 310., 140., Ended);
    assert_eq!(read(), (0, 0));
}

/// Runs the pending `changed` handlers, as the event loop does between events.
fn settle() {
    slint::platform::update_timers_and_animations();
}

#[test]
fn jog_survives_toggling_both_mid_hold() {
    i_slint_backend_testing::init_no_event_loop();
    let host = Host::new().unwrap();
    let manual = Rc::new(RefCell::new(Manual::default()));
    let m = manual.clone();
    host.on_jog_pressed(move |side, dir| m.borrow_mut().jog_press(side as u8, dir as i8));
    let m = manual.clone();
    host.on_jog_released(move |side| m.borrow_mut().jog_release(side as u8));
    let m = manual.clone();
    let h = host.as_weak();
    host.on_both_changed(move || m.borrow_mut().set_both(h.unwrap().get_both()));
    host.show().unwrap();
    let w = host.window();
    let read = || {
        let mut c = ControlInput::default();
        manual.borrow().apply(&mut c, &DEFAULT_LEVELS, true);
        (c.jog_left, c.jog_right)
    };

    host.set_both(true);
    settle();
    touch(w, 0, 310., 140., Started);
    assert_eq!(read(), (1, 1));
    // 両方 turned off while the finger is down (set like the 両方 button does).
    host.set_both(false);
    settle();
    assert_eq!(read(), (0, 0), "toggling 両方 drops held jogs");
    touch(w, 0, 310., 140., Ended);
    assert_eq!(read(), (0, 0));
    // And the other way round.
    touch(w, 0, 310., 140., Started);
    assert_eq!(read(), (1, 0));
    host.set_both(true);
    settle();
    touch(w, 0, 310., 140., Ended);
    assert_eq!(read(), (0, 0), "no side is left jogging");
}

#[test]
fn two_fingers_on_one_side_share_it() {
    i_slint_backend_testing::init_no_event_loop();
    let host = Host::new().unwrap();
    let manual = Rc::new(RefCell::new(Manual::default()));
    let m = manual.clone();
    host.on_jog_pressed(move |side, dir| m.borrow_mut().jog_press(side as u8, dir as i8));
    let m = manual.clone();
    host.on_jog_released(move |side| m.borrow_mut().jog_release(side as u8));
    host.show().unwrap();
    let w = host.window();
    let read = || {
        let mut c = ControlInput::default();
        manual.borrow().apply(&mut c, &DEFAULT_LEVELS, true);
        (c.jog_left, c.jog_right)
    };
    // Left 上 and left 下: the later press wins, and lifting either stops that side (intended).
    touch(w, 0, 310., 140., Started);
    touch(w, 1, 350., 140., Started);
    assert_eq!(read(), (-1, 0));
    touch(w, 1, 350., 140., Ended);
    assert_eq!(read(), (0, 0));
    touch(w, 0, 310., 140., Ended);
    assert_eq!(read(), (0, 0));
}

fn full_host() -> (Host, Rc<RefCell<Manual>>) {
    i_slint_backend_testing::init_no_event_loop();
    let host = Host::new().unwrap();
    let manual = Rc::new(RefCell::new(Manual::default()));
    let m = manual.clone();
    host.on_stick(move |x, y| m.borrow_mut().set_stick(x, y));
    let m = manual.clone();
    host.on_slider(move |s| m.borrow_mut().set_slider(s));
    let m = manual.clone();
    host.on_jog_pressed(move |side, dir| m.borrow_mut().jog_press(side as u8, dir as i8));
    let m = manual.clone();
    host.on_jog_released(move |side| m.borrow_mut().jog_release(side as u8));
    host.show().unwrap();
    (host, manual)
}

fn read_all(manual: &Rc<RefCell<Manual>>) -> ControlInput {
    let mut c = ControlInput::default();
    manual.borrow().apply(&mut c, &DEFAULT_LEVELS, true);
    c
}

#[test]
fn cancel_of_all_fingers_zeroes_everything_and_ids_are_reusable() {
    let (host, manual) = full_host();
    let w = host.window();
    touch(w, 0, 130., 60., Started);
    touch(w, 1, 800., 250., Started);
    touch(w, 2, 310., 140., Started);
    let c = read_all(&manual);
    assert!(c.vx > 0.0 && c.wz < 0.0 && c.jog_left == 1, "{c:?}");

    touch(w, 0, 130., 60., Cancelled);
    touch(w, 1, 800., 250., Cancelled);
    touch(w, 2, 310., 140., Cancelled);
    let c = read_all(&manual);
    assert_eq!((c.vx, c.vy, c.wz, c.jog_left, c.jog_right), (0.0, 0.0, 0.0, 0, 0), "{c:?}");

    // id 0 is reused for a fresh press on the stick.
    touch(w, 0, 130., 60., Started);
    assert!(read_all(&manual).vx > 0.0);
    touch(w, 0, 130., 60., Ended);
    assert_eq!(read_all(&manual).vx, 0.0);
}

#[test]
fn first_finger_up_before_second_finger_up() {
    let (host, manual) = full_host();
    let w = host.window();
    touch(w, 0, 130., 60., Started);
    touch(w, 1, 800., 250., Started);
    touch(w, 0, 130., 60., Ended);
    let c = read_all(&manual);
    assert_eq!((c.vx, c.vy), (0.0, 0.0));
    assert!(c.wz < 0.0, "slider still held: {c:?}");
    touch(w, 1, 800., 250., Ended);
    assert_eq!(read_all(&manual).wz, 0.0);
}
