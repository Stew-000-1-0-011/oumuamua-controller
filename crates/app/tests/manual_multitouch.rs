//! Two fingers on the real ManualPanel: the stick and the rotation slider are separate
//! TouchAreas and drive the Manual state at the same time.

use std::cell::RefCell;
use std::rc::Rc;

use app::manual::{DEFAULT_LEVELS, Manual};
use i_slint_core::input::TouchPhase::{self, Cancelled, Ended, Moved, Started};
use i_slint_core::lengths::LogicalPoint;
use i_slint_core::platform::{InternalEvent, WindowEvent};
use link::ControlInput;
use slint::ComponentHandle;

// The measured coordinates below depend on ManualPanel's layout in a 1000x500 host (stick
// bottom left, rotation slider bottom right, lift groups top centre, starting below the 32px
// status band, so the jog buttons sit at y=151); re-measure after
// changing manual.slint.
slint::slint! {
    import { ManualPanel } from "../ui/manual.slint";
    export component Host inherits Window {
        width: 1000px;
        height: 500px;
        callback stick(float, float);
        callback slider(float);
        callback jog-pressed(int, int);
        callback jog-released(int);
        in-out property <bool> both <=> page.both;
        callback both-changed();
        page := ManualPanel {
            changed both => { root.both-changed(); }
            stick-moved(x, y) => { root.stick(x, y); }
            slider-moved(s) => { root.slider(s); }
            jog-pressed(side, dir) => { root.jog-pressed(side, dir); }
            jog-released(side) => { root.jog-released(side); }
        }
    }
    export component HariteHost inherits Window {
        width: 1000px;
        height: 500px;
        in-out property <bool> panel-visible: true;
        in-out property <bool> harite-visible: true;
        in-out property <bool> stopped: false;
        in-out property <bool> auto-active: false;
        callback jog-pressed(int, int);
        callback jog-released(int);
        callback harite-pressed(int);
        callback harite-released();
        ManualPanel {
            visible: root.panel-visible;
            harite-visible: root.harite-visible;
            stopped: root.stopped;
            auto-active: root.auto-active;
            jog-pressed(side, dir) => { root.jog-pressed(side, dir); }
            jog-released(side) => { root.jog-released(side); }
            harite-pressed(d) => { root.harite-pressed(d); }
            harite-released => { root.harite-released(); }
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
    touch(w, 0, 113., 330., Started);
    touch(w, 1, 900., 458., Started);
    let c = read();
    assert!(c.vx > 0.0, "stick drives vx: {c:?}");
    assert!(c.wz < 0.0, "slider drives wz at the same time: {c:?}");

    // Moving one finger leaves the other control alone.
    touch(w, 1, 860., 458., Moved);
    let c = read();
    assert!(c.vx > 0.0 && c.wz < 0.0, "{c:?}");

    // Lifting finger 0 zeroes only the stick.
    touch(w, 0, 113., 330., Ended);
    let c = read();
    assert_eq!((c.vx, c.vy), (0.0, 0.0));
    assert!(c.wz < 0.0, "slider still held by finger 1: {c:?}");

    touch(w, 1, 860., 458., Ended);
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
    touch(w, 0, 265., 151., Started);
    touch(w, 1, 505., 151., Started);
    assert_eq!(read(), (1, -1));
    touch(w, 0, 265., 151., Ended);
    assert_eq!(read(), (0, -1), "right keeps jogging");
    touch(w, 0, 265., 151., Started);
    assert_eq!(read(), (1, -1));
    touch(w, 1, 505., 151., Ended);
    assert_eq!(read(), (1, 0), "left keeps jogging");
    touch(w, 0, 265., 151., Ended);
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
    touch(w, 0, 265., 151., Started);
    assert_eq!(read(), (1, 1));
    // 両方 turned off while the finger is down (set like the 両方 button does).
    host.set_both(false);
    settle();
    assert_eq!(read(), (0, 0), "toggling 両方 drops held jogs");
    touch(w, 0, 265., 151., Ended);
    assert_eq!(read(), (0, 0));
    // And the other way round.
    touch(w, 0, 265., 151., Started);
    assert_eq!(read(), (1, 0));
    host.set_both(true);
    settle();
    touch(w, 0, 265., 151., Ended);
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
    touch(w, 0, 265., 151., Started);
    touch(w, 1, 315., 151., Started);
    assert_eq!(read(), (-1, 0));
    touch(w, 1, 315., 151., Ended);
    assert_eq!(read(), (0, 0));
    touch(w, 0, 265., 151., Ended);
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
    touch(w, 0, 113., 330., Started);
    touch(w, 1, 900., 458., Started);
    touch(w, 2, 265., 151., Started);
    let c = read_all(&manual);
    assert!(c.vx > 0.0 && c.wz < 0.0 && c.jog_left == 1, "{c:?}");

    touch(w, 0, 113., 330., Cancelled);
    touch(w, 1, 900., 458., Cancelled);
    touch(w, 2, 265., 151., Cancelled);
    let c = read_all(&manual);
    assert_eq!((c.vx, c.vy, c.wz, c.jog_left, c.jog_right), (0.0, 0.0, 0.0, 0, 0), "{c:?}");

    // id 0 is reused for a fresh press on the stick.
    touch(w, 0, 113., 330., Started);
    assert!(read_all(&manual).vx > 0.0);
    touch(w, 0, 113., 330., Ended);
    assert_eq!(read_all(&manual).vx, 0.0);
}

#[test]
fn first_finger_up_before_second_finger_up() {
    let (host, manual) = full_host();
    let w = host.window();
    touch(w, 0, 113., 330., Started);
    touch(w, 1, 900., 458., Started);
    touch(w, 0, 113., 330., Ended);
    let c = read_all(&manual);
    assert_eq!((c.vx, c.vy), (0.0, 0.0));
    assert!(c.wz < 0.0, "slider still held: {c:?}");
    touch(w, 1, 900., 458., Ended);
    assert_eq!(read_all(&manual).wz, 0.0);
}

// ---- ハリテ ---------------------------------------------------------------------------------------
// The same panel with the ハリテ column shown.
// Measured in the 1000x500 host with the ハリテ column shown (jog row at y=151); re-measure after changing
// manual.slint.
const LIFT_UP: (f32, f32) = (269., 151.);
const HARITE_UP: (f32, f32) = (645., 151.);
const HARITE_DOWN: (f32, f32) = (693., 151.);

/// The host with Manual wired like glue.rs does, for harite and the lift jogs.
fn harite_host() -> (HariteHost, Rc<RefCell<Manual>>, Rc<RefCell<u32>>) {
    thread_local!(static INIT: std::cell::Cell<bool> = const { std::cell::Cell::new(false) });
    if !INIT.replace(true) {
        i_slint_backend_testing::init_no_event_loop();
    }
    let host = HariteHost::new().unwrap();
    let manual = Rc::new(RefCell::new(Manual::default()));
    let releases = Rc::new(RefCell::new(0u32));
    let m = manual.clone();
    host.on_jog_pressed(move |side, dir| m.borrow_mut().jog_press(side as u8, dir as i8));
    let m = manual.clone();
    host.on_jog_released(move |side| m.borrow_mut().jog_release(side as u8));
    let m = manual.clone();
    host.on_harite_pressed(move |dir| m.borrow_mut().harite_press(dir as i8));
    let (m, r) = (manual.clone(), releases.clone());
    host.on_harite_released(move || {
        *r.borrow_mut() += 1;
        m.borrow_mut().harite_release();
    });
    host.show().unwrap();
    settle();
    (host, manual, releases)
}

fn read_harite(manual: &Rc<RefCell<Manual>>) -> (i8, i8) {
    let c = read_all(manual);
    (c.jog_harite, c.jog_left)
}

#[test]
fn harite_press_and_release_are_per_finger_and_independent_of_the_lift_jogs() {
    let (host, manual, _) = harite_host();
    let w = host.window();
    let (up, down, lift_up) = (HARITE_UP, HARITE_DOWN, LIFT_UP);
    // Finger 0 on ハリテ ▲, finger 1 on the left lift 上: both are held.
    touch(w, 0, up.0, up.1, Started);
    touch(w, 1, lift_up.0, lift_up.1, Started);
    assert_eq!(read_harite(&manual), (1, 1));
    // Lifting the lift finger leaves harite held, and the other way round.
    touch(w, 1, lift_up.0, lift_up.1, Ended);
    assert_eq!(read_harite(&manual), (1, 0));
    touch(w, 1, lift_up.0, lift_up.1, Started);
    touch(w, 0, up.0, up.1, Ended);
    assert_eq!(read_harite(&manual), (0, 1), "ハリテ released, the lift keeps jogging");
    touch(w, 1, lift_up.0, lift_up.1, Ended);
    // ▲ then ▼ (two fingers on the one mechanism): the later press wins, and lifting either stops it.
    touch(w, 0, up.0, up.1, Started);
    touch(w, 2, down.0, down.1, Started);
    assert_eq!(read_harite(&manual).0, -1);
    touch(w, 2, down.0, down.1, Ended);
    assert_eq!(read_harite(&manual).0, 0);
    touch(w, 0, up.0, up.1, Ended);
    assert_eq!(read_harite(&manual).0, 0);
}

#[test]
fn a_cancelled_touch_releases_harite() {
    let (host, manual, _) = harite_host();
    let w = host.window();
    let up = HARITE_UP;
    touch(w, 0, up.0, up.1, Started);
    assert_eq!(read_harite(&manual).0, 1);
    touch(w, 0, up.0, up.1, Cancelled);
    assert_eq!(read_harite(&manual).0, 0);
}

#[test]
fn hiding_stopping_or_removing_the_column_releases_a_held_harite_button() {
    for what in ["panel", "stopped", "auto", "column"] {
        let (host, manual, releases) = harite_host();
        let w = host.window();
        let up = HARITE_UP;
        touch(w, 0, up.0, up.1, Started);
        assert_eq!(read_harite(&manual).0, 1, "{what}");
        match what {
            "panel" => host.set_panel_visible(false),
            "stopped" => host.set_stopped(true),
            "auto" => host.set_auto_active(true),
            _ => host.set_harite_visible(false),
        }
        settle();
        assert_eq!(read_harite(&manual).0, 0, "{what}: the held ▲ was released without a finger-up");
        assert!(*releases.borrow() >= 1, "{what}");
        // The finger-up arrives later (or never): nothing changes, and showing it again does not resume.
        touch(w, 0, up.0, up.1, Ended);
        match what {
            "panel" => host.set_panel_visible(true),
            "stopped" => host.set_stopped(false),
            "auto" => host.set_auto_active(false),
            _ => host.set_harite_visible(true),
        }
        settle();
        assert_eq!(read_harite(&manual).0, 0, "{what}: does not resume");
    }
}
