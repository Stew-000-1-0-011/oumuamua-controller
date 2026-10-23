//! Screens stay alive under the window and are switched with `visible`, so a finger held on the
//! manual stick when the page changes must end in zero motion: the stick lets go itself, and
//! the Rust side (`motion_allowed`) refuses manual input off page 1.

use std::cell::RefCell;
use std::rc::Rc;

use app::MainWindow;
use app::manual::{DEFAULT_LEVELS, Manual, motion_allowed};
use i_slint_core::input::TouchPhase::{self, Ended, Moved, Started};
use i_slint_core::lengths::LogicalPoint;
use i_slint_core::platform::{InternalEvent, WindowEvent};
use link::ControlInput;
use slint::ComponentHandle;

fn touch(w: &slint::Window, id: i32, x: f32, y: f32, phase: TouchPhase) {
    w.dispatch_event(WindowEvent::internal(InternalEvent::Touch { id, position: LogicalPoint::new(x, y), phase }));
}

fn settle() {
    slint::platform::update_timers_and_animations();
}

#[test]
fn holding_the_stick_while_the_page_changes_stops_motion() {
    i_slint_backend_testing::init_no_event_loop();
    let win = MainWindow::new().unwrap();
    win.window().set_size(slint::LogicalSize::new(800., 360.));
    win.set_connected(true);
    win.set_page(1);
    let manual = Rc::new(RefCell::new(Manual::default()));
    // Same gating as glue.rs: input is applied only while the manual page is showing.
    let last = Rc::new(RefCell::new((9.0f32, 9.0f32)));
    let (m, w, l) = (manual.clone(), win.as_weak(), last.clone());
    win.on_manual_stick(move |x, y| {
        *l.borrow_mut() = (x, y);
        let page = w.unwrap().get_page();
        if motion_allowed(true, true, page, Some(proto::Mode::Manual)) {
            m.borrow_mut().set_stick(x, y);
        } else {
            m.borrow_mut().release_all();
        }
    });
    win.show().unwrap();
    settle();
    let ww = win.window();
    let read = |page: i32| {
        let mut c = ControlInput::default();
        manual.borrow().apply(&mut c, &DEFAULT_LEVELS, motion_allowed(true, true, page, Some(proto::Mode::Manual)));
        c
    };

    // Stick centre is about (86, 274) at 800x360; press well above it.
    touch(ww, 0, 86., 220., Started);
    assert!(read(1).vx > 0.0, "stick drives vx on the manual page: {:?}", read(1));

    // The page changes with the finger still down.
    win.set_page(0);
    settle();
    assert_eq!(*last.borrow(), (0.0, 0.0), "the hidden panel reports the stick as released");
    let c = read(0);
    assert_eq!((c.vx, c.vy, c.wz), (0.0, 0.0, 0.0), "no motion after the page change: {c:?}");
    let c = read(1);
    assert_eq!((c.vx, c.vy), (0.0, 0.0), "and nothing is left held: {c:?}");

    // The finger keeps moving or lifts: still nothing, and coming back does not resume it.
    touch(ww, 0, 86., 210., Moved);
    assert_eq!(read(1).vx, 0.0);
    win.set_page(1);
    settle();
    assert_eq!(read(1).vx, 0.0, "a finger from before the switch does not resume");
    touch(ww, 0, 86., 210., Ended);
    assert_eq!(read(1).vx, 0.0);
}
