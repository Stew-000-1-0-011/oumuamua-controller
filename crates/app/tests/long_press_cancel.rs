//! A cancelled long-press (reset / forget) must not later confirm.

use std::cell::Cell;
use std::rc::Rc;
use std::time::Duration;

use i_slint_core::input::TouchPhase::{self, Cancelled, Ended, Started};
use i_slint_core::lengths::LogicalPoint;
use i_slint_core::platform::{InternalEvent, WindowEvent};
use slint::ComponentHandle;

slint::slint! {
    import { ManualPanel } from "../ui/manual.slint";
    import { AutoPanel } from "../ui/auto.slint";
    export component MHost inherits Window {
        width: 1000px;
        height: 500px;
        callback reset-pressed();
        callback reset-confirmed();
        manual := ManualPanel {
            reset-enabled: true;
            reset-pressed => { root.reset-pressed(); }
            reset-confirmed => { root.reset-confirmed(); }
        }
    }
    export component AHost inherits Window {
        width: 1000px;
        height: 500px;
        callback forget-pressed();
        callback forget-confirmed();
        in property <bool> shown: true;
        auto := AutoPanel {
            visible: root.shown;
            forget-enabled: true;
            forget-pressed => { root.forget-pressed(); }
            forget-confirmed => { root.forget-confirmed(); }
        }
    }
}

fn touch(w: &slint::Window, id: i32, x: f32, y: f32, phase: TouchPhase) {
    w.dispatch_event(WindowEvent::internal(InternalEvent::Touch {
        id,
        position: LogicalPoint::new(x, y),
        phase,
    }));
}

fn elapse(ms: u64) {
    i_slint_backend_testing::mock_elapsed_time(Duration::from_millis(ms));
    slint::platform::update_timers_and_animations();
}

/// Finds a point where a press raises `pressed`; leaves the finger lifted.
fn find_button(w: &slint::Window, pressed: &Cell<u32>) -> (f32, f32) {
    for y in (5..500).step_by(10) {
        for x in (5..1000).step_by(10) {
            let before = pressed.get();
            touch(w, 0, x as f32, y as f32, Started);
            let hit = pressed.get() > before;
            touch(w, 0, x as f32, y as f32, Cancelled);
            elapse(1500);
            if hit {
                return (x as f32, y as f32);
            }
        }
    }
    panic!("button not found");
}

#[test]
fn reset_long_press_cancelled_never_confirms() {
    i_slint_backend_testing::init_no_event_loop();
    let h = MHost::new().unwrap();
    let pressed = Rc::new(Cell::new(0));
    let confirmed = Rc::new(Cell::new(0));
    let p = pressed.clone();
    h.on_reset_pressed(move || p.set(p.get() + 1));
    let c = confirmed.clone();
    h.on_reset_confirmed(move || c.set(c.get() + 1));
    h.show().unwrap();
    let w = h.window();
    let (x, y) = find_button(w, &pressed);
    assert_eq!(confirmed.get(), 0, "probing with cancels never confirms");
    touch(w, 0, x, y, Started);
    elapse(300);
    touch(w, 0, x, y, Cancelled);
    elapse(2000);
    assert_eq!(confirmed.get(), 0);
    // control: a held press still confirms
    touch(w, 0, x, y, Started);
    elapse(1100);
    assert_eq!(confirmed.get(), 1);
    touch(w, 0, x, y, Ended);
}

#[test]
fn forget_long_press_cancelled_never_confirms() {
    i_slint_backend_testing::init_no_event_loop();
    let h = AHost::new().unwrap();
    let pressed = Rc::new(Cell::new(0));
    let confirmed = Rc::new(Cell::new(0));
    let p = pressed.clone();
    h.on_forget_pressed(move || p.set(p.get() + 1));
    let c = confirmed.clone();
    h.on_forget_confirmed(move || c.set(c.get() + 1));
    h.show().unwrap();
    let w = h.window();
    let (x, y) = find_button(w, &pressed);
    assert_eq!(confirmed.get(), 0);
    touch(w, 0, x, y, Started);
    elapse(300);
    touch(w, 0, x, y, Cancelled);
    elapse(2000);
    assert_eq!(confirmed.get(), 0);
    touch(w, 0, x, y, Started);
    elapse(1100);
    assert_eq!(confirmed.get(), 1);
    touch(w, 0, x, y, Ended);
}

#[test]
fn forget_long_press_does_not_confirm_while_hidden() {
    i_slint_backend_testing::init_no_event_loop();
    let h = AHost::new().unwrap();
    let pressed = Rc::new(Cell::new(0));
    let confirmed = Rc::new(Cell::new(0));
    let p = pressed.clone();
    h.on_forget_pressed(move || p.set(p.get() + 1));
    let c = confirmed.clone();
    h.on_forget_confirmed(move || c.set(c.get() + 1));
    h.show().unwrap();
    let w = h.window();
    let (x, y) = find_button(w, &pressed);
    touch(w, 0, x, y, Started);
    elapse(300);
    // The page changes with the finger still down.
    h.set_shown(false);
    elapse(50);
    elapse(2000);
    assert_eq!(confirmed.get(), 0, "a hidden panel must not confirm 忘れさせる");
    touch(w, 0, x, y, Ended);
    elapse(2000);
    assert_eq!(confirmed.get(), 0);
}
