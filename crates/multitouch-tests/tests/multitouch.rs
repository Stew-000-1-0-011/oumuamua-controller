//! Multitouch behaviour tests (spec §3 / §5).
//!
//! Fingers are injected as `InternalEvent::Touch` through the testing backend.
//! Layout (logical px, window 400x300):
//!   left area   (  0,  0) 100x100
//!   right area  (200,  0) 100x100
//!   flickable   (  0,150) 200x100, button inside at (0,0) 100x50; empty part elsewhere
//!   pinch area  (300,150) 100x100

use i_slint_core::input::TouchPhase;
use i_slint_core::lengths::LogicalPoint;
use i_slint_core::platform::{InternalEvent, WindowEvent};
use slint::ComponentHandle;

slint::slint! {
    export component Ui inherits Window {
        width: 400px;
        height: 300px;

        out property <bool> left-pressed: left.pressed;
        out property <length> left-x: left.mouse-x;
        out property <bool> right-pressed: right.pressed;
        out property <length> right-x: right.mouse-x;
        out property <bool> button-pressed: button.pressed;
        out property <length> flick-y: fl.viewport-y;
        out property <float> pinch-scale: pinch.scale;
        out property <bool> pinch-active: pinch.active;

        left := TouchArea { x: 0px; y: 0px; width: 100px; height: 100px; }
        right := TouchArea { x: 200px; y: 0px; width: 100px; height: 100px; }
        fl := Flickable {
            x: 0px; y: 150px; width: 200px; height: 100px;
            viewport-height: 400px;
            viewport-width: 200px;
            button := TouchArea { x: 0px; y: 0px; width: 100px; height: 50px; }
        }
        pinch := ScaleRotateGestureHandler { x: 300px; y: 150px; width: 100px; height: 100px; }
    }
}

fn touch(window: &slint::Window, id: i32, x: f32, y: f32, phase: TouchPhase) {
    window.dispatch_event(WindowEvent::internal(InternalEvent::Touch {
        id,
        position: LogicalPoint::new(x, y),
        phase,
    }));
}

fn setup() -> Ui {
    i_slint_backend_testing::init_no_event_loop();
    let ui = Ui::new().unwrap();
    ui.show().unwrap();
    ui
}

use TouchPhase::{Cancelled, Ended, Moved, Started};

#[test]
fn f_single_finger_press_move_release() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    assert!(ui.get_left_pressed());
    touch(w, 0, 40., 20., Moved);
    assert!(ui.get_left_pressed());
    assert_eq!(ui.get_left_x(), 40.);
    touch(w, 0, 40., 20., Ended);
    assert!(!ui.get_left_pressed());
}

#[test]
fn cancelled_phase_releases_area() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    assert!(ui.get_left_pressed());
    touch(w, 0, 10., 10., Cancelled);
    assert!(!ui.get_left_pressed());
}

#[test]
fn a_two_areas_moved_independently_by_two_fingers() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    assert!(ui.get_left_pressed(), "left area should stay pressed by finger 0");
    assert!(ui.get_right_pressed(), "right area should be pressed by finger 1");
    touch(w, 0, 50., 30., Moved);
    assert_eq!(ui.get_left_x(), 50.);
    assert_eq!(ui.get_right_x(), 10.);
    touch(w, 1, 270., 30., Moved);
    assert_eq!(ui.get_left_x(), 50., "left must not follow finger 1");
    assert_eq!(ui.get_right_x(), 70.);
    touch(w, 0, 50., 30., Ended);
    assert!(!ui.get_left_pressed());
    assert!(ui.get_right_pressed(), "right still held by finger 1");
    touch(w, 1, 270., 30., Ended);
    assert!(!ui.get_right_pressed());
}

#[test]
fn b_second_finger_on_same_area_is_ignored() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 60., 60., Started);
    assert!(ui.get_left_pressed(), "state must stay finger 0's");
    touch(w, 1, 80., 80., Moved);
    assert_eq!(ui.get_left_x(), 10., "ignored finger must not move the area");
    touch(w, 0, 30., 10., Moved);
    assert_eq!(ui.get_left_x(), 30.);
    touch(w, 1, 80., 80., Ended);
    assert!(ui.get_left_pressed(), "ignored finger lifting must not release");
    touch(w, 0, 30., 10., Ended);
    assert!(!ui.get_left_pressed());
}

#[test]
fn c_flickable_empty_part_ignored_while_button_held() {
    let ui = setup();
    let w = ui.window();
    // finger 0 holds the button inside the flickable
    touch(w, 0, 10., 160., Started);
    assert!(ui.get_button_pressed());
    // finger 1 on the empty part of the same flickable
    touch(w, 1, 150., 200., Started);
    touch(w, 1, 150., 120., Moved);
    touch(w, 1, 150., 100., Moved);
    assert_eq!(ui.get_flick_y(), 0., "ignored finger must not scroll");
    assert!(ui.get_button_pressed(), "button must remain pressed");
    touch(w, 1, 150., 100., Ended);
    touch(w, 0, 10., 160., Ended);
    assert!(!ui.get_button_pressed());
}

#[test]
fn d_two_fingers_in_gesture_handler_pinch() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 320., 200., Started);
    touch(w, 1, 360., 200., Started);
    touch(w, 0, 310., 200., Moved);
    touch(w, 1, 380., 200., Moved);
    assert!(ui.get_pinch_active(), "pinch should be active");
    assert!(ui.get_pinch_scale() > 1.0, "scale = {}", ui.get_pinch_scale());
    touch(w, 0, 310., 200., Ended);
    touch(w, 1, 380., 200., Ended);
}

#[test]
fn e_ignored_finger_not_handed_over_when_owner_lifts() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 60., 60., Started); // ignored
    touch(w, 0, 10., 10., Ended);
    assert!(!ui.get_left_pressed());
    touch(w, 1, 70., 70., Moved);
    assert!(!ui.get_left_pressed(), "ignored finger must not take over");
    touch(w, 1, 70., 70., Ended);
    assert!(!ui.get_left_pressed());
}
