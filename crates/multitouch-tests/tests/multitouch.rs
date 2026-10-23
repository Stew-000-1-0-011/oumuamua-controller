//! Multitouch behaviour tests (spec §3 / §5).
//!
//! Fingers are injected as `InternalEvent::Touch` through the testing backend.
//! Layout (logical px, window 400x300):
//!   left area   (  0,  0) 100x100
//!   right area  (200,  0) 100x100
//!   flickable   (  0,150) 200x100, button inside at (0,0) 100x50; empty part elsewhere
//!   pinch area  (300,150) 100x100
//!   popup       (  0,260) 100x 30 (PopupWindow, closed until `open-popup()`)

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
        out property <length> flick-y: fl.content-y;
        out property <float> pinch-scale: pinch.scale;
        out property <bool> pinch-active: pinch.active;
        out property <bool> left-hover: left.has-hover;
        out property <bool> right-hover: right.has-hover;
        in-out property <bool> popup-pressed;
        public function open-popup() { popup.show(); }

        left := TouchArea { x: 0px; y: 0px; width: 100px; height: 100px; }
        right := TouchArea { x: 200px; y: 0px; width: 100px; height: 100px; }
        fl := Flickable {
            x: 0px; y: 150px; width: 200px; height: 100px;
            content-height: 400px;
            content-width: 200px;
            button := TouchArea { x: 0px; y: 0px; width: 100px; height: 50px; }
        }
        pinch := ScaleRotateGestureHandler { x: 300px; y: 150px; width: 100px; height: 100px; }
        popup := PopupWindow {
            x: 0px; y: 260px; width: 100px; height: 30px;
            TouchArea { changed pressed => { root.popup-pressed = self.pressed; } }
        }
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

fn elapse() {
    i_slint_backend_testing::mock_elapsed_time(std::time::Duration::from_millis(500));
}

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
    assert!(
        ui.get_left_pressed(),
        "left area should stay pressed by finger 0"
    );
    assert!(
        ui.get_right_pressed(),
        "right area should be pressed by finger 1"
    );
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
    assert_eq!(
        ui.get_left_x(),
        10.,
        "ignored finger must not move the area"
    );
    touch(w, 0, 30., 10., Moved);
    assert_eq!(ui.get_left_x(), 30.);
    touch(w, 1, 80., 80., Ended);
    assert!(
        ui.get_left_pressed(),
        "ignored finger lifting must not release"
    );
    touch(w, 0, 30., 10., Ended);
    assert!(!ui.get_left_pressed());
}

#[test]
fn c_flickable_empty_part_ignored_while_button_held() {
    let ui = setup();
    let w = ui.window();
    // finger 0 holds the button inside the flickable
    touch(w, 0, 10., 160., Started);
    // the Flickable delays the press of the button; let the delay elapse
    elapse();
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
    assert!(
        ui.get_pinch_scale() > 1.0,
        "scale = {}",
        ui.get_pinch_scale()
    );
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

// ---- Review Focus -------------------------------------------------------------------------

// 1. cleanup of per-finger state

#[test]
fn cleanup_secondary_cancelled_releases_area_and_primary_stays() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    assert!(ui.get_right_pressed());
    touch(w, 1, 210., 10., Cancelled);
    assert!(
        !ui.get_right_pressed(),
        "cancel must release the secondary's area"
    );
    assert!(!ui.get_right_hover());
    assert!(ui.get_left_pressed(), "primary unaffected");
    // the id is free again and the area can be pressed anew
    touch(w, 1, 210., 10., Started);
    assert!(ui.get_right_pressed());
    touch(w, 1, 210., 10., Ended);
    touch(w, 0, 10., 10., Ended);
    assert!(!ui.get_right_pressed() && !ui.get_left_pressed());
}

#[test]
fn cleanup_secondary_leaving_window_then_ended() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    // drag far outside the window; the area keeps following (grab)
    touch(w, 1, 900., 700., Moved);
    assert!(ui.get_right_pressed());
    touch(w, 1, 900., 700., Ended);
    assert!(
        !ui.get_right_pressed(),
        "Ended outside the window must still release"
    );
    assert!(!ui.get_right_hover());
    touch(w, 0, 10., 10., Ended);
    assert!(!ui.get_left_pressed());
}

#[test]
fn cleanup_hide_releases_secondary_fingers() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    assert!(ui.get_right_pressed());
    ui.hide().unwrap();
    assert!(
        !ui.get_right_pressed(),
        "hiding must reset items held by secondary fingers"
    );
    ui.show().unwrap();
    // stale events of the old fingers do nothing, new ones work
    touch(w, 1, 210., 10., Ended);
    assert!(!ui.get_right_pressed());
    touch(w, 2, 210., 10., Started);
    assert!(ui.get_right_pressed());
    touch(w, 2, 210., 10., Ended);
    touch(w, 0, 10., 10., Ended);
}

#[test]
fn cleanup_ignored_finger_forgotten_after_it_lifts() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 60., 60., Started); // ignored
    touch(w, 1, 60., 60., Ended);
    touch(w, 0, 10., 10., Ended);
    // id 1 may now press normally
    touch(w, 1, 10., 10., Started);
    assert!(ui.get_left_pressed());
    touch(w, 1, 10., 10., Ended);
    assert!(!ui.get_left_pressed());
}

#[test]
fn secondary_finger_in_flickable_gets_its_delayed_press() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 10., 160., Started); // button inside the flickable, delayed press
    assert!(!ui.get_button_pressed());
    elapse();
    assert!(
        ui.get_button_pressed(),
        "delay timer must fire on the secondary's own state"
    );
    assert!(ui.get_left_pressed());
    touch(w, 1, 10., 160., Ended);
    assert!(!ui.get_button_pressed());
    assert!(ui.get_left_pressed());
    touch(w, 0, 10., 10., Ended);
}

#[test]
fn three_fingers_three_owners() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    touch(w, 2, 10., 160., Started);
    elapse();
    assert!(ui.get_left_pressed() && ui.get_right_pressed() && ui.get_button_pressed());
    touch(w, 0, 10., 10., Ended);
    touch(w, 1, 210., 10., Ended);
    touch(w, 2, 10., 160., Ended);
    assert!(!ui.get_left_pressed() && !ui.get_right_pressed() && !ui.get_button_pressed());
}

// 2. popups

#[test]
fn popup_open_while_second_finger_lands() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    ui.invoke_open_popup();
    touch(w, 1, 210., 10., Started); // popup open: secondary fingers are ignored
    assert!(!ui.get_right_pressed());
    touch(w, 1, 220., 20., Moved);
    touch(w, 1, 220., 20., Ended);
    touch(w, 0, 10., 10., Ended);
    // the popup still works with the primary finger and closes on click
    touch(w, 2, 10., 270., Started);
    assert!(ui.get_popup_pressed());
    touch(w, 2, 10., 270., Ended);
    touch(w, 3, 10., 270., Started);
    assert!(!ui.get_popup_pressed(), "popup must have closed");
    touch(w, 3, 10., 270., Ended);
}

#[test]
fn popup_opened_while_secondary_finger_is_down() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 210., 10., Started);
    touch(w, 1, 10., 10., Started); // secondary on left
    assert!(ui.get_left_pressed());
    ui.invoke_open_popup();
    touch(w, 1, 10., 10., Ended); // must not panic or leave the area pressed
    assert!(!ui.get_left_pressed());
    // primary press outside the popup closes it
    touch(w, 0, 210., 10., Ended);
    touch(w, 2, 210., 100., Started);
    touch(w, 2, 210., 100., Ended);
    touch(w, 3, 10., 270., Started);
    assert!(!ui.get_popup_pressed());
    touch(w, 3, 10., 270., Ended);
}

// 3. map pinch

#[test]
fn pinch_unchanged_with_other_finger_on_independent_area() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 320., 200., Started);
    touch(w, 1, 360., 200., Started);
    touch(w, 2, 10., 10., Started); // independent third finger
    assert!(ui.get_left_pressed());
    touch(w, 0, 310., 200., Moved);
    touch(w, 1, 380., 200., Moved);
    assert!(ui.get_pinch_active());
    let scale = ui.get_pinch_scale();
    assert!(scale > 1.0, "scale = {scale}");
    touch(w, 2, 20., 10., Moved);
    assert_eq!(ui.get_left_x(), 20.);
    touch(w, 0, 310., 200., Ended);
    touch(w, 1, 380., 200., Ended);
    assert!(!ui.get_pinch_active());
    assert!(
        ui.get_left_pressed(),
        "pinch ending must not touch the other finger"
    );
    touch(w, 2, 20., 10., Ended);
    assert!(!ui.get_left_pressed());
}

#[test]
fn pinch_remaining_finger_continues_after_one_lifts() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 320., 200., Started);
    touch(w, 1, 360., 200., Started);
    touch(w, 0, 310., 200., Moved);
    touch(w, 1, 380., 200., Moved);
    assert!(ui.get_pinch_active());
    touch(w, 0, 310., 200., Ended);
    assert!(!ui.get_pinch_active());
    touch(w, 1, 380., 200., Ended);
}

#[test]
fn no_pinch_when_fingers_are_in_different_places() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started); // outside the handler
    touch(w, 1, 360., 200., Started); // inside the handler
    touch(w, 0, 5., 10., Moved);
    touch(w, 1, 390., 200., Moved);
    assert!(
        !ui.get_pinch_active(),
        "only two fingers inside the same handler pinch"
    );
    assert!(ui.get_left_pressed());
    touch(w, 0, 5., 10., Ended);
    touch(w, 1, 390., 200., Ended);
}

#[test]
fn third_finger_in_pinch_handler_is_ignored() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 320., 200., Started);
    touch(w, 1, 360., 200., Started);
    touch(w, 2, 340., 220., Started);
    touch(w, 0, 300., 200., Moved);
    touch(w, 1, 390., 200., Moved);
    assert!(ui.get_pinch_active());
    touch(w, 2, 340., 220., Ended);
    assert!(ui.get_pinch_active());
    touch(w, 0, 300., 200., Ended);
    touch(w, 1, 390., 200., Ended);
}

// 4. hover / cursor with a primary that lifts first

#[test]
fn primary_lifts_secondary_keeps_hover_and_press() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    assert!(ui.get_left_hover() && ui.get_right_hover());
    touch(w, 0, 10., 10., Ended);
    assert!(!ui.get_left_hover() && !ui.get_left_pressed());
    assert!(
        ui.get_right_hover() && ui.get_right_pressed(),
        "secondary state must survive"
    );
    touch(w, 1, 250., 30., Moved);
    assert_eq!(ui.get_right_x(), 50.);
    // a new finger takes the (now free) primary state
    touch(w, 2, 10., 10., Started);
    assert!(ui.get_left_pressed() && ui.get_left_hover());
    touch(w, 2, 10., 10., Ended);
    assert!(!ui.get_left_hover());
    assert!(ui.get_right_pressed());
    touch(w, 1, 250., 30., Ended);
    assert!(!ui.get_right_hover() && !ui.get_right_pressed());
}

#[test]
fn secondary_lifts_primary_keeps_hover() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    touch(w, 1, 210., 10., Ended);
    assert!(!ui.get_right_hover());
    assert!(ui.get_left_hover() && ui.get_left_pressed());
    touch(w, 0, 10., 10., Ended);
    assert!(!ui.get_left_hover());
}

// mouse is a finger too

#[test]
fn held_mouse_button_owns_its_area_against_touch() {
    use slint::platform::{PointerEventButton, WindowEvent as WE};
    let ui = setup();
    let w = ui.window();
    w.dispatch_event(WE::PointerPressed {
        position: slint::LogicalPosition::new(10., 10.),
        button: PointerEventButton::Left,
    });
    assert!(ui.get_left_pressed());
    touch(w, 0, 60., 60., Started); // same area: ignored
    touch(w, 1, 210., 10., Started); // other area: independent
    assert!(ui.get_right_pressed());
    touch(w, 0, 60., 60., Ended);
    touch(w, 1, 210., 10., Ended);
    assert!(ui.get_left_pressed(), "mouse still holds the area");
    w.dispatch_event(WE::PointerReleased {
        position: slint::LogicalPosition::new(10., 10.),
        button: PointerEventButton::Left,
    });
    assert!(!ui.get_left_pressed());
}

// ---- fix round 1 --------------------------------------------------------------------------

#[test]
fn pinch_by_two_secondary_fingers_while_primary_holds_an_area() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started); // primary holds left
    touch(w, 1, 320., 200., Started); // secondary inside the handler
    touch(w, 2, 360., 200., Started); // second finger in the handler -> pinch
    touch(w, 1, 310., 200., Moved);
    touch(w, 2, 380., 200., Moved);
    assert!(
        ui.get_pinch_active(),
        "two fingers in one handler must pinch"
    );
    assert!(ui.get_pinch_scale() > 1.0);
    assert!(ui.get_left_pressed(), "primary keeps its area");
    touch(w, 0, 40., 10., Moved);
    assert_eq!(ui.get_left_x(), 40.);
    // one pinch finger lifts: the pinch ends, the other carries on
    touch(w, 1, 310., 200., Ended);
    assert!(!ui.get_pinch_active());
    touch(w, 2, 380., 200., Ended);
    assert!(ui.get_left_pressed());
    touch(w, 0, 40., 10., Ended);
    assert!(!ui.get_left_pressed());
}

#[test]
fn pinch_by_secondary_pair_other_finger_order_and_second_secondary() {
    let ui = setup();
    let w = ui.window();
    touch(w, 5, 320., 200., Started); // first finger is in the handler (primary)
    touch(w, 0, 10., 10., Started); // secondary on left
    touch(w, 1, 210., 10., Started); // secondary on right
    // primary lifts; a new finger in the handler becomes primary again, the next one pinches
    touch(w, 5, 320., 200., Ended);
    touch(w, 2, 320., 200., Started);
    touch(w, 3, 360., 200., Started);
    touch(w, 2, 300., 200., Moved);
    touch(w, 3, 390., 200., Moved);
    assert!(ui.get_pinch_active());
    assert!(ui.get_left_pressed() && ui.get_right_pressed());
    // here the pair is the (re-taken) primary and a new finger; one of them lifts
    touch(w, 3, 390., 200., Ended);
    assert!(!ui.get_pinch_active());
    touch(w, 2, 300., 200., Ended);
    touch(w, 0, 10., 10., Ended);
    touch(w, 1, 210., 10., Ended);
    assert!(!ui.get_left_pressed() && !ui.get_right_pressed());
}

#[test]
fn pinch_end_repress_onto_item_of_another_finger_is_refused() {
    let ui = setup();
    let w = ui.window();
    touch(w, 2, 10., 10., Started); // primary holds left
    touch(w, 0, 320., 200., Started); // secondary in the handler
    touch(w, 1, 360., 200., Started); // pair
    touch(w, 0, 310., 200., Moved);
    touch(w, 1, 50., 50., Moved); // finger 1 wanders onto the left area
    assert!(ui.get_pinch_active());
    touch(w, 0, 310., 200., Ended); // pinch ends, finger 1 would be pressed on left
    assert!(ui.get_left_pressed(), "left still owned by finger 2");
    touch(w, 1, 70., 70., Moved);
    assert_eq!(
        ui.get_left_x(),
        10.,
        "refused finger must not move the area"
    );
    touch(w, 1, 70., 70., Ended);
    assert!(
        ui.get_left_pressed(),
        "refused finger must not release it either"
    );
    touch(w, 2, 20., 10., Moved);
    assert_eq!(ui.get_left_x(), 20.);
    touch(w, 2, 20., 10., Ended);
    assert!(!ui.get_left_pressed());
}

#[test]
fn stock_pinch_end_repress_onto_item_of_secondary_is_refused() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 320., 200., Started); // primary in handler
    touch(w, 1, 10., 10., Started); // secondary holds left
    touch(w, 2, 360., 200., Started); // pinch with the primary
    touch(w, 0, 310., 200., Moved);
    touch(w, 2, 50., 50., Moved);
    assert!(ui.get_pinch_active());
    touch(w, 0, 310., 200., Ended);
    touch(w, 2, 70., 70., Moved);
    assert_eq!(ui.get_left_x(), 10.);
    touch(w, 2, 70., 70., Ended);
    assert!(ui.get_left_pressed());
    touch(w, 1, 10., 10., Ended);
    assert!(!ui.get_left_pressed());
}

#[test]
fn second_finger_while_first_flickable_press_is_pending() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 160., Started); // press delayed by the Flickable
    assert!(!ui.get_button_pressed());
    touch(w, 1, 150., 200., Started); // empty part of the same Flickable: ignored
    touch(w, 1, 150., 100., Moved);
    assert_eq!(ui.get_flick_y(), 0.);
    elapse();
    assert!(ui.get_button_pressed());
    assert_eq!(ui.get_flick_y(), 0.);
    touch(w, 1, 150., 100., Ended);
    touch(w, 0, 10., 160., Ended);
    assert!(!ui.get_button_pressed());
}

// ---- final review fixes ----

slint::slint! {
    export component ClickUi inherits Window {
        width: 400px;
        height: 300px;
        in-out property <int> clicks;
        out property <bool> a-pressed: a.pressed;
        out property <bool> b-pressed: b.pressed;
        a := TouchArea { x: 0px; y: 0px; width: 100px; height: 100px; clicked => { root.clicks += 1; } }
        b := TouchArea { x: 200px; y: 0px; width: 100px; height: 100px; clicked => { root.clicks += 1; } }
    }
}

fn click_setup() -> ClickUi {
    i_slint_backend_testing::init_no_event_loop();
    let ui = ClickUi::new().unwrap();
    ui.show().unwrap();
    ui
}

#[test]
fn cancel_is_not_a_click_primary_and_secondary_but_ended_clicks() {
    let ui = click_setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    assert!(ui.get_a_pressed());
    touch(w, 0, 10., 10., Cancelled);
    assert!(!ui.get_a_pressed());
    assert_eq!(ui.get_clicks(), 0);

    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    assert!(ui.get_b_pressed());
    touch(w, 1, 210., 10., Cancelled);
    assert!(!ui.get_b_pressed());
    assert_eq!(ui.get_clicks(), 0);
    touch(w, 0, 10., 10., Cancelled);
    assert_eq!(ui.get_clicks(), 0);

    touch(w, 0, 10., 10., Started);
    touch(w, 0, 10., 10., Ended);
    assert_eq!(ui.get_clicks(), 1);
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    touch(w, 1, 210., 10., Ended);
    assert_eq!(ui.get_clicks(), 2);
    touch(w, 0, 10., 10., Ended);
    assert_eq!(ui.get_clicks(), 3);
}

#[test]
fn window_deactivation_clears_secondary_and_ignored_touches() {
    let ui = setup();
    let w = ui.window();
    touch(w, 0, 10., 10., Started);
    touch(w, 1, 210., 10., Started);
    touch(w, 2, 20., 20., Started); // ignored: left area is owned by finger 0
    assert!(ui.get_right_pressed());
    w.dispatch_event(WindowEvent::WindowActiveChanged(false));
    assert!(!ui.get_right_pressed(), "secondary item must be released");
    assert!(!ui.get_right_hover());
    w.dispatch_event(WindowEvent::WindowActiveChanged(true));
    // stale ids are forgotten: id 2 (formerly ignored) can start fresh on a free area
    touch(w, 0, 10., 10., Ended);
    touch(w, 2, 210., 10., Started);
    assert!(ui.get_right_pressed());
    touch(w, 2, 210., 10., Ended);
    assert!(!ui.get_right_pressed());
}
