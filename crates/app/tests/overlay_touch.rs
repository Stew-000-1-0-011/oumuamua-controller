//! The manual and auto panels are drawn over the output screen: touches that miss their controls
//! must not reach the map below, and the 地図で姿勢を指定 mode ends when the output screen is left.

use std::cell::Cell;
use std::rc::Rc;
use std::time::Duration;

use app::MainWindow;
use i_slint_core::input::TouchPhase::{self, Ended, Moved, Started};
use i_slint_core::lengths::LogicalPoint;
use i_slint_core::platform::{InternalEvent, WindowEvent};
use slint::ComponentHandle;

// All coordinates below are measured at 800x360 (content area 716x360, status band 32px high,
// map = left 55%): manual overlay with the stick bottom left and the group right of it, auto
// overlay with 回収 left, 配置 right and the button rows at the bottom.
fn touch(w: &slint::Window, x: f32, y: f32, phase: TouchPhase) {
    touch_id(w, 0, x, y, phase);
}

fn touch_id(w: &slint::Window, id: i32, x: f32, y: f32, phase: TouchPhase) {
    w.dispatch_event(WindowEvent::internal(InternalEvent::Touch { id, position: LogicalPoint::new(x, y), phase }));
}

fn elapse(ms: u64) {
    i_slint_backend_testing::mock_elapsed_time(Duration::from_millis(ms));
    slint::platform::update_timers_and_animations();
}

fn window() -> MainWindow {
    i_slint_backend_testing::init_no_event_loop();
    let win = MainWindow::new().unwrap();
    win.window().set_size(slint::LogicalSize::new(800., 360.));
    win.set_connected(true);
    win.show().unwrap();
    elapse(50);
    win
}

/// Drags on the map at `(x, y)` and returns how many pan / zoom events the map reported.
fn drag_and_count(win: &MainWindow, panned: &Cell<u32>, x: f32, y: f32) -> u32 {
    let before = panned.get();
    let w = win.window();
    touch(w, x, y, Started);
    touch(w, x - 20., y - 10., Moved);
    touch(w, x - 50., y - 30., Moved);
    touch(w, x - 50., y - 30., Ended);
    elapse(50);
    panned.get() - before
}

fn count_pans(win: &MainWindow) -> Rc<Cell<u32>> {
    let panned = Rc::new(Cell::new(0));
    let p = panned.clone();
    win.on_map_panned(move |_, _, _| p.set(p.get() + 1));
    panned
}

#[test]
fn touching_the_map_under_the_manual_overlay_does_not_pan_it() {
    let win = window();
    let panned = count_pans(&win);
    win.set_page(0);
    elapse(50);
    assert!(drag_and_count(&win, &panned, 300., 260.) > 0, "control: the map pans on the output screen");
    win.set_page(1);
    elapse(50);
    // A point between the stick, the lifts and the slider.
    assert_eq!(drag_and_count(&win, &panned, 300., 260.), 0);
    // A point under a plate, too.
    assert_eq!(drag_and_count(&win, &panned, 200., 150.), 0);
}

#[test]
fn touching_the_map_under_the_auto_overlay_does_not_pan_it() {
    let win = window();
    let panned = count_pans(&win);
    win.set_page(2);
    elapse(50);
    // Between the pick and place plates.
    assert_eq!(drag_and_count(&win, &panned, 390., 150.), 0);
    // On a plate.
    assert_eq!(drag_and_count(&win, &panned, 150., 80.), 0);
    win.set_page(0);
    elapse(50);
    assert!(drag_and_count(&win, &panned, 390., 150.) > 0, "control: back on the output screen");
}

#[test]
fn pose_pick_mode_ends_when_the_page_changes() {
    let win = window();
    let cancelled = Rc::new(Cell::new(0));
    let c = cancelled.clone();
    win.on_auto_pose_cancelled(move || c.set(c.get() + 1));
    win.set_page(0);
    win.set_pose_pick(true);
    elapse(50);
    assert!(win.get_pose_pick());
    win.set_page(1);
    elapse(50);
    assert!(!win.get_pose_pick(), "leaving page 0 cancels the pick");
    assert_eq!(cancelled.get(), 1);
    // Coming back does not resume it.
    win.set_page(0);
    elapse(50);
    assert!(!win.get_pose_pick());
}

#[test]
fn the_auto_button_opens_the_pick_and_やめる_closes_it_without_confirming() {
    let win = window();
    let confirmed = Rc::new(Cell::new(0));
    let c = confirmed.clone();
    win.on_auto_pose_confirm(move || c.set(c.get() + 1));
    let cancelled = Rc::new(Cell::new(0));
    let c = cancelled.clone();
    win.on_auto_pose_cancelled(move || c.set(c.get() + 1));
    win.set_page(2);
    elapse(50);
    let w = win.window();
    // 地図で姿勢を指定: bottom row, right end.
    touch(w, 630., 332., Started);
    touch(w, 630., 332., Ended);
    elapse(50);
    assert_eq!(win.get_page(), 0);
    assert!(win.get_pose_pick());
    // やめる on the banner.
    touch(w, 335., 66., Started);
    touch(w, 335., 66., Ended);
    elapse(50);
    assert!(!win.get_pose_pick());
    assert_eq!(confirmed.get(), 0);
    assert_eq!(cancelled.get(), 1);
}

fn count_zooms(win: &MainWindow) -> Rc<Cell<u32>> {
    let zoomed = Rc::new(Cell::new(0));
    let z = zoomed.clone();
    win.on_map_zoomed(move |_, _, _, _| z.set(z.get() + 1));
    zoomed
}

/// Two fingers spread apart around (cx, cy); the second finger has id 1.
fn pinch(win: &MainWindow, cx: f32, cy: f32) {
    let w = win.window();
    touch_id(w, 0, cx - 10., cy, Started);
    touch_id(w, 1, cx + 10., cy, Started);
    for d in [20., 40., 60.] {
        touch_id(w, 0, cx - 10. - d, cy, Moved);
        touch_id(w, 1, cx + 10. + d, cy, Moved);
    }
    touch_id(w, 0, cx - 70., cy, Ended);
    touch_id(w, 1, cx + 70., cy, Ended);
    elapse(50);
}

#[test]
fn a_second_finger_on_the_map_under_the_manual_overlay_neither_pans_nor_zooms() {
    let win = window();
    let panned = count_pans(&win);
    let zoomed = count_zooms(&win);
    let sticks = Rc::new(Cell::new(0));
    let s = sticks.clone();
    win.on_manual_stick(move |_, _| s.set(s.get() + 1));
    win.set_page(1);
    elapse(50);
    let w = win.window();
    // Finger 0 holds the stick (bottom left), finger 1 drags on the map area between the plates.
    touch_id(w, 0, 86., 274., Started);
    touch_id(w, 0, 110., 250., Moved);
    elapse(50);
    assert!(sticks.get() > 0, "control: the stick is held");
    touch_id(w, 1, 300., 260., Started);
    touch_id(w, 1, 280., 250., Moved);
    touch_id(w, 1, 250., 230., Moved);
    touch_id(w, 1, 250., 230., Ended);
    elapse(50);
    touch_id(w, 0, 110., 250., Ended);
    elapse(50);
    assert_eq!(panned.get(), 0, "the map must not pan");
    assert_eq!(zoomed.get(), 0, "the map must not zoom");
}

#[test]
fn a_two_finger_pinch_on_the_map_under_the_auto_overlay_does_not_zoom() {
    let win = window();
    let panned = count_pans(&win);
    let zoomed = count_zooms(&win);
    win.set_page(0);
    elapse(50);
    pinch(&win, 300., 150.);
    assert!(zoomed.get() > 0, "control: the pinch zooms on the output screen");
    let (z0, p0) = (zoomed.get(), panned.get());
    win.set_page(2);
    elapse(50);
    // Between the pick and place plates.
    pinch(&win, 390., 150.);
    assert_eq!(zoomed.get(), z0, "no zoom under the auto overlay");
    assert_eq!(panned.get(), p0, "no pan under the auto overlay");
}

#[test]
fn losing_the_connection_ends_a_pose_pick() {
    let win = window();
    let cancelled = Rc::new(Cell::new(0));
    let c = cancelled.clone();
    win.on_auto_pose_cancelled(move || c.set(c.get() + 1));
    win.set_page(0);
    win.set_pose_pick(true);
    elapse(50);
    win.set_connected(false);
    elapse(50);
    assert!(!win.get_pose_pick());
    assert_eq!(cancelled.get(), 1, "glue is told, so its pending pose is cleared");
    // Not picking: losing the connection tells glue nothing.
    win.set_connected(true);
    elapse(50);
    win.set_connected(false);
    elapse(50);
    assert_eq!(cancelled.get(), 1);
}
