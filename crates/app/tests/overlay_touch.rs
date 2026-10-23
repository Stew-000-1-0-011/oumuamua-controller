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

fn touch(w: &slint::Window, x: f32, y: f32, phase: TouchPhase) {
    w.dispatch_event(WindowEvent::internal(InternalEvent::Touch { id: 0, position: LogicalPoint::new(x, y), phase }));
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
    touch(w, 335., 106., Started);
    touch(w, 335., 106., Ended);
    elapse(50);
    assert!(!win.get_pose_pick());
    assert_eq!(confirmed.get(), 0);
    assert_eq!(cancelled.get(), 1);
}
