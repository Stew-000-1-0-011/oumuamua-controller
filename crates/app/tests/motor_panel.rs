//! The motor display on the output screen: compact voltages in the band, a 44px tap target,
//! and the detail panel that opens on a tap and closes on a tap or ×.

use std::cell::Cell;
use std::rc::Rc;
use std::time::Duration;

use app::{MainWindow, MotorRow, WheelInfo};
use i_slint_backend_testing::ElementHandle;
use i_slint_core::input::TouchPhase::{self, Ended, Moved, Started};
use i_slint_core::lengths::LogicalPoint;
use i_slint_core::platform::{InternalEvent, WindowEvent};
use slint::{ComponentHandle, Model, ModelRc, VecModel};

fn touch(w: &slint::Window, x: f32, y: f32, phase: TouchPhase) {
    w.dispatch_event(WindowEvent::internal(InternalEvent::Touch { id: 0, position: LogicalPoint::new(x, y), phase }));
}

fn tap(win: &MainWindow, x: f32, y: f32) {
    touch(win.window(), x, y, Started);
    touch(win.window(), x, y, Ended);
    elapse(50);
}

fn elapse(ms: u64) {
    i_slint_backend_testing::mock_elapsed_time(Duration::from_millis(ms));
    slint::platform::update_timers_and_animations();
}

fn window(w: f32, h: f32) -> MainWindow {
    thread_local!(static INIT: std::sync::Once = const { std::sync::Once::new() });
    INIT.with(|o| o.call_once(i_slint_backend_testing::init_no_event_loop));
    let win = MainWindow::new().unwrap();
    win.window().set_size(slint::LogicalSize::new(w, h));
    win.set_connected(true);
    win.show().unwrap();
    elapse(50);
    win
}

fn row(name: &str, ok: bool, saturated: bool) -> MotorRow {
    MotorRow {
        name: name.into(), volts: "24.1".into(), amps: "1.23".into(), speed: "-8.6".into(), mode: "速度".into(),
        saturated, vdc_low: false, ok,
    }
}

fn set_motors(win: &MainWindow, n: usize) -> Rc<VecModel<MotorRow>> {
    let names = ["車輪0", "車輪1", "車輪2", "吸引 左", "吸引 右"];
    let rows = Rc::new(VecModel::from((0..n).map(|i| row(names[i], i != 2, i == 1)).collect::<Vec<_>>()));
    win.set_band_motors(ModelRc::from(rows.clone()));
    let chips: Vec<WheelInfo> = (0..n).map(|i| WheelInfo { text: format!("24.{i}V").into(), ok: i != 2 }).collect();
    win.set_band_wheels(ModelRc::from(Rc::new(VecModel::from(chips))));
    elapse(50);
    rows
}

fn count(win: &MainWindow, id: &str) -> usize {
    ElementHandle::find_by_element_id(win, id).count()
}

fn compact_center(win: &MainWindow) -> (f32, f32, f32) {
    let b = ElementHandle::find_by_element_id(win, "OutputScreen::motors-box").next().expect("compact box");
    let (p, s) = (b.absolute_position(), b.size());
    (p.x + s.width / 2., p.y + s.height / 2., s.width)
}

#[test]
fn compact_voltages_show_in_the_band_in_a_small_font() {
    let win = window(800., 360.);
    set_motors(&win, 5);
    let b = ElementHandle::find_by_element_id(&win, "OutputScreen::motors-box").next().unwrap();
    let (p, s) = (b.absolute_position(), b.size());
    assert!(p.y >= 0. && p.y + s.height <= 32.0 + 0.5, "inside the 32px band: {p:?} {s:?}");
    assert!(s.width <= 300., "five short voltages in a small font stay compact: {s:?}");
    assert!(p.x + s.width <= 716., "does not run under the strip");
    assert_eq!(count(&win, "OutputScreen::motor-row"), 0, "closed by default");
}

#[test]
fn tapping_the_compact_area_toggles_the_panel_and_shows_one_row_per_motor() {
    let win = window(800., 360.);
    let rows = set_motors(&win, 5);
    let (cx, cy, _) = compact_center(&win);
    assert!(!win.get_motors_open());
    tap(&win, cx, cy);
    assert!(win.get_motors_open());
    assert_eq!(count(&win, "OutputScreen::motor-row"), 5);
    // Live: a sixth row appears, a seventh disappears again.
    rows.push(row("モータ5", true, false));
    elapse(50);
    assert_eq!(count(&win, "OutputScreen::motor-row"), 6);
    rows.remove(5);
    elapse(50);
    // Tapping the compact area again closes it.
    tap(&win, cx, cy);
    assert!(!win.get_motors_open());
    assert_eq!(count(&win, "OutputScreen::motor-row"), 0);
}

#[test]
fn the_compact_target_is_44px_tall_even_below_the_band() {
    let win = window(800., 360.);
    set_motors(&win, 5);
    let (cx, _, _) = compact_center(&win);
    // The band is 32px; the target reaches 12px into the map (y 0..44 in total).
    for y in [1., 16., 31., 36., 42.] {
        tap(&win, cx, y);
        assert!(win.get_motors_open(), "y={y} should open");
        tap(&win, cx, 16.);
        assert!(!win.get_motors_open());
    }
    // Just below the target the map gets the touch, not the toggle.
    tap(&win, cx, 47.);
    assert!(!win.get_motors_open());
    // Beside the compact area (the position text) does nothing either.
    tap(&win, 4., 16.);
    assert!(!win.get_motors_open());
}

#[test]
fn the_panel_closes_on_a_tap_on_it_or_on_the_cross() {
    let win = window(800., 360.);
    set_motors(&win, 5);
    let (cx, cy, _) = compact_center(&win);
    tap(&win, cx, cy);
    let r = ElementHandle::find_by_element_id(&win, "OutputScreen::motor-row").next().unwrap();
    let rp = r.absolute_position();
    // A row.
    tap(&win, rp.x + 20., rp.y + 10.);
    assert!(!win.get_motors_open());
    // The × at the right end of the title row.
    tap(&win, cx, cy);
    assert!(win.get_motors_open());
    let panel_right = {
        let rows: Vec<_> = ElementHandle::find_by_element_id(&win, "OutputScreen::motor-row").collect();
        rows[0].absolute_position().x + rows[0].size().width + 8.
    };
    tap(&win, panel_right - 22., rp.y - 20. - 20.);
    assert!(!win.get_motors_open());
}

#[test]
fn the_panel_sits_below_the_band_and_fits_every_screen() {
    // 800x336 is 800x360 minus a 24px inset; 640x300 the smallest supported screen.
    for (w, h) in [(800., 360.), (800., 336.), (640., 300.), (914., 411.), (1280., 720.)] {
        let win = window(w, h);
        set_motors(&win, 5);
        let (cx, cy, _) = compact_center(&win);
        tap(&win, cx, cy);
        let rows: Vec<_> = ElementHandle::find_by_element_id(&win, "OutputScreen::motor-row").collect();
        assert_eq!(rows.len(), 5, "{w}x{h}");
        let top = rows[0].absolute_position().y;
        assert!(top >= 32. + 12., "{w}x{h}: rows start below the band, got {top}");
        let last = rows.last().unwrap();
        assert!(last.absolute_position().y + last.size().height <= h, "{w}x{h}: bottom");
        // Everything stays left of the strip (84px wide at the right edge).
        assert!(last.absolute_position().x + last.size().width <= w - 84., "{w}x{h}: right");
    }
}

#[test]
fn an_open_panel_does_not_take_the_stop_button_or_the_strip() {
    let win = window(800., 360.);
    set_motors(&win, 5);
    let stops = Rc::new(Cell::new(0));
    let s = stops.clone();
    win.on_stop_pressed(move || s.set(s.get() + 1));
    let (cx, cy, _) = compact_center(&win);
    tap(&win, cx, cy);
    assert!(win.get_motors_open());
    // 全停止 is the big button at the bottom of the strip (x 716..800).
    tap(&win, 758., 325.);
    assert_eq!(stops.get(), 1);
    assert!(win.get_motors_open(), "a tap outside the panel leaves it open");
    // The strip's page buttons still switch pages, which closes the panel.
    win.set_page(1);
    elapse(50);
    assert!(!win.get_motors_open());
}

#[test]
fn the_map_below_the_panel_still_pans() {
    let win = window(800., 360.);
    set_motors(&win, 5);
    let panned = Rc::new(Cell::new(0));
    let p = panned.clone();
    win.on_map_panned(move |_, _, _| p.set(p.get() + 1));
    let (cx, cy, _) = compact_center(&win);
    tap(&win, cx, cy);
    // The panel ends at y=44+8+40+20+5*28+8 = 260; drag below it.
    touch(win.window(), 300., 300., Started);
    touch(win.window(), 280., 290., Moved);
    touch(win.window(), 250., 280., Moved);
    touch(win.window(), 250., 280., Ended);
    elapse(50);
    assert!(panned.get() > 0);
}

#[test]
fn no_motors_means_no_tap_target_and_no_panel() {
    let win = window(800., 360.);
    set_motors(&win, 0);
    win.set_motors_open(true);
    elapse(50);
    assert_eq!(count(&win, "OutputScreen::motor-row"), 0);
    assert_eq!(win.get_band_motors().row_count(), 0);
}

#[test]
fn leaving_the_output_page_or_the_connection_closes_the_panel() {
    let win = window(800., 360.);
    set_motors(&win, 5);
    win.set_motors_open(true);
    win.set_page(2);
    elapse(50);
    assert!(!win.get_motors_open());
    win.set_page(0);
    win.set_motors_open(true);
    win.set_connected(false);
    elapse(50);
    assert!(!win.get_motors_open());
}
