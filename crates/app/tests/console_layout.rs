//! The console on the output screen: the keyboard must fit inside the window at any size (the
//! panel derives its height budget itself), and the output must scroll by a vertical drag while
//! a horizontal swipe still switches tabs.

use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use app::{ConsoleRow, ConsoleSpan, ConsoleState, MainWindow};
use i_slint_backend_testing::ElementHandle;
use i_slint_core::input::TouchPhase::{self, Ended, Moved, Started};
use i_slint_core::lengths::LogicalPoint;
use i_slint_core::platform::{InternalEvent, WindowEvent};
use slint::{ComponentHandle, Model, ModelRc, SharedString, VecModel};

fn touch(w: &slint::Window, x: f32, y: f32, phase: TouchPhase) {
    w.dispatch_event(WindowEvent::internal(InternalEvent::Touch { id: 0, position: LogicalPoint::new(x, y), phase }));
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

fn row(text: &str) -> ConsoleRow {
    let span = ConsoleSpan {
        text: text.into(),
        fg: slint::Color::from_rgb_u8(220, 220, 220),
        bg: slint::Color::from_argb_u8(0, 0, 0, 0),
        has_bg: false,
        bold: false,
    };
    ConsoleRow { folded: false, hidden: 0, block: 0, spans: ModelRc::from(Rc::new(VecModel::from(vec![span]))) }
}

fn tab(name: &str, active: bool) -> app::ConsoleTabInfo {
    app::ConsoleTabInfo { name: name.into(), active }
}

/// A shell tab with `n` output rows. Returns the row model.
fn shell(win: &MainWindow, n: usize, keyboard: bool) -> Rc<VecModel<ConsoleRow>> {
    let cs = win.global::<ConsoleState>();
    cs.set_tabs(ModelRc::from(Rc::new(VecModel::from(vec![tab("rosout", false), tab("a", true)]))));
    cs.set_is_shell(true);
    let rows = Rc::new(VecModel::from((0..n).map(|i| row(&format!("line {i}"))).collect::<Vec<_>>()));
    cs.set_rows(ModelRc::from(rows.clone()));
    cs.set_kbd_shown(keyboard);
    elapse(50);
    rows
}

fn find(win: &MainWindow, id: &str) -> ElementHandle {
    ElementHandle::find_by_element_id(win, id).next().unwrap_or_else(|| panic!("no element {id}"))
}

fn bottom_of(e: &ElementHandle) -> f32 {
    e.absolute_position().y + e.size().height
}

#[test]
fn keyboard_fits_inside_the_window_at_every_size() {
    // (width, height); 800x336 is 800x360 minus a 24px bottom inset (the content area is what
    // the console gets), 640x300 the smallest supported screen.
    let sizes = [(800., 360.), (800., 336.), (640., 300.), (640., 324.), (800., 400.), (1000., 700.), (900., 500.)];
    for (w, h) in sizes {
        for cands in [0usize, 4] {
            let win = window(w, h);
            shell(&win, 5, true);
            let cs = win.global::<ConsoleState>();
            let c: Vec<SharedString> = (0..cands).map(|i| format!("cand{i}").into()).collect();
            cs.set_candidates(ModelRc::from(Rc::new(VecModel::from(c))));
            cs.set_quick(ModelRc::from(Rc::new(VecModel::from(vec![SharedString::from("ls")]))));
            elapse(50);
            let kbd = ElementHandle::find_by_element_type_name(&win, "Keyboard").next().expect("no keyboard");
            let (pos, size) = (kbd.absolute_position(), kbd.size());
            assert!(pos.y >= 0. && bottom_of(&kbd) <= h + 0.01, "{w}x{h} cands={cands}: keyboard {pos:?} {size:?} leaves the window");
            assert!(pos.x >= 0. && pos.x + size.width <= w + 0.01, "{w}x{h}: keyboard sticks out sideways");
            // 32px keys wherever the panel has the room for them.
            let enter = ElementHandle::find_by_accessible_label(&win, "Enter").next().unwrap();
            assert!(enter.size().height >= 32., "{w}x{h}: Enter is only {}px tall", enter.size().height);
            assert!(bottom_of(&enter) <= h + 0.01);
            // The input line stays above the keyboard and output keeps room.
            let lv = find(&win, "ConsolePanel::lv");
            assert!(lv.size().height >= 39.9, "{w}x{h} cands={cands}: output {}px", lv.size().height);
            assert!(bottom_of(&lv) <= pos.y + 0.01);
        }
    }
}

#[test]
fn keyboard_never_overflows_on_tiny_windows_and_keeps_a_way_to_close_it() {
    for (w, h) in [(640., 260.), (640., 240.)] {
        let win = window(w, h);
        shell(&win, 5, true);
        let kbd = ElementHandle::find_by_element_type_name(&win, "Keyboard").next().expect("no keyboard");
        assert!(bottom_of(&kbd) <= h + 0.01, "{w}x{h}: keyboard bottom {}", bottom_of(&kbd));
    }
}

#[test]
fn enter_is_clickable_at_800x360_and_800x336() {
    for h in [360., 336.] {
        let win = window(800., h);
        shell(&win, 5, true);
        let keys = Rc::new(RefCell::new(Vec::<String>::new()));
        let k = keys.clone();
        win.global::<ConsoleState>().on_key(move |c| k.borrow_mut().push(c.to_string()));
        let enter = ElementHandle::find_by_accessible_label(&win, "Enter").next().unwrap();
        let (p, s) = (enter.absolute_position(), enter.size());
        assert!(p.y + s.height <= h + 0.01, "{h}: Enter bottom {} outside the window", p.y + s.height);
        // Click near the bottom edge of the key.
        let (x, y) = (p.x + s.width / 2., p.y + s.height - 3.);
        touch(win.window(), x, y, Started);
        touch(win.window(), x, y, Ended);
        elapse(50);
        assert_eq!(*keys.borrow(), vec!["Enter".to_string()], "at {h}");
    }
}

#[test]
fn the_keyboard_can_be_closed_when_the_tab_row_is_dropped() {
    let win = window(800., 336.);
    shell(&win, 5, true);
    // No room for the tab row at this size: a button on the input line closes the keyboard.
    let cs = win.global::<ConsoleState>();
    assert!(cs.get_kbd_shown());
    let input = find(&win, "ConsolePanel::lv");
    // The ▼ chip is the last element inside the input row, right edge of the console.
    let panel_right = 800. - 84.;
    let y = input.absolute_position().y + input.size().height + 22.;
    touch(win.window(), panel_right - 30., y, Started);
    touch(win.window(), panel_right - 30., y, Ended);
    elapse(50);
    assert!(!cs.get_kbd_shown(), "the ▼ chip did not close the keyboard");
}

struct Scene {
    win: MainWindow,
    rows: Rc<VecModel<ConsoleRow>>,
    swipes: Rc<RefCell<Vec<i32>>>,
}

fn scene(n: usize) -> Scene {
    let win = window(800., 360.);
    let rows = shell(&win, n, false);
    let swipes = Rc::new(RefCell::new(Vec::new()));
    let s = swipes.clone();
    win.global::<ConsoleState>().on_swipe(move |d| s.borrow_mut().push(d));
    elapse(50);
    Scene { win, rows, swipes }
}

fn drag(w: &slint::Window, from: (f32, f32), to: (f32, f32)) {
    drag_paced(w, from, to, 0, 16);
}

fn drag_paced(w: &slint::Window, from: (f32, f32), to: (f32, f32), hold: u64, step: u64) {
    touch(w, from.0, from.1, Started);
    elapse(hold);
    for i in 1..=8 {
        let t = i as f32 / 8.;
        touch(w, from.0 + (to.0 - from.0) * t, from.1 + (to.1 - from.1) * t, Moved);
        elapse(step);
    }
    touch(w, to.0, to.1, Ended);
    elapse(300);
}

fn centre(e: &ElementHandle) -> (f32, f32) {
    let (p, s) = (e.absolute_position(), e.size());
    (p.x + s.width / 2., p.y + s.height / 2.)
}

#[test]
fn a_vertical_drag_scrolls_the_output_and_follow_stops_while_scrolled_up() {
    let sc = scene(120);
    let cs = sc.win.global::<ConsoleState>();
    let lv = find(&sc.win, "ConsolePanel::lv");
    let (cx, cy) = centre(&lv);
    // Starts at the bottom (followed).
    assert!(cs.get_view_stuck());
    let bottom_y = cs.get_view_y();
    assert!(bottom_y < -100., "content-y {bottom_y} should be at the end of 120 rows");

    // Finger moves down: earlier rows come into view.
    drag(sc.win.window(), (cx, cy - 30.), (cx, cy + 40.));
    let y1 = cs.get_view_y();
    assert!(y1 > bottom_y + 40., "content-y {y1} did not move up from {bottom_y}");
    assert!(!cs.get_view_stuck(), "still following after scrolling up");
    assert!(sc.swipes.borrow().is_empty(), "a vertical drag switched tabs");

    // New output while scrolled up does not move the view.
    for i in 0..10 {
        sc.rows.push(row(&format!("new {i}")));
    }
    elapse(100);
    assert_eq!(cs.get_view_y(), y1, "new output pulled the view");
    assert!(!cs.get_view_stuck());

    // Scrolling back to the bottom resumes the follow.
    for _ in 0..12 {
        drag(sc.win.window(), (cx, cy + 40.), (cx, cy - 40.));
    }
    assert!(cs.get_view_stuck(), "not following at the bottom (y {})", cs.get_view_y());
    let n = sc.rows.row_count();
    sc.rows.push(row("tail"));
    elapse(100);
    assert!(cs.get_view_stuck());
    assert_eq!(sc.rows.row_count(), n + 1);

    // A horizontal drag on the output neither scrolls nor switches tabs.
    let before = cs.get_view_y();
    drag(sc.win.window(), (cx + 60., cy), (cx - 60., cy));
    assert_eq!(cs.get_view_y(), before, "a horizontal drag scrolled the output");
    assert!(sc.swipes.borrow().is_empty());
}

#[test]
fn a_horizontal_swipe_on_the_tab_bar_switches_tabs_and_taps_still_work() {
    let sc = scene(5);
    let bar = find(&sc.win, "ConsolePanel::swipe");
    let (cx, cy) = centre(&bar);
    // Over the empty part of the bar and over a chip, at a fast pace.
    for (x0, y) in [(cx + 20., cy), (bar.absolute_position().x + 80., cy)] {
        sc.swipes.borrow_mut().clear();
        drag(sc.win.window(), (x0 + 60., y), (x0 - 60., y));
        assert_eq!(*sc.swipes.borrow(), vec![1], "swipe left from {x0}");
        drag(sc.win.window(), (x0 - 60., y), (x0 + 60., y));
        assert_eq!(*sc.swipes.borrow(), vec![1, -1], "swipe right from {x0}");
    }
    // A tap on the first chip selects its tab (quick tap, 40ms).
    let picked = Rc::new(RefCell::new(Vec::<i32>::new()));
    let p = picked.clone();
    sc.win.global::<ConsoleState>().on_select_tab(move |i| p.borrow_mut().push(i));
    let (x, y) = (bar.absolute_position().x + 30., cy);
    touch(sc.win.window(), x, y, Started);
    elapse(40);
    touch(sc.win.window(), x, y, Ended);
    elapse(300);
    assert_eq!(*picked.borrow(), vec![0]);
    assert!(sc.swipes.borrow().len() == 2, "a tap counted as a swipe");
}

#[test]
fn enter_jumps_to_the_bottom_again() {
    let sc = scene(120);
    let cs = sc.win.global::<ConsoleState>();
    let (cx, cy) = centre(&find(&sc.win, "ConsolePanel::lv"));
    let bottom_y = cs.get_view_y();
    drag(sc.win.window(), (cx, cy - 30.), (cx, cy + 50.));
    assert!(!cs.get_view_stuck());
    cs.set_bottom_seq(cs.get_bottom_seq() + 1);
    elapse(100);
    assert!(cs.get_view_stuck());
    assert_eq!(cs.get_view_y(), bottom_y);
}

