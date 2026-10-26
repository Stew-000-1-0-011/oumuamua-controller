//! Manual-control logic that does not need Slint: stick/slider mapping, lift/suction targets,
//! labels. `glue.rs` owns an instance and copies its motion into `ControlInput`.

use link::ControlInput;
use proto::{Gripper, Mode, TaskState};

/// Stick deflection (0..1) below which nothing moves.
pub const DEADZONE: f32 = 0.1;
/// How long the reset button must be held (enforced by the Slint timer).
pub const RESET_HOLD_MS: u64 = 1000;
/// Speeds used until CONFIG arrives (protocol.md defaults).
pub const DEFAULT_LEVELS: [(f64, f64); 3] = [(0.3, 1.0), (0.8, 1.8), (1.5, 2.5)];

/// How long after a reset press a confirmation is still believed.
pub const RESET_CONFIRM_WINDOW_MS: u64 = 1500;
/// Manual-page requests older than this no longer wait for their RESULT.
pub const REQ_KEEP_MS: u64 = 10_000;

/// Whether the manual page may drive the robot: it is showing, on a live link, and the robot
/// is not in auto mode (or unknown).
pub fn motion_allowed(connected: bool, has_client: bool, page: i32, mode: Option<Mode>) -> bool {
    connected && has_client && page == 1 && mode.is_some_and(|m| m != Mode::Auto)
}

/// Lift motion, suction-on, lift reset and tracker-on are refused by the robot while it is
/// stopped (or an emergency stop is latched on this phone); the UI does not offer them.
pub fn actuation_blocked(mode: Option<Mode>, estop: bool) -> bool {
    estop || mode == Some(Mode::Stopped)
}

/// Maps a stick deflection to (vx, vy) in m/s. `dx` is positive to the screen's right and `dy`
/// positive to the screen's bottom (Slint coordinates), both in -1..1.
///
/// Robot frame (`field_frame` false): up is +x (forward) and left is +y.
/// Field frame: the stick is a map joystick. The map draws +x to the right and +y up, so up is
/// field +y and right is field +x. The bridge rotates (vx, vy) by the robot's yaw, so only the
/// axes are chosen here.
///
/// Inside the deadzone the result is zero; the deflection length is clamped to 1.
pub fn stick_to_velocity(dx: f32, dy: f32, level: (f64, f64), field_frame: bool) -> (f64, f64) {
    let (dx, dy) = (f64::from(dx), f64::from(dy));
    let len = dx.hypot(dy);
    if !len.is_finite() || len < f64::from(DEADZONE) {
        return (0.0, 0.0);
    }
    let k = if len > 1.0 { 1.0 / len } else { 1.0 };
    let (vx, vy) = if field_frame {
        (dx * k * level.0, -dy * k * level.0)
    } else {
        (-dy * k * level.0, -dx * k * level.0)
    };
    // Avoid "-0.0" on the wire for exact axes.
    (vx + 0.0, vy + 0.0)
}

/// Maps the rotation slider (-1 left .. 1 right) to wz in rad/s. Right is clockwise, which is
/// negative.
pub fn slider_to_omega(s: f32, level: (f64, f64)) -> f64 {
    let s = f64::from(s);
    if !s.is_finite() || s.abs() < f64::from(DEADZONE) {
        return 0.0;
    }
    -s.clamp(-1.0, 1.0) * level.1 + 0.0
}

/// Gripper id on the wire: 0 left, 1 right, 2 both.
pub fn gripper_id(side: u8, both: bool) -> u8 {
    if both { 2 } else { side.min(1) }
}

/// A lift preset is usable only up to the configured wiring limit.
pub fn preset_enabled(height: f64, lift_max: f64) -> bool {
    height <= lift_max + 1e-9
}

pub fn lift_limit_reason(lift_max: f64) -> String {
    format!("配線の都合で {lift_max:.2} m まで")
}

/// "高さ 0.12 m  目標 0.34 m", or dashes while the robot reports nothing.
pub fn height_text(g: Option<&Gripper>) -> String {
    match g {
        Some(g) => format!("高さ {:.2} m  目標 {:.2} m", g.height, g.goal),
        None => "高さ —  目標 —".to_owned(),
    }
}

pub fn task_kind_name(kind: u8) -> &'static str {
    match kind {
        0 => "回収",
        1 => "スラローム入口へ",
        2 => "スタートへ",
        3 => "スラローム往路",
        4 => "スラローム復路",
        5 => "配置",
        6 => "スラローム出口へ",
        _ => "作業",
    }
}

/// What the auto popup says is running: the task and its step, or the path name.
pub fn auto_label(task: Option<&TaskState>, path: Option<&(String, bool)>) -> String {
    if let Some(t) = task {
        let kind = task_kind_name(t.kind);
        return if t.step.is_empty() { kind.to_owned() } else { format!("{kind}: {}", t.step) };
    }
    match path {
        Some((name, _)) => format!("経路 {name}"),
        None => String::new(),
    }
}

/// The user's manual inputs. Everything here is "intent"; `apply` decides what is sent.
#[derive(Debug, Clone, PartialEq)]
pub struct Manual {
    pub level: usize,
    pub field_frame: bool,
    pub both: bool,
    stick: (f32, f32),
    slider: f32,
    jog: [i8; 2],
    /// Per pressing side: the sides that press drove.
    driven: [[bool; 2]; 2],
}

impl Default for Manual {
    fn default() -> Self {
        Manual { level: 0, field_frame: false, both: false, stick: (0.0, 0.0), slider: 0.0, jog: [0, 0], driven: [[false; 2]; 2] }
    }
}

impl Manual {
    pub fn set_stick(&mut self, dx: f32, dy: f32) {
        self.stick = (dx, dy);
    }
    pub fn set_slider(&mut self, s: f32) {
        self.slider = s;
    }
    /// Pressing 上/下 on `side` (0 left, 1 right); `dir` is +1 up, -1 down. With 両方 both move.
    /// The sides this press drove are remembered, so its release stops exactly those, whatever
    /// 両方 says by then.
    pub fn jog_press(&mut self, side: u8, dir: i8) {
        let side = usize::from(side.min(1));
        let driven = if self.both { [true, true] } else { [side == 0, side == 1] };
        for (i, d) in driven.into_iter().enumerate() {
            if d {
                self.jog[i] = dir;
            }
        }
        self.driven[side] = driven;
    }
    /// Lifting a finger from `side`'s 上/下 stops the sides that press drove. Two fingers on 上
    /// and 下 of the same side share that side, so lifting either one stops it.
    pub fn jog_release(&mut self, side: u8) {
        let side = usize::from(side.min(1));
        for (i, d) in self.driven[side].into_iter().enumerate() {
            if d {
                self.jog[i] = 0;
            }
        }
        self.driven[side] = [false, false];
    }
    /// Changes 両方. Any held jog is dropped, since its press was made under the old meaning.
    pub fn set_both(&mut self, both: bool) {
        if self.both != both {
            self.both = both;
            self.jog = [0, 0];
            self.driven = [[false; 2]; 2];
        }
    }
    /// Drops every held input (page left, popup shown, link lost).
    pub fn release_all(&mut self) {
        self.stick = (0.0, 0.0);
        self.slider = 0.0;
        self.jog = [0, 0];
        self.driven = [[false; 2]; 2];
    }
    pub fn level_pair(&self, levels: &[(f64, f64); 3]) -> (f64, f64) {
        levels[self.level.min(2)]
    }

    /// Writes the motion fields of `c` (never `estop`). With `allowed` false everything is zero.
    /// Returns whether `c` changed.
    pub fn apply(&self, c: &mut ControlInput, levels: &[(f64, f64); 3], allowed: bool) -> bool {
        let before = *c;
        if allowed {
            let lv = self.level_pair(levels);
            (c.vx, c.vy) = stick_to_velocity(self.stick.0, self.stick.1, lv, self.field_frame);
            c.wz = slider_to_omega(self.slider, lv);
            c.field_frame = self.field_frame;
            [c.jog_left, c.jog_right] = self.jog;
        } else {
            (c.vx, c.vy, c.wz) = (0.0, 0.0, 0.0);
            c.field_frame = self.field_frame;
            (c.jog_left, c.jog_right) = (0, 0);
        }
        *c != before
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LV: (f64, f64) = (0.8, 1.8);

    #[test]
    fn stick_deadzone_and_clamp() {
        assert_eq!(stick_to_velocity(0.0, 0.0, LV, false), (0.0, 0.0));
        assert_eq!(stick_to_velocity(0.05, -0.05, LV, false), (0.0, 0.0));
        // Just outside the deadzone moves.
        assert!(stick_to_velocity(0.0, -0.11, LV, false).0 > 0.0);
        // Full deflection gives the level's speed; beyond that is clamped to length 1.
        let (vx, vy) = stick_to_velocity(0.0, -1.0, LV, false);
        assert!((vx - 0.8).abs() < 1e-9 && vy == 0.0);
        let (vx, vy) = stick_to_velocity(-3.0, -4.0, LV, false);
        assert!((vx.hypot(vy) - 0.8).abs() < 1e-9);
        assert!(stick_to_velocity(f32::NAN, 0.0, LV, false) == (0.0, 0.0));
    }

    #[test]
    fn stick_axes() {
        let (vx, vy) = stick_to_velocity(0.0, -0.5, LV, false);
        assert!((vx - 0.4).abs() < 1e-9 && vy == 0.0, "up is +vx");
        let (vx, vy) = stick_to_velocity(-0.5, 0.0, LV, false);
        assert!(vx == 0.0 && (vy - 0.4).abs() < 1e-9, "left is +vy");
        let (vx, vy) = stick_to_velocity(0.5, 0.5, LV, false);
        assert!(vx < 0.0 && vy < 0.0, "down-right is backward and to the right");
    }

    #[test]
    fn stick_axes_in_field_frame() {
        // Up is field +y, right is field +x, matching the map (+x right, +y up).
        let (vx, vy) = stick_to_velocity(0.0, -0.5, LV, true);
        assert!(vx == 0.0 && (vy - 0.4).abs() < 1e-9, "up is +y");
        let (vx, vy) = stick_to_velocity(0.5, 0.0, LV, true);
        assert!((vx - 0.4).abs() < 1e-9 && vy == 0.0, "right is +x");
        let (vx, vy) = stick_to_velocity(-0.5, 0.0, LV, true);
        assert!((vx + 0.4).abs() < 1e-9 && vy == 0.0, "left is -x");
        let (vx, vy) = stick_to_velocity(0.0, 0.5, LV, true);
        assert!(vx == 0.0 && (vy + 0.4).abs() < 1e-9, "down is -y");
        // Up-right is +x +y; the length is clamped and the deadzone still applies.
        let (vx, vy) = stick_to_velocity(3.0, -3.0, LV, true);
        assert!(vx > 0.0 && vy > 0.0 && (vx.hypot(vy) - 0.8).abs() < 1e-9);
        assert_eq!(stick_to_velocity(0.05, -0.05, LV, true), (0.0, 0.0));
    }

    #[test]
    fn apply_uses_the_frame_of_the_stick_mapping() {
        let mut c = ControlInput::default();
        let mut m = Manual::default();
        m.set_stick(0.0, -1.0); // up
        m.apply(&mut c, &DEFAULT_LEVELS, true);
        assert!((c.vx - 0.3).abs() < 1e-9 && c.vy == 0.0 && !c.field_frame, "robot: forward");
        m.field_frame = true;
        m.apply(&mut c, &DEFAULT_LEVELS, true);
        assert!(c.vx == 0.0 && (c.vy - 0.3).abs() < 1e-9 && c.field_frame, "field: +y");
        m.set_stick(1.0, 0.0); // right
        m.apply(&mut c, &DEFAULT_LEVELS, true);
        assert!((c.vx - 0.3).abs() < 1e-9 && c.vy == 0.0, "field: +x");
    }

    #[test]
    fn slider_sign() {
        assert!((slider_to_omega(1.0, LV) + 1.8).abs() < 1e-9, "right is clockwise = negative");
        assert!((slider_to_omega(-0.5, LV) - 0.9).abs() < 1e-9);
        assert_eq!(slider_to_omega(0.05, LV), 0.0);
        assert!((slider_to_omega(5.0, LV) + 1.8).abs() < 1e-9);
    }

    #[test]
    fn motion_gating() {
        assert!(motion_allowed(true, true, 1, Some(Mode::Manual)));
        assert!(motion_allowed(true, true, 1, Some(Mode::Stopped)));
        assert!(!motion_allowed(true, true, 1, Some(Mode::Auto)));
        assert!(!motion_allowed(true, true, 1, None));
        assert!(!motion_allowed(true, true, 0, Some(Mode::Manual)));
        assert!(!motion_allowed(false, true, 1, Some(Mode::Manual)));
        assert!(!motion_allowed(true, false, 1, Some(Mode::Manual)));
    }

    #[test]
    fn actuation_blocked_when_stopped() {
        assert!(actuation_blocked(Some(Mode::Stopped), false));
        assert!(actuation_blocked(Some(Mode::Manual), true));
        assert!(!actuation_blocked(Some(Mode::Manual), false));
        assert!(!actuation_blocked(Some(Mode::Auto), false));
        assert!(!actuation_blocked(None, false));
    }

    #[test]
    fn gripper_selection() {
        assert_eq!((gripper_id(0, false), gripper_id(1, false)), (0, 1));
        assert_eq!((gripper_id(0, true), gripper_id(1, true)), (2, 2));
    }

    #[test]
    fn presets_above_lift_max_are_disabled() {
        assert!(preset_enabled(0.01, 0.19));
        assert!(preset_enabled(0.19, 0.19));
        assert!(!preset_enabled(0.35, 0.19));
        assert_eq!(lift_limit_reason(0.19), "配線の都合で 0.19 m まで");
    }

    #[test]
    fn height_text_handles_missing_gripper() {
        assert_eq!(height_text(None), "高さ —  目標 —");
        let g = Gripper { height: 0.123, goal: 0.35, moving: false, arrived: true, suction: false, feedback: false };
        assert_eq!(height_text(Some(&g)), "高さ 0.12 m  目標 0.35 m");
    }

    #[test]
    fn auto_label_prefers_task_then_path() {
        let t = TaskState { kind: 5, step: "差し込み".into(), args: [0; 4] };
        assert_eq!(auto_label(Some(&t), None), "配置: 差し込み");
        assert_eq!(auto_label(None, Some(&("slalom_forward".into(), true))), "経路 slalom_forward");
        assert_eq!(auto_label(None, None), "");
    }

    #[test]
    fn apply_writes_motion_but_never_estop() {
        let levels = DEFAULT_LEVELS;
        let mut m = Manual::default();
        let mut c = ControlInput { estop: true, ..ControlInput::default() };
        m.set_stick(0.0, -1.0);
        m.set_slider(1.0);
        assert!(m.apply(&mut c, &levels, true));
        assert!(c.estop);
        assert!((c.vx - 0.3).abs() < 1e-9 && (c.wz + 1.0).abs() < 1e-9, "level 0 = 低");
        m.level = 2;
        m.apply(&mut c, &levels, true);
        assert!((c.vx - 1.5).abs() < 1e-9 && (c.wz + 2.5).abs() < 1e-9);
        // Not allowed (other page, auto popup): zero at once, estop untouched.
        assert!(m.apply(&mut c, &levels, false));
        assert_eq!((c.vx, c.vy, c.wz), (0.0, 0.0, 0.0));
        assert!(c.estop);
        assert!(!m.apply(&mut c, &levels, false), "no change the second time");
    }

    #[test]
    fn release_zeroes_and_jog_follows_both() {
        let levels = DEFAULT_LEVELS;
        let mut m = Manual::default();
        let mut c = ControlInput::default();
        m.jog_press(1, -1);
        m.apply(&mut c, &levels, true);
        assert_eq!((c.jog_left, c.jog_right), (0, -1));
        m.both = true;
        m.jog_press(0, 1);
        m.apply(&mut c, &levels, true);
        assert_eq!((c.jog_left, c.jog_right), (1, 1));
        m.set_stick(1.0, 0.0);
        m.release_all();
        m.apply(&mut c, &levels, true);
        assert_eq!((c.vx, c.vy, c.wz, c.jog_left, c.jog_right), (0.0, 0.0, 0.0, 0, 0));
    }

    #[test]
    fn jog_release_is_per_side_unless_both() {
        let levels = DEFAULT_LEVELS;
        let mut m = Manual::default();
        let mut c = ControlInput::default();
        m.jog_press(0, 1);
        m.jog_press(1, -1);
        m.apply(&mut c, &levels, true);
        assert_eq!((c.jog_left, c.jog_right), (1, -1));
        m.jog_release(0);
        m.apply(&mut c, &levels, true);
        assert_eq!((c.jog_left, c.jog_right), (0, -1));
        m.both = true;
        m.jog_press(0, 1);
        m.jog_release(1);
        m.apply(&mut c, &levels, true);
        assert_eq!((c.jog_left, c.jog_right), (1, 0), "the right press only drove the right side");
        m.jog_release(0);
        m.apply(&mut c, &levels, true);
        assert_eq!((c.jog_left, c.jog_right), (0, 0));
    }

    #[test]
    fn jog_release_stops_what_the_press_drove() {
        let levels = DEFAULT_LEVELS;
        let mut c = ControlInput::default();
        // 両方 on, press, 両方 off (field set directly), release.
        let mut m = Manual { both: true, ..Manual::default() };
        m.jog_press(0, 1);
        m.both = false;
        m.jog_release(0);
        m.apply(&mut c, &levels, true);
        assert_eq!((c.jog_left, c.jog_right), (0, 0));
        // 両方 off, press left, 両方 on, release.
        let mut m = Manual::default();
        m.jog_press(0, 1);
        m.both = true;
        m.jog_release(0);
        m.apply(&mut c, &levels, true);
        assert_eq!((c.jog_left, c.jog_right), (0, 0));
        // Changing 両方 through set_both drops held jogs.
        let mut m = Manual { both: true, ..Manual::default() };
        m.jog_press(0, 1);
        m.set_both(false);
        m.apply(&mut c, &levels, true);
        assert_eq!((c.jog_left, c.jog_right), (0, 0));
    }

    #[test]
    fn field_frame_is_passed_through() {
        let mut m = Manual { field_frame: true, ..Manual::default() };
        let mut c = ControlInput::default();
        m.apply(&mut c, &DEFAULT_LEVELS, false);
        assert!(c.field_frame);
        m.field_frame = false;
        m.apply(&mut c, &DEFAULT_LEVELS, true);
        assert!(!c.field_frame);
    }
}
