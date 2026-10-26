//! Text for the motor display on the output screen: a compact line of bus voltages in the status
//! band and one row per motor in the detail panel. No Slint here, so it is unit tested.
//!
//! The motors are the three wheels followed by the two suction motors (protocol.md 2.1). A peer
//! that sends only (vdc, ok) leaves the mode "unknown" and the name empty: those show as dashes
//! and get a name from their position.

use proto::{
    MOTOR_MODE_CURRENT, MOTOR_MODE_DISABLED, MOTOR_MODE_FAULT, MOTOR_MODE_POSITION, MOTOR_MODE_VELOCITY,
    MOTOR_SATURATED_LOWER, MOTOR_SATURATED_UPPER, MOTOR_VDC_LOW, Wheel,
};

/// Wheels come first; the rest are suction motors.
const WHEELS: usize = 3;
const SUCTION_NAMES: [&str; 2] = ["吸引 左", "吸引 右"];

/// One voltage in the band.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Chip {
    pub text: String,
    /// False for a stale or faulted motor (drawn red).
    pub ok: bool,
}

/// One row of the detail panel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DetailRow {
    pub name: String,
    pub volts: String,
    pub amps: String,
    pub speed: String,
    pub mode: String,
    pub saturated: bool,
    pub vdc_low: bool,
    pub ok: bool,
}

/// The label for motor `i`: the name the robot sent, else 車輪N / 吸引 左 / 吸引 右.
pub fn display_name(i: usize, w: &Wheel) -> String {
    if !w.name.is_empty() {
        return w.name.clone();
    }
    match i.checked_sub(WHEELS).and_then(|k| SUCTION_NAMES.get(k)) {
        Some(n) => (*n).to_owned(),
        None if i < WHEELS => format!("車輪{i}"),
        None => format!("モータ{i}"),
    }
}

fn is_suction(i: usize, w: &Wheel) -> bool {
    w.name.starts_with('吸') || (w.name.is_empty() && i >= WHEELS)
}

fn number(v: f64, decimals: usize) -> String {
    if v.is_finite() { format!("{v:.decimals$}") } else { "-".to_owned() }
}

/// Mode in Japanese. A motor that is not ok and not in fault is not reporting (途絶).
pub fn mode_text(w: &Wheel) -> &'static str {
    match w.mode {
        MOTOR_MODE_FAULT => "異常",
        _ if !w.ok => "途絶",
        MOTOR_MODE_DISABLED => "無効",
        MOTOR_MODE_CURRENT => "電流",
        MOTOR_MODE_VELOCITY => "速度",
        MOTOR_MODE_POSITION => "位置",
        _ => "-",
    }
}

/// Whether the entry carries current / velocity (an older peer sends neither).
fn has_detail(w: &Wheel) -> bool {
    w.mode <= MOTOR_MODE_FAULT
}

/// Supply voltages for the band, e.g. `24.1V`, with `吸` in front of the suction motors.
pub fn compact(wheels: &[Wheel]) -> Vec<Chip> {
    wheels
        .iter()
        .enumerate()
        .map(|(i, w)| {
            let v = number(w.vdc, 1);
            let text = match (v.as_str(), is_suction(i, w)) {
                ("-", true) => "吸-".to_owned(),
                ("-", false) => "-".to_owned(),
                (v, true) => format!("吸{v}V"),
                (v, false) => format!("{v}V"),
            };
            Chip { text, ok: w.ok }
        })
        .collect()
}

/// One row per motor for the detail panel.
pub fn detail(wheels: &[Wheel]) -> Vec<DetailRow> {
    wheels
        .iter()
        .enumerate()
        .map(|(i, w)| {
            let known = has_detail(w);
            DetailRow {
                name: display_name(i, w),
                volts: number(w.vdc, 1),
                amps: if known { number(w.current, 2) } else { "-".to_owned() },
                speed: if known { number(w.velocity, 1) } else { "-".to_owned() },
                mode: mode_text(w).to_owned(),
                saturated: w.flags & (MOTOR_SATURATED_UPPER | MOTOR_SATURATED_LOWER) != 0,
                vdc_low: w.flags & MOTOR_VDC_LOW != 0,
                ok: w.ok,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn w(name: &str, vdc: f64, ok: bool, current: f64, velocity: f64, mode: u8, flags: u8) -> Wheel {
        Wheel { vdc, ok, current, velocity, mode, flags, name: name.into() }
    }

    fn five() -> Vec<Wheel> {
        vec![
            w("車輪0", 24.14, true, 1.234, -8.56, 2, 0),
            w("車輪1", 23.96, true, 0.0, 0.0, 2, 0),
            w("車輪2", 0.0, false, 0.0, 0.0, 4, 0),
            w("吸引 左", 24.0, true, 0.0, 0.0, 0, 0),
            w("吸引 右", 23.5, true, 2.5, 300.0, 1, 5),
        ]
    }

    #[test]
    fn compact_is_one_short_voltage_per_motor_and_red_when_not_ok() {
        let c = compact(&five());
        let text: Vec<&str> = c.iter().map(|c| c.text.as_str()).collect();
        assert_eq!(text, ["24.1V", "24.0V", "0.0V", "吸24.0V", "吸23.5V"]);
        assert_eq!(c.iter().map(|c| c.ok).collect::<Vec<_>>(), [true, true, false, true, true]);
        assert!(c.iter().all(|c| c.text.chars().count() <= 7), "stays short");
    }

    #[test]
    fn detail_has_one_row_per_motor_with_current_speed_and_japanese_mode() {
        let rows = detail(&five());
        assert_eq!(rows.len(), 5);
        assert_eq!(
            rows[0],
            DetailRow {
                name: "車輪0".into(), volts: "24.1".into(), amps: "1.23".into(), speed: "-8.6".into(),
                mode: "速度".into(), saturated: false, vdc_low: false, ok: true,
            }
        );
        assert_eq!(rows.iter().map(|r| r.mode.as_str()).collect::<Vec<_>>(), ["速度", "速度", "異常", "無効", "電流"]);
        assert!(!rows[2].ok);
        assert_eq!((rows[4].saturated, rows[4].vdc_low), (true, true));
        assert_eq!((rows[4].amps.as_str(), rows[4].speed.as_str()), ("2.50", "300.0"));
    }

    #[test]
    fn mode_names_cover_every_mode() {
        let m = |mode, ok| mode_text(&w("", 24.0, ok, 0.0, 0.0, mode, 0));
        assert_eq!([m(0, true), m(1, true), m(2, true), m(3, true), m(4, false)], ["無効", "電流", "速度", "位置", "異常"]);
        assert_eq!(m(4, true), "異常");
        assert_eq!(m(2, false), "途絶", "stale but not faulted");
        assert_eq!(m(255, true), "-");
    }

    #[test]
    fn saturation_either_way_marks_it() {
        for flags in [MOTOR_SATURATED_UPPER, MOTOR_SATURATED_LOWER] {
            let r = detail(&[w("車輪0", 24.0, true, 0.0, 0.0, 2, flags)]);
            assert!(r[0].saturated && !r[0].vdc_low);
        }
        let r = detail(&[w("車輪0", 24.0, true, 0.0, 0.0, 2, MOTOR_VDC_LOW)]);
        assert!(!r[0].saturated && r[0].vdc_low);
    }

    #[test]
    fn an_older_peer_gives_voltage_only_with_names_by_position() {
        let old: Vec<Wheel> = (0..5).map(|i| Wheel::legacy(24.0 - i as f64, true)).collect();
        let rows = detail(&old);
        let names: Vec<&str> = rows.iter().map(|r| r.name.as_str()).collect();
        assert_eq!(names, ["車輪0", "車輪1", "車輪2", "吸引 左", "吸引 右"]);
        assert!(rows.iter().all(|r| r.amps == "-" && r.speed == "-" && r.mode == "-" && r.ok));
        assert_eq!(rows[1].volts, "23.0");
        let c = compact(&old);
        assert_eq!((c[0].text.as_str(), c[3].text.as_str()), ("24.0V", "吸21.0V"));
        assert_eq!(display_name(7, &old[0]), "モータ7");
    }

    #[test]
    fn non_finite_numbers_are_dashes_not_nan() {
        let r = detail(&[w("車輪0", f64::NAN, true, f64::INFINITY, f64::NAN, 2, 0)]);
        assert_eq!((r[0].volts.as_str(), r[0].amps.as_str(), r[0].speed.as_str()), ("-", "-", "-"));
        assert_eq!(compact(&[w("車輪0", f64::NAN, true, 0.0, 0.0, 2, 0)])[0].text, "-");
    }

    #[test]
    fn nothing_reported_yet_is_not_ok_and_stale() {
        let never = Wheel { ok: false, ..Wheel::legacy(0.0, false) };
        let r = detail(&[never]);
        assert_eq!((r[0].ok, r[0].mode.as_str()), (false, "途絶"));
    }
}
