//! Automatic-control page logic that does not need Slint: task labels, shelf/pick rules,
//! button gating. `glue.rs` owns the requests and results.

use crate::model::TaskMemory;
use proto::{Mode, TaskState};

/// `RunTask.action` types (protocol.md: RUN_TASK `kind`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TaskKind {
    Pick = 0,
    ToSlalomEntrance = 1,
    ToStart = 2,
    SlalomForward = 3,
    SlalomBackward = 4,
    Place = 5,
    ToSlalomExit = 6,
}

/// Defaults until CONFIG arrives (protocol.md).
pub const DEFAULT_BLOCKS: u8 = 6;
pub const DEFAULT_COLUMNS: u8 = 3;
pub const DEFAULT_ROWS: u8 = 3;

/// 0 = left = gripper0, 1 = right = gripper1.
pub fn gripper_name(g: i32) -> &'static str {
    match g {
        0 => "左",
        1 => "右",
        _ => "?",
    }
}

/// "回収 左 ブロック 2", "配置 右 列 1 段 0", "スタートへ". `args` is `[gripper, block, column, row]`.
pub fn task_label(kind: u8, args: [i32; 4]) -> String {
    match kind {
        0 => format!("回収 {} ブロック {}", gripper_name(args[0]), args[1]),
        5 => format!("配置 {} 列 {} 段 {}", gripper_name(args[0]), args[2], args[3]),
        1 => "スラロームの入口へ".to_owned(),
        2 => "スタートへ".to_owned(),
        3 => "スラローム往路".to_owned(),
        4 => "スラローム復路".to_owned(),
        6 => "スラロームの出口へ".to_owned(),
        _ => "作業".to_owned(),
    }
}

/// A warning (never a block) when a block would go on a shelf cell whose lower neighbour this
/// controller has not filled.
pub fn place_warning(memory: &TaskMemory, column: u8, row: u8) -> Option<String> {
    if row >= 1 && !memory.placed.contains(&(column, row - 1)) {
        Some("下の段にまだ置いていません".to_owned())
    } else {
        None
    }
}

/// A block that is not yet picked can be picked.
pub fn can_pick(memory: &TaskMemory, block: u8) -> bool {
    !memory.picked.contains(&block)
}

/// The grippers (0 left, 1 right) that hold a block.
pub fn holding_gripper(memory: &TaskMemory) -> Vec<u8> {
    (0u8..2).filter(|&i| memory.holding[usize::from(i)]).collect()
}

/// The gripper PLACE should use: `current` if it holds, else the first that does.
pub fn place_gripper(memory: &TaskMemory, current: u8) -> Option<u8> {
    let h = holding_gripper(memory);
    if h.contains(&current) { Some(current) } else { h.first().copied() }
}

/// Why the task buttons cannot be used now, or `None` when they can.
pub fn task_block_reason(mode: Option<Mode>, task: Option<&TaskState>) -> Option<String> {
    if task.is_some() {
        return Some("作業中のため押せません".to_owned());
    }
    match mode {
        None => Some("ロボットの状態を待っています".to_owned()),
        Some(Mode::Stopped) => Some("停止中のため押せません (解除してください)".to_owned()),
        Some(_) => None,
    }
}

/// "持っている 左 / 回収済み 0, 2 / 置いた 列0 段0, …".
pub fn memory_text(memory: &TaskMemory) -> String {
    let holding = holding_gripper(memory);
    let holding = if holding.is_empty() {
        "なし".to_owned()
    } else {
        holding.iter().map(|&g| gripper_name(i32::from(g))).collect::<Vec<_>>().join("・")
    };
    let picked = if memory.picked.is_empty() {
        "なし".to_owned()
    } else {
        memory.picked.iter().map(u8::to_string).collect::<Vec<_>>().join(", ")
    };
    let placed = if memory.placed.is_empty() {
        "なし".to_owned()
    } else {
        memory.placed.iter().map(|(c, r)| format!("列{c} 段{r}")).collect::<Vec<_>>().join(", ")
    };
    format!("持っている: {holding}\n回収済み: {picked}\n置いた: {placed}")
}

/// The bottom line for a finished task.
pub fn done_text(label: &str, ok: bool, message: &str) -> String {
    match (ok, message.is_empty()) {
        (true, true) => format!("{label}: 成功"),
        (true, false) => format!("{label}: 成功 ({message})"),
        (false, true) => format!("{label}: 失敗"),
        (false, false) => format!("{label}: 失敗 ({message})"),
    }
}

/// The picked pose as the confirmation shows it (yaw in rad in, degrees out).
pub fn pose_text(x: f64, y: f64, yaw: f64) -> String {
    format!("x {x:.2}  y {y:.2} m  向き {:.0}°", yaw.to_degrees())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mem(holding: [bool; 2], picked: &[u8], placed: &[(u8, u8)]) -> TaskMemory {
        TaskMemory { holding, picked: picked.to_vec(), placed: placed.to_vec() }
    }

    #[test]
    fn labels() {
        assert_eq!(task_label(TaskKind::Pick as u8, [0, 2, 0, 0]), "回収 左 ブロック 2");
        assert_eq!(task_label(TaskKind::Place as u8, [1, 0, 1, 0]), "配置 右 列 1 段 0");
        assert_eq!(task_label(TaskKind::ToStart as u8, [0; 4]), "スタートへ");
        assert_eq!(task_label(TaskKind::ToSlalomEntrance as u8, [0; 4]), "スラロームの入口へ");
        assert_eq!(task_label(TaskKind::ToSlalomExit as u8, [0; 4]), "スラロームの出口へ");
        assert_eq!(task_label(TaskKind::SlalomForward as u8, [0; 4]), "スラローム往路");
        assert_eq!(task_label(TaskKind::SlalomBackward as u8, [0; 4]), "スラローム復路");
        assert_eq!(task_label(99, [0; 4]), "作業");
    }

    #[test]
    fn place_warning_rules() {
        let m = mem([true, false], &[], &[(1, 0)]);
        assert_eq!(place_warning(&m, 0, 0), None, "bottom row never warns");
        assert_eq!(place_warning(&m, 1, 1), None, "own block below");
        assert_eq!(place_warning(&m, 0, 1).as_deref(), Some("下の段にまだ置いていません"));
        assert!(place_warning(&m, 1, 2).is_some(), "row 1 of that column is empty");
    }

    #[test]
    fn can_pick_excludes_picked() {
        let m = mem([false, false], &[1, 4], &[]);
        assert!(can_pick(&m, 0));
        assert!(!can_pick(&m, 1));
        assert!(!can_pick(&m, 4));
        assert!(can_pick(&m, 5));
    }

    #[test]
    fn holding_grippers() {
        assert_eq!(holding_gripper(&mem([false, false], &[], &[])), Vec::<u8>::new());
        assert_eq!(holding_gripper(&mem([true, false], &[], &[])), vec![0]);
        assert_eq!(holding_gripper(&mem([false, true], &[], &[])), vec![1]);
        assert_eq!(holding_gripper(&mem([true, true], &[], &[])), vec![0, 1]);
    }

    #[test]
    fn place_gripper_defaults_to_a_holding_one() {
        assert_eq!(place_gripper(&mem([false, false], &[], &[]), 0), None);
        assert_eq!(place_gripper(&mem([false, true], &[], &[]), 0), Some(1));
        assert_eq!(place_gripper(&mem([true, true], &[], &[]), 1), Some(1));
        assert_eq!(place_gripper(&mem([true, false], &[], &[]), 0), Some(0));
    }

    #[test]
    fn task_buttons_gating() {
        let t = TaskState { kind: 0, step: String::new(), args: [0; 4] };
        assert!(task_block_reason(Some(Mode::Manual), None).is_none());
        assert!(task_block_reason(Some(Mode::Auto), None).is_none());
        assert!(task_block_reason(Some(Mode::Manual), Some(&t)).unwrap().contains("作業中"));
        assert!(task_block_reason(Some(Mode::Stopped), None).unwrap().contains("停止中"));
        assert!(task_block_reason(None, None).is_some());
    }

    #[test]
    fn texts() {
        let m = mem([true, false], &[0, 2], &[(1, 0)]);
        assert_eq!(memory_text(&m), "持っている: 左\n回収済み: 0, 2\n置いた: 列1 段0");
        assert!(memory_text(&TaskMemory::default()).contains("持っている: なし"));
        assert_eq!(done_text("スタートへ", true, ""), "スタートへ: 成功");
        assert_eq!(done_text("回収 左 ブロック 1", false, "no block"), "回収 左 ブロック 1: 失敗 (no block)");
        assert_eq!(pose_text(1.0, -2.5, std::f64::consts::FRAC_PI_2), "x 1.00  y -2.50 m  向き 90°");
    }
}
