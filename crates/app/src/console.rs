//! Console state (Slint-free): tabs, line editing, keyboard keys, shell events, persistence.
//!
//! Messages built here carry `req: 0`; the glue stamps a fresh request id before sending
//! (see [`stamp`]).

use crate::model::ShellEvent;
use crate::storage;
use proto::Msg;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};
use term::{Color, EditAction, Line, Span, History, Key, KeyEvent, LineEditor, ModState, Modifier, Modifiers, Row, Scrollback};

pub const ROSOUT: &str = "rosout";
/// Lines kept per tab on the phone.
pub const SCROLLBACK_LINES: usize = 5000;
pub const NAME_MAX: usize = 32;
/// Default LOG_LEVEL: INFO.
pub const DEFAULT_LOG_LEVEL: u8 = 20;
const GLOBAL_FILE: &str = "console_history_global";
const QUICK_FILE: &str = "console_quick";

fn hist_file(tab: &str) -> String {
    format!("console_hist_{tab}")
}

pub fn default_quick_commands() -> Vec<String> {
    ["ls", "pwd", "cd ..", "ros2 topic list", "ros2 node list"].iter().map(|s| (*s).to_owned()).collect()
}

/// A tab name the robot accepts: ASCII `[A-Za-z0-9_-]{1,32}`.
pub fn valid_name(name: &str) -> bool {
    !name.is_empty() && name.len() <= NAME_MAX && name.chars().all(valid_name_char)
}

fn valid_name_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_' || c == '-'
}

/// Put a fresh request id into a request built by the console.
pub fn stamp(msg: Msg, new_req: u32) -> Msg {
    match msg {
        Msg::LogLevel { min_level, .. } => Msg::LogLevel { req: new_req, min_level },
        Msg::ShellOpen { tab, .. } => Msg::ShellOpen { req: new_req, tab },
        Msg::ShellInput { tab, data, .. } => Msg::ShellInput { req: new_req, tab, data },
        Msg::ShellComplete { tab, line, cursor, .. } => Msg::ShellComplete { req: new_req, tab, line, cursor },
        Msg::ShellClose { tab, .. } => Msg::ShellClose { req: new_req, tab },
        Msg::ShellRename { tab, new_name, .. } => Msg::ShellRename { req: new_req, tab, new_name },
        other => other,
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PromptKind {
    New,
    /// Renaming the tab at this index.
    Rename(usize),
}

/// The name entry shown over the console (typed with the on-screen keyboard).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Prompt {
    pub kind: PromptKind,
    pub text: String,
    pub error: Option<String>,
}

struct Tab {
    name: String,
    /// False for the rosout tab.
    shell: bool,
    scroll: Scrollback,
    editor: LineEditor,
    history: History,
    rev: u64,
    cands: Vec<String>,
    cand_start: usize,
    pending_complete: bool,
    /// Request id of the SHELL_COMPLETE in flight, once the glue has sent it.
    complete_req: Option<u32>,
    /// Created before the last (re)connect and not yet confirmed by the robot.
    stale: bool,
}

impl Tab {
    fn new(name: &str, shell: bool, history: History) -> Self {
        Tab {
            name: name.to_owned(),
            shell,
            scroll: Scrollback::new(SCROLLBACK_LINES),
            editor: LineEditor::new(),
            history,
            rev: 0,
            cands: Vec::new(),
            cand_start: 0,
            pending_complete: false,
            complete_req: None,
            stale: false,
        }
    }
}

pub struct Console {
    tabs: Vec<Tab>,
    active: usize,
    global: History,
    /// Histories of closed tabs, kept for when the name is opened again.
    stash: HashMap<String, History>,
    mods: Modifiers,
    sym: bool,
    quick: Vec<String>,
    log_filter: u8,
    prompt: Option<Prompt>,
    dirty: bool,
    clock: u64,
    /// Grows on every Enter, so the view can jump to the bottom.
    submit_seq: u64,
    /// Fixed UTC offset for log times (tests); `None` uses the local zone.
    tz_offset: Option<i64>,
    dir: Option<PathBuf>,
}

impl Default for Console {
    fn default() -> Self {
        Self::new()
    }
}

fn parse_key(cap: &str) -> Option<Key> {
    let mut cs = cap.chars();
    if let (Some(c), None) = (cs.next(), cs.next()) {
        return Some(Key::Char(c));
    }
    Some(match cap {
        "Enter" => Key::Enter,
        "Tab" => Key::Tab,
        "Backspace" => Key::Backspace,
        "Delete" => Key::Delete,
        "Left" => Key::Left,
        "Right" => Key::Right,
        "Up" => Key::Up,
        "Down" => Key::Down,
        "Home" => Key::Home,
        "End" => Key::End,
        "Esc" => Key::Esc,
        _ => return None,
    })
}

fn level_name(level: u8) -> &'static str {
    match level {
        0..=10 => "DEBUG",
        11..=20 => "INFO",
        21..=30 => "WARN",
        31..=40 => "ERROR",
        _ => "FATAL",
    }
}

fn level_sgr(level: u8) -> &'static str {
    match level {
        0..=10 => "\x1b[90m",
        11..=20 => "",
        21..=30 => "\x1b[33m",
        31..=40 => "\x1b[31m",
        _ => "\x1b[1;31m",
    }
}

/// "HH:MM:SS [LEVEL] name: text" (UTC), colored by level, as terminal bytes.
pub fn format_log(stamp_ms: u64, offset: Option<i64>, level: u8, name: &str, text: &str) -> Vec<u8> {
    let utc = offset.is_none();
    let local = (stamp_ms / 1000) as i64 + offset.unwrap_or(0);
    let secs = local.rem_euclid(86_400) as u64;
    let clean = |s: &str| -> String {
        s.chars().map(|c| if c == '\n' || c == '\r' || c == '\t' { ' ' } else { c }).filter(|c| !c.is_control()).collect()
    };
    let sgr = level_sgr(level);
    let reset = if sgr.is_empty() { "" } else { "\x1b[0m" };
    format!(
        "{sgr}{:02}:{:02}:{:02}{} [{}] {}: {}{reset}\r\n",
        secs / 3600,
        secs / 60 % 60,
        secs % 60,
        if utc { "Z" } else { "" },
        level_name(level),
        clean(name),
        clean(text)
    )
    .into_bytes()
}

const ANSI16: [(u8, u8, u8); 16] = [
    (0, 0, 0),
    (205, 49, 49),
    (13, 188, 121),
    (229, 229, 16),
    (36, 114, 200),
    (188, 63, 188),
    (17, 168, 205),
    (229, 229, 229),
    (102, 102, 102),
    (241, 76, 76),
    (35, 209, 139),
    (245, 245, 67),
    (59, 142, 234),
    (214, 112, 214),
    (41, 184, 219),
    (255, 255, 255),
];

/// 16-color, 256-color (cube and gray ramp) and truecolor to RGB.
pub fn palette(c: Color) -> (u8, u8, u8) {
    match c {
        Color::Rgb(r, g, b) => (r, g, b),
        Color::Indexed(i @ 0..=15) => ANSI16[usize::from(i)],
        Color::Indexed(i @ 16..=231) => {
            const LV: [u8; 6] = [0, 95, 135, 175, 215, 255];
            let n = usize::from(i - 16);
            (LV[n / 36], LV[n / 6 % 6], LV[n % 6])
        }
        Color::Indexed(i) => {
            let v = 8 + 10 * (i - 232);
            (v, v, v)
        }
    }
}

/// Local UTC offset in seconds at `secs` since the epoch; `None` when the zone is unknown
/// (the time is then shown as UTC with a trailing Z).
#[cfg(unix)]
fn local_offset(secs: u64) -> Option<i64> {
    use std::ffi::{c_char, c_int, c_long};
    #[repr(C)]
    struct Tm {
        ints: [c_int; 9],
        gmtoff: c_long,
        zone: *const c_char,
    }
    unsafe extern "C" {
        fn localtime_r(t: *const c_long, out: *mut Tm) -> *mut Tm;
    }
    let t = secs as c_long;
    let mut tm = Tm { ints: [0; 9], gmtoff: 0, zone: std::ptr::null() };
    // SAFETY: `t` and `tm` are valid for the call; localtime_r writes only into `tm`.
    let r = unsafe { localtime_r(&t, &mut tm) };
    (!r.is_null()).then_some(tm.gmtoff as i64)
}

#[cfg(not(unix))]
fn local_offset(_secs: u64) -> Option<i64> {
    None
}

/// Cells a character takes: 2 for wide (CJK and similar), 1 otherwise.
fn cells(c: char) -> usize {
    if (c as u32) < 0x2e80 { 1 } else { 2 }
}

/// Split one output row into rows of at most `cols` cells, keeping the styles. A row cannot
/// wrap inside the window (it is a run of separate texts), so it is cut here.
pub fn wrap_row(row: &Row, cols: usize) -> Vec<Row> {
    let Row::Line(line) = row else { return vec![row.clone()] };
    let cols = cols.max(8);
    let mut out: Vec<Line> = Vec::new();
    let mut cur = Line { spans: Vec::new() };
    let mut used = 0;
    for sp in &line.spans {
        let mut text = String::new();
        for c in sp.text.chars() {
            let w = cells(c);
            if used + w > cols {
                if !text.is_empty() {
                    cur.spans.push(Span { text: std::mem::take(&mut text), ..sp.clone() });
                }
                out.push(std::mem::replace(&mut cur, Line { spans: Vec::new() }));
                used = 0;
            }
            text.push(c);
            used += w;
        }
        if !text.is_empty() {
            cur.spans.push(Span { text, ..sp.clone() });
        }
    }
    out.push(cur);
    out.into_iter().map(Row::Line).collect()
}

impl Console {
    pub fn new() -> Self {
        Console {
            tabs: vec![Tab::new(ROSOUT, false, History::new(1))],
            active: 0,
            global: History::new(term::history::DEFAULT_MAX),
            stash: HashMap::new(),
            mods: Modifiers::new(),
            sym: false,
            quick: default_quick_commands(),
            log_filter: DEFAULT_LOG_LEVEL,
            prompt: None,
            dirty: false,
            clock: 0,
            submit_seq: 0,
            tz_offset: None,
            dir: None,
        }
    }

    fn touch(&mut self, i: usize) {
        self.clock += 1;
        self.tabs[i].rev = self.clock;
    }

    // ---- tabs ----

    pub fn tabs(&self) -> Vec<String> {
        self.tabs.iter().map(|t| t.name.clone()).collect()
    }

    pub fn active(&self) -> usize {
        self.active
    }

    pub fn set_active(&mut self, i: usize) {
        if i < self.tabs.len() {
            self.active = i;
        }
    }

    /// Switch by one tab (swipe): `-1` left, `1` right.
    pub fn step_active(&mut self, d: i32) {
        let n = self.tabs.len() as i32;
        self.active = (self.active as i32 + d).clamp(0, n - 1) as usize;
    }

    pub fn active_is_shell(&self) -> bool {
        self.tabs[self.active].shell
    }

    /// Changes whenever the active tab's rows change (and when another tab becomes active).
    pub fn view_rev(&self) -> u64 {
        self.tabs[self.active].rev
    }

    pub fn submit_seq(&self) -> u64 {
        self.submit_seq
    }

    fn find(&self, name: &str) -> Option<usize> {
        self.tabs.iter().position(|t| t.name == name)
    }

    fn make_history(&mut self, name: &str) -> History {
        if let Some(h) = self.stash.remove(name) {
            return h;
        }
        self.dir
            .as_ref()
            .and_then(|d| storage::load_from(d, &hist_file(name)))
            .map_or_else(|| History::new(term::history::DEFAULT_MAX), |t| History::from_text(&t))
    }

    fn push_tab(&mut self, name: &str) -> usize {
        let h = self.make_history(name);
        self.tabs.push(Tab::new(name, true, h));
        let i = self.tabs.len() - 1;
        self.touch(i);
        i
    }

    /// Open a tab locally and ask the robot to open its bash.
    pub fn new_tab(&mut self, name: &str) -> Option<Msg> {
        if !valid_name(name) || name == ROSOUT || self.find(name).is_some() {
            return None;
        }
        self.active = self.push_tab(name);
        Some(Msg::ShellOpen { req: 0, tab: name.to_owned() })
    }

    fn remove_tab(&mut self, i: usize) {
        let t = self.tabs.remove(i);
        self.stash.insert(t.name, t.history);
        if i < self.active || (i == self.active && i >= self.tabs.len()) {
            self.active -= 1;
        }
        self.dirty = true;
    }

    /// Close a tab (the rosout tab cannot be closed); asks the robot to end its bash.
    pub fn close_tab(&mut self, idx: usize) -> Option<Msg> {
        if idx == 0 || idx >= self.tabs.len() {
            return None;
        }
        let name = self.tabs[idx].name.clone();
        self.remove_tab(idx);
        Some(Msg::ShellClose { req: 0, tab: name })
    }

    pub fn rename_tab(&mut self, idx: usize, new_name: &str) -> Option<Msg> {
        if idx == 0 || idx >= self.tabs.len() || !valid_name(new_name) || new_name == ROSOUT || self.find(new_name).is_some() {
            return None;
        }
        let old = std::mem::replace(&mut self.tabs[idx].name, new_name.to_owned());
        self.dirty = true;
        Some(Msg::ShellRename { req: 0, tab: old, new_name: new_name.to_owned() })
    }

    /// Drop a tab the robot refused to open (no message back).
    pub fn forget_tab(&mut self, name: &str) {
        if let Some(i) = self.find(name).filter(|&i| i > 0) {
            self.remove_tab(i);
        }
    }

    /// Undo a refused rename.
    pub fn undo_rename(&mut self, current: &str, old: &str) {
        if let Some(i) = self.find(current).filter(|&i| i > 0)
            && self.find(old).is_none()
        {
            self.tabs[i].name = old.to_owned();
        }
    }

    // ---- reconnect ----

    /// A (re)connect started: tabs now existing on the phone only will be dropped by `end_sync`
    /// unless the robot reports them with SHELL_OPENED.
    pub fn begin_sync(&mut self) {
        for t in self.tabs.iter_mut().filter(|t| t.shell) {
            t.stale = true;
        }
    }

    pub fn end_sync(&mut self) {
        let keep = self.tabs[self.active].name.clone();
        let stale: Vec<usize> = (1..self.tabs.len()).rev().filter(|&i| self.tabs[i].stale).collect();
        for i in stale {
            let t = self.tabs.remove(i);
            self.stash.insert(t.name, t.history);
        }
        self.active = self.find(&keep).unwrap_or(0);
    }

    // ---- keys ----

    pub fn mod_state(&self, m: Modifier) -> ModState {
        self.mods.state(m)
    }

    pub fn sym(&self) -> bool {
        self.sym
    }

    /// One key of the on-screen keyboard (see the task brief for the `cap` names).
    pub fn key(&mut self, cap: &str) -> Vec<Msg> {
        match cap {
            "Ctrl" => return self.tap_mod(Modifier::Ctrl),
            "Alt" => return self.tap_mod(Modifier::Alt),
            "Shift" => return self.tap_mod(Modifier::Shift),
            "Sym" => {
                self.sym = !self.sym;
                return Vec::new();
            }
            _ => {}
        }
        let Some(key) = parse_key(cap) else { return Vec::new() };
        let ev = self.mods.press(key);
        if self.prompt.is_some() {
            return self.prompt_key(ev);
        }
        self.shell_key(ev)
    }

    fn tap_mod(&mut self, m: Modifier) -> Vec<Msg> {
        self.mods.tap(m);
        Vec::new()
    }

    fn shell_key(&mut self, ev: KeyEvent) -> Vec<Msg> {
        let i = self.active;
        if !self.tabs[i].shell {
            return Vec::new();
        }
        let tab = &mut self.tabs[i];
        if ev != KeyEvent::Key(Key::Tab) {
            tab.cands.clear();
            tab.pending_complete = false;
        }
        let action = tab.editor.handle(ev, &mut tab.history, &mut self.global);
        let name = tab.name.clone();
        let mut out = Vec::new();
        match action {
            EditAction::None => {}
            EditAction::Submit(line) => {
                tab.scroll.mark_command();
                self.dirty = true;
                self.submit_seq += 1;
                let mut data = line.into_bytes();
                data.push(b'\n');
                out.push(Msg::ShellInput { req: 0, tab: name, data });
            }
            EditAction::Complete { line, cursor } => {
                tab.pending_complete = true;
                tab.complete_req = None;
                out.push(Msg::ShellComplete { req: 0, tab: name, line, cursor: cursor as u32 });
            }
            EditAction::SendNow(data) => out.push(Msg::ShellInput { req: 0, tab: name, data }),
            EditAction::ClearScreen => tab.scroll.clear(),
        }
        self.touch(i);
        out
    }

    /// Put text into the input line (a quick command).
    pub fn insert_text(&mut self, text: &str) {
        let i = self.active;
        if !self.tabs[i].shell {
            return;
        }
        let tab = &mut self.tabs[i];
        tab.cands.clear();
        tab.pending_complete = false;
        for c in text.chars() {
            tab.editor.handle(KeyEvent::Key(Key::Char(c)), &mut tab.history, &mut self.global);
        }
        self.touch(i);
    }

    pub fn input_display(&self) -> (String, usize) {
        let t = &self.tabs[self.active];
        if t.shell { t.editor.display() } else { (String::new(), 0) }
    }

    pub fn candidates(&self) -> &[String] {
        &self.tabs[self.active].cands
    }

    pub fn pick_candidate(&mut self, i: usize) {
        let a = self.active;
        let tab = &mut self.tabs[a];
        let Some(c) = tab.cands.get(i).cloned() else { return };
        tab.editor.apply_completion(tab.cand_start, &[c]);
        tab.cands.clear();
        self.touch(a);
    }

    // ---- name prompt ----

    pub fn begin_new_tab(&mut self) {
        self.prompt = Some(Prompt { kind: PromptKind::New, text: String::new(), error: None });
    }

    pub fn begin_rename(&mut self, idx: usize) {
        if idx > 0 && idx < self.tabs.len() {
            self.prompt = Some(Prompt { kind: PromptKind::Rename(idx), text: self.tabs[idx].name.clone(), error: None });
        }
    }

    pub fn prompt(&self) -> Option<&Prompt> {
        self.prompt.as_ref()
    }

    pub fn cancel_prompt(&mut self) {
        self.prompt = None;
    }

    fn prompt_key(&mut self, ev: KeyEvent) -> Vec<Msg> {
        let Some(p) = self.prompt.as_mut() else { return Vec::new() };
        match ev {
            KeyEvent::Key(Key::Char(c)) if valid_name_char(c) && p.text.len() < NAME_MAX => {
                p.text.push(c);
                p.error = None;
            }
            KeyEvent::Key(Key::Backspace) => {
                p.text.pop();
                p.error = None;
            }
            KeyEvent::Key(Key::Esc) => self.prompt = None,
            KeyEvent::Key(Key::Enter) => {
                let (kind, text) = (p.kind.clone(), p.text.clone());
                let msg = match kind {
                    PromptKind::New => self.new_tab(&text),
                    PromptKind::Rename(i) => self.rename_tab(i, &text),
                };
                match msg {
                    Some(m) => {
                        self.prompt = None;
                        return vec![m];
                    }
                    None => {
                        if let Some(p) = self.prompt.as_mut() {
                            p.error = Some("その名前は使えません (英数字 _ - 、32 文字まで、重複不可)".to_owned());
                        }
                    }
                }
            }
            _ => {}
        }
        Vec::new()
    }

    // ---- events ----

    pub fn on_event(&mut self, ev: ShellEvent) {
        match ev {
            ShellEvent::Opened { tab, replay } => {
                if tab == ROSOUT || tab.is_empty() {
                    return;
                }
                let i = match self.find(&tab) {
                    Some(i) => i,
                    None => self.push_tab(&tab),
                };
                let t = &mut self.tabs[i];
                t.stale = false;
                t.scroll.replace(&replay);
                self.touch(i);
            }
            ShellEvent::Output { tab, data } => {
                if let Some(i) = self.find(&tab).filter(|&i| self.tabs[i].shell) {
                    self.tabs[i].scroll.feed(&data);
                    self.touch(i);
                }
            }
            ShellEvent::Completion { req, tab, start, candidates } => {
                if let Some(i) = self.find(&tab)
                    && self.tabs[i].pending_complete
                    && self.tabs[i].complete_req.is_none_or(|r| r == req)
                {
                    let t = &mut self.tabs[i];
                    t.pending_complete = false;
                    t.cand_start = start as usize;
                    t.cands = t.editor.apply_completion(start as usize, &candidates);
                    self.touch(i);
                }
            }
            ShellEvent::Closed { tab, .. } => {
                if let Some(i) = self.find(&tab).filter(|&i| i > 0) {
                    self.remove_tab(i);
                }
            }
            ShellEvent::Log { stamp_ms, level, name, text } => self.add_log(stamp_ms, level, &name, &text),
        }
    }

    fn add_log(&mut self, stamp_ms: u64, level: u8, name: &str, text: &str) {
        if level < self.log_filter {
            return;
        }
        let offset = self.tz_offset.map(Some).unwrap_or_else(|| local_offset(stamp_ms / 1000));
        self.tabs[0].scroll.feed(&format_log(stamp_ms, offset, level, name, text));
        self.touch(0);
    }

    /// A line made on the phone (a RESULT or TASK_DONE), shown in the rosout tab.
    pub fn log_local(&mut self, level: u8, name: &str, text: &str) {
        let now = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64);
        self.add_log(now, level, name, text);
    }

    /// Use a fixed UTC offset (seconds) for log times instead of the local zone.
    pub fn set_tz_offset(&mut self, secs: i64) {
        self.tz_offset = Some(secs);
    }

    /// Record the request id given to the SHELL_COMPLETE just sent for `tab`.
    pub fn note_complete_req(&mut self, tab: &str, req: u32) {
        if let Some(i) = self.find(tab) {
            self.tabs[i].complete_req = Some(req);
        }
    }

    pub fn log_filter(&self) -> u8 {
        self.log_filter
    }

    /// Set the minimum level shown; the robot filters too.
    pub fn set_log_filter(&mut self, level: u8) -> Msg {
        self.log_filter = level;
        Msg::LogLevel { req: 0, min_level: level }
    }

    // ---- output ----

    pub fn rows(&self) -> Vec<Row> {
        self.tabs[self.active].scroll.rows()
    }

    pub fn toggle_fold(&mut self, block: usize) {
        let a = self.active;
        self.tabs[a].scroll.toggle_fold(block);
        self.touch(a);
    }

    // ---- quick commands ----

    pub fn quick_commands(&self) -> &[String] {
        &self.quick
    }

    pub fn set_quick_commands(&mut self, q: Vec<String>) {
        self.quick = q;
        self.dirty = true;
    }

    /// Register the current input line as a quick command.
    pub fn add_quick_from_line(&mut self) {
        let (line, _) = self.input_display();
        let line = line.trim().to_owned();
        if !line.is_empty() && !self.quick.contains(&line) && self.tabs[self.active].shell {
            self.quick.push(line);
            self.dirty = true;
        }
    }

    pub fn remove_quick(&mut self, i: usize) {
        if i < self.quick.len() {
            self.quick.remove(i);
            self.dirty = true;
        }
    }

    // ---- persistence ----

    /// True once after something worth saving changed.
    pub fn take_dirty(&mut self) -> bool {
        std::mem::take(&mut self.dirty)
    }

    pub fn save_to(&self, dir: &Path) {
        let put = |name: &str, text: String| {
            if let Err(e) = storage::save_to(dir, name, &text) {
                eprintln!("could not save {name}: {e}");
            }
        };
        put(GLOBAL_FILE, self.global.to_text());
        put(QUICK_FILE, self.quick.join("\n"));
        for t in self.tabs.iter().filter(|t| t.shell) {
            put(&hist_file(&t.name), t.history.to_text());
        }
        for (n, h) in &self.stash {
            put(&hist_file(n), h.to_text());
        }
    }

    pub fn load_from(&mut self, dir: &Path) {
        self.dir = Some(dir.to_owned());
        if let Some(t) = storage::load_from(dir, GLOBAL_FILE) {
            self.global = History::from_text(&t);
        }
        if let Some(t) = storage::load_from(dir, QUICK_FILE) {
            self.quick = t.lines().filter(|l| !l.trim().is_empty()).map(str::to_owned).collect();
        }
        for i in 1..self.tabs.len() {
            if let Some(t) = storage::load_from(dir, &hist_file(&self.tabs[i].name)) {
                self.tabs[i].history = History::from_text(&t);
            }
        }
    }

    pub fn save(&self) {
        self.save_to(&storage::data_dir());
    }

    pub fn load(&mut self) {
        self.load_from(&storage::data_dir());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn texts(c: &Console) -> Vec<String> {
        c.rows()
            .iter()
            .map(|r| match r {
                Row::Line(l) => l.text(),
                Row::Folded { hidden, .. } => format!("<{hidden}>"),
            })
            .collect()
    }

    fn input(tab: &str, data: &[u8]) -> Msg {
        Msg::ShellInput { req: 0, tab: tab.into(), data: data.to_vec() }
    }

    fn typed(c: &mut Console, s: &str) -> Vec<Msg> {
        s.chars().flat_map(|ch| c.key(&ch.to_string())).collect()
    }

    fn console_with_tab(name: &str) -> Console {
        let mut c = Console::new();
        assert!(c.new_tab(name).is_some());
        c
    }

    #[test]
    fn typing_and_enter_sends_line_with_newline() {
        let mut c = console_with_tab("a");
        assert!(typed(&mut c, "ls").is_empty(), "editing sends nothing");
        assert_eq!(c.input_display(), ("ls".to_string(), 2));
        assert_eq!(c.key("Enter"), vec![input("a", b"ls\n")]);
        assert_eq!(c.input_display().0, "");
        // Enter starts a fold block: a long output after it folds.
        let long: String = (0..40).map(|i| format!("l{i}\r\n")).collect();
        c.on_event(ShellEvent::Output { tab: "a".into(), data: long.into_bytes() });
        assert!(texts(&c).iter().any(|t| t.starts_with('<')), "{:?}", texts(&c));
    }

    #[test]
    fn ctrl_c_sends_etx() {
        let mut c = console_with_tab("a");
        typed(&mut c, "sleep");
        assert!(c.key("Ctrl").is_empty());
        assert_eq!(c.key("c"), vec![input("a", &[0x03])]);
        assert_eq!(c.input_display().0, "", "Ctrl+C clears the line");
        // one-shot: the next c is a plain character
        assert!(c.key("c").is_empty());
        assert_eq!(c.input_display().0, "c");
    }

    #[test]
    fn tab_sends_complete_and_applies_result() {
        let mut c = console_with_tab("a");
        typed(&mut c, "ech");
        assert_eq!(
            c.key("Tab"),
            vec![Msg::ShellComplete { req: 0, tab: "a".into(), line: "ech".into(), cursor: 3 }]
        );
        c.on_event(ShellEvent::Completion { req: 1, tab: "a".into(), start: 0, candidates: vec!["echo".into()] });
        assert_eq!(c.input_display(), ("echo ".to_string(), 5));
        assert!(c.candidates().is_empty());

        // several candidates: common prefix goes in, the list is shown, Esc hides it
        let mut c = console_with_tab("b");
        typed(&mut c, "ec");
        c.key("Tab");
        c.on_event(ShellEvent::Completion {
            req: 1,
            tab: "b".into(),
            start: 0,
            candidates: vec!["echo".into(), "ecryptfs".into()],
        });
        assert_eq!(c.input_display().0, "ec");
        assert_eq!(c.candidates(), ["echo", "ecryptfs"]);
        c.pick_candidate(0);
        assert_eq!(c.input_display().0, "echo ");
        assert!(c.candidates().is_empty());

        typed(&mut c, "x");
        c.key("Tab");
        c.on_event(ShellEvent::Completion { req: 2, tab: "b".into(), start: 5, candidates: vec!["xa".into(), "xb".into()] });
        assert_eq!(c.candidates().len(), 2);
        assert!(c.key("Esc").is_empty());
        assert!(c.candidates().is_empty());
    }

    #[test]
    fn stale_completion_is_ignored() {
        let mut c = console_with_tab("a");
        typed(&mut c, "ech");
        c.key("Tab");
        typed(&mut c, "x"); // the line moved on before the answer came
        c.on_event(ShellEvent::Completion { req: 1, tab: "a".into(), start: 0, candidates: vec!["echo".into()] });
        assert_eq!(c.input_display().0, "echx");
    }

    #[test]
    fn rosout_tab_receives_logs_and_cannot_close() {
        let mut c = Console::new();
        c.set_tz_offset(0);
        assert_eq!(c.tabs(), [ROSOUT]);
        assert_eq!(c.close_tab(0), None);
        assert_eq!(c.tabs(), [ROSOUT]);
        c.on_event(ShellEvent::Log { stamp_ms: 3_723_000, level: 30, name: "nav".into(), text: "slow".into() });
        assert_eq!(texts(&c), ["01:02:03 [WARN] nav: slow"]);
        // typing into rosout sends nothing
        assert!(c.key("a").is_empty());
        assert!(c.key("Enter").is_empty());
        // lines below the filter are dropped
        assert!(matches!(c.set_log_filter(40), Msg::LogLevel { min_level: 40, .. }));
        c.on_event(ShellEvent::Log { stamp_ms: 0, level: 20, name: "n".into(), text: "info".into() });
        assert_eq!(texts(&c).len(), 1);
        c.set_log_filter(20);
        c.log_local(30, "auto", "task done");
        assert!(texts(&c).last().unwrap().ends_with("[WARN] auto: task done"));
    }

    #[test]
    fn opened_replaces_content() {
        let mut c = Console::new();
        c.on_event(ShellEvent::Opened { tab: "a".into(), replay: b"old\r\n".to_vec() });
        assert_eq!(c.tabs(), [ROSOUT, "a"]);
        c.on_event(ShellEvent::Output { tab: "a".into(), data: b"more\r\n".to_vec() });
        c.on_event(ShellEvent::Opened { tab: "a".into(), replay: b"new\r\n".to_vec() });
        assert_eq!(c.tabs(), [ROSOUT, "a"]);
        c.new_tab("b");
        c.on_event(ShellEvent::Opened { tab: "a".into(), replay: b"new\r\n".to_vec() });
        // "a" is not active now; look at it via its own rows
        assert_eq!(c.active(), 2);
        c.set_active(1);
        assert_eq!(texts(&c), ["new"]);
    }

    #[test]
    fn closed_event_removes_tab() {
        let mut c = console_with_tab("a");
        c.new_tab("b");
        assert_eq!(c.active(), 2);
        c.on_event(ShellEvent::Closed { tab: "b".into(), reason: "exit".into() });
        assert_eq!(c.tabs(), [ROSOUT, "a"]);
        assert_eq!(c.active(), 1);
        c.on_event(ShellEvent::Closed { tab: ROSOUT.into(), reason: "x".into() });
        assert_eq!(c.tabs(), [ROSOUT, "a"]);
        assert_eq!(c.close_tab(1), Some(Msg::ShellClose { req: 0, tab: "a".into() }));
        assert_eq!(c.tabs(), [ROSOUT]);
    }

    #[test]
    fn history_persists() {
        let dir = tempfile::tempdir().unwrap();
        let mut c = console_with_tab("a");
        typed(&mut c, "ls");
        c.key("Enter");
        typed(&mut c, "pwd");
        c.key("Enter");
        c.set_quick_commands(vec!["ls -l".into(), "ros2 topic list".into()]);
        c.save_to(dir.path());

        let mut d = Console::new();
        d.load_from(dir.path());
        assert_eq!(d.quick_commands(), ["ls -l", "ros2 topic list"]);
        d.new_tab("a");
        d.key("Up");
        assert_eq!(d.input_display().0, "pwd");
        // another tab has its own history but shares the global one (Ctrl+R)
        d.new_tab("b");
        d.key("Up");
        assert_eq!(d.input_display().0, "");
        d.key("Ctrl");
        d.key("r");
        d.key("l");
        assert!(d.input_display().0.contains("ls"));
    }

    #[test]
    fn new_tab_names_are_validated() {
        let mut c = Console::new();
        for bad in ["", "a b", "日本", "a/b", &"x".repeat(33), ROSOUT] {
            assert_eq!(c.new_tab(bad), None, "{bad:?}");
        }
        assert_eq!(c.new_tab("ok_1-x"), Some(Msg::ShellOpen { req: 0, tab: "ok_1-x".into() }));
        assert_eq!(c.new_tab("ok_1-x"), None, "duplicate");
        assert_eq!(c.rename_tab(1, "new"), Some(Msg::ShellRename { req: 0, tab: "ok_1-x".into(), new_name: "new".into() }));
        assert_eq!(c.tabs(), [ROSOUT, "new"]);
        assert_eq!(c.rename_tab(0, "z"), None);
        assert_eq!(c.rename_tab(1, "bad name"), None);
    }

    #[test]
    fn name_prompt_uses_the_keyboard() {
        let mut c = Console::new();
        c.begin_new_tab();
        assert!(c.prompt().is_some());
        typed(&mut c, "a b");
        assert_eq!(c.prompt().unwrap().text, "ab", "space is not allowed");
        c.key("Backspace");
        c.key("x");
        assert!(c.key("Esc").is_empty());
        assert!(c.prompt().is_none());
        c.begin_new_tab();
        typed(&mut c, "sh1");
        assert_eq!(c.key("Enter"), vec![Msg::ShellOpen { req: 0, tab: "sh1".into() }]);
        assert!(c.prompt().is_none());
        assert_eq!(c.tabs(), [ROSOUT, "sh1"]);
        // a duplicate keeps the prompt open with an error
        c.begin_new_tab();
        typed(&mut c, "sh1");
        assert!(c.key("Enter").is_empty());
        assert!(c.prompt().unwrap().error.is_some());
        // rename
        c.key("Esc");
        c.begin_rename(1);
        assert_eq!(c.prompt().unwrap().text, "sh1");
        c.key("2");
        assert_eq!(
            c.key("Enter"),
            vec![Msg::ShellRename { req: 0, tab: "sh1".into(), new_name: "sh12".into() }]
        );
    }

    #[test]
    fn reconnect_replaces_phone_tabs_with_robot_tabs() {
        let mut c = console_with_tab("gone");
        c.new_tab("kept");
        c.begin_sync();
        c.on_event(ShellEvent::Opened { tab: "kept".into(), replay: b"x\r\n".to_vec() });
        c.on_event(ShellEvent::Opened { tab: "robot".into(), replay: Vec::new() });
        c.new_tab("fresh"); // made by the user after the sync began
        c.end_sync();
        assert_eq!(c.tabs(), [ROSOUT, "kept", "robot", "fresh"]);
    }

    #[test]
    fn palette_maps_16_256_and_truecolor() {
        assert_eq!(palette(Color::Rgb(1, 2, 3)), (1, 2, 3));
        assert_eq!(palette(Color::Indexed(1)), (205, 49, 49));
        assert_eq!(palette(Color::Indexed(16)), (0, 0, 0));
        assert_eq!(palette(Color::Indexed(21)), (0, 0, 255));
        assert_eq!(palette(Color::Indexed(231)), (255, 255, 255));
        assert_eq!(palette(Color::Indexed(232)), (8, 8, 8));
        assert_eq!(palette(Color::Indexed(255)), (238, 238, 238));
    }

    #[test]
    fn long_rows_wrap_keeping_styles() {
        let red = Some(Color::Indexed(1));
        let row = Row::Line(Line {
            spans: vec![
                Span { text: "abcdef".into(), fg: None, bg: None, bold: false },
                Span { text: "ghij日本".into(), fg: red, bg: None, bold: true },
            ],
        });
        let w = wrap_row(&row, 8);
        let texts: Vec<String> = w.iter().map(|r| if let Row::Line(l) = r { l.text() } else { String::new() }).collect();
        assert_eq!(texts, ["abcdefgh", "ij日本"]);
        if let Row::Line(l) = &w[1] {
            assert_eq!(l.spans.len(), 1);
            assert_eq!(l.spans[0].fg, red);
        }
        assert_eq!(wrap_row(&Row::Folded { block: 1, hidden: 5 }, 8), [Row::Folded { block: 1, hidden: 5 }]);
        let empty = Row::Line(Line { spans: vec![] });
        assert_eq!(wrap_row(&empty, 8), [empty]);
    }

    #[test]
    fn ctrl_c_works_in_search_and_with_candidates_shown() {
        let mut c = console_with_tab("a");
        typed(&mut c, "ls");
        c.key("Enter");
        c.key("Ctrl");
        c.key("r");
        assert!(c.input_display().0.starts_with("(reverse-i-search)"));
        c.key("Ctrl");
        assert_eq!(c.key("c"), vec![input("a", &[0x03])]);
        assert!(!c.input_display().0.starts_with("(reverse"));

        typed(&mut c, "ec");
        c.key("Tab");
        c.on_event(ShellEvent::Completion {
            req: 1,
            tab: "a".into(),
            start: 0,
            candidates: vec!["echo".into(), "ecryptfs".into()],
        });
        assert_eq!(c.candidates().len(), 2);
        c.key("Ctrl");
        assert_eq!(c.key("c"), vec![input("a", &[0x03])]);
        assert!(c.candidates().is_empty());
    }

    #[test]
    fn completion_is_matched_by_req() {
        let mut c = console_with_tab("a");
        typed(&mut c, "ech");
        c.key("Tab");
        c.note_complete_req("a", 7);
        c.on_event(ShellEvent::Completion { req: 6, tab: "a".into(), start: 0, candidates: vec!["old".into()] });
        assert_eq!(c.input_display().0, "ech");
        c.on_event(ShellEvent::Completion { req: 7, tab: "a".into(), start: 0, candidates: vec!["echo".into()] });
        assert_eq!(c.input_display().0, "echo ");
    }

    #[test]
    fn log_time_uses_offset_or_marks_utc() {
        let b = |o| String::from_utf8(format_log(3_723_000, o, 20, "n", "t")).unwrap();
        assert_eq!(b(Some(32_400)), "10:02:03 [INFO] n: t\r\n");
        assert_eq!(b(None), "01:02:03Z [INFO] n: t\r\n");
        assert_eq!(b(Some(-7200)), "23:02:03 [INFO] n: t\r\n");
    }
}
