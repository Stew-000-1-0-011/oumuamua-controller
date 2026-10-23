//! The demo shell: a few canned commands behind a `demo$ ` prompt, per tab.

use proto::Msg;
use std::collections::BTreeMap;
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROMPT: &str = "demo$ ";
pub const MAX_CHUNK: usize = 900;
const REPLAY_MAX: usize = 64 * 1024;
const COMMANDS: [&str; 6] = ["date", "echo", "help", "ls", "pwd", "ros2"];
const TOPICS: [&str; 9] = [
    "/cmd_vel", "/initialpose", "/scan", "/tf", "/wheel0/status", "/wheel1/status", "/wheel2/status",
    "/gripper0/state", "/gripper1/state",
];
const NODES: [&str; 5] = ["/bridge", "/path_follower", "/task_commander", "/tracker_node", "/lidar_driver"];

#[derive(Default)]
struct Tab {
    replay: String,
    line: String,
}

#[derive(Default)]
pub struct Shell {
    tabs: BTreeMap<String, Tab>,
}

fn valid_name(name: &str) -> bool {
    (1..=32).contains(&name.len()) && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

/// Pieces of at most `max` bytes, cut on character boundaries.
fn chunks(s: &str, max: usize) -> Vec<&str> {
    let mut out = Vec::new();
    let mut rest = s;
    while !rest.is_empty() {
        let mut n = rest.len().min(max);
        while !rest.is_char_boundary(n) {
            n -= 1;
        }
        let (a, b) = rest.split_at(n);
        out.push(a);
        rest = b;
    }
    out
}

/// SHELL_OUTPUT messages for `text`, none over 900 bytes.
pub fn output_msgs(tab: &str, text: &str) -> Vec<Msg> {
    chunks(text, MAX_CHUNK)
        .into_iter()
        .map(|c| Msg::ShellOutput { tab: tab.to_owned(), data: c.as_bytes().to_vec() })
        .collect()
}

/// SHELL_OPENED with the first 900 bytes of the replay, then SHELL_OUTPUT for the rest.
pub fn opened_msgs(tab: &str, replay: &str) -> Vec<Msg> {
    let parts = chunks(replay, MAX_CHUNK);
    let first = parts.first().copied().unwrap_or("");
    let mut out = vec![Msg::ShellOpened { tab: tab.to_owned(), replay: first.as_bytes().to_vec() }];
    out.extend(output_msgs(tab, &replay[first.len()..]));
    out
}

impl Shell {
    pub fn tabs(&self) -> Vec<String> {
        self.tabs.keys().cloned().collect()
    }

    pub fn replay(&self, tab: &str) -> Option<&str> {
        self.tabs.get(tab).map(|t| t.replay.as_str())
    }

    pub fn open(&mut self, tab: &str) -> Result<(), String> {
        if !valid_name(tab) {
            return Err("bad tab name".into());
        }
        if self.tabs.contains_key(tab) {
            return Err("tab already exists".into());
        }
        let banner = format!("oumuamua デモシェル (help でコマンド一覧)\r\n{PROMPT}");
        self.tabs.insert(tab.to_owned(), Tab { replay: banner, line: String::new() });
        Ok(())
    }

    pub fn close(&mut self, tab: &str) -> Result<(), String> {
        self.tabs.remove(tab).map(|_| ()).ok_or_else(|| "no such tab".into())
    }

    pub fn rename(&mut self, tab: &str, new_name: &str) -> Result<(), String> {
        if !valid_name(new_name) {
            return Err("bad tab name".into());
        }
        if self.tabs.contains_key(new_name) {
            return Err("tab already exists".into());
        }
        let t = self.tabs.remove(tab).ok_or("no such tab")?;
        self.tabs.insert(new_name.to_owned(), t);
        Ok(())
    }

    /// Feed typed bytes (a line plus newline, or one control byte); returns the terminal output
    /// (echo, command output, prompt), which is also appended to the tab's replay.
    pub fn input(&mut self, tab: &str, data: &[u8]) -> Result<String, String> {
        let t = self.tabs.get_mut(tab).ok_or("no such tab")?;
        let mut out = String::new();
        let mut prev = '\0';
        for ch in String::from_utf8_lossy(data).chars() {
            match ch {
                '\x03' => {
                    t.line.clear();
                    out += &format!("^C\r\n{PROMPT}");
                }
                '\r' | '\n' if !(ch == '\n' && prev == '\r') => {
                    out += "\r\n";
                    out += &run(&std::mem::take(&mut t.line));
                    out += PROMPT;
                }
                '\x7f' | '\x08' => {
                    if t.line.pop().is_some() {
                        out += "\x08 \x08";
                    }
                }
                c if !c.is_control() => {
                    t.line.push(c);
                    out.push(c);
                }
                _ => {}
            }
            prev = ch;
        }
        t.replay += &out;
        if t.replay.len() > REPLAY_MAX {
            let mut cut = t.replay.len() - REPLAY_MAX;
            while !t.replay.is_char_boundary(cut) {
                cut += 1;
            }
            t.replay.drain(..cut);
        }
        Ok(out)
    }

    pub fn has_tab(&self, tab: &str) -> bool {
        self.tabs.contains_key(tab)
    }
}

/// Candidates replacing `line[start..cursor]` (character positions); command names for the first
/// word, `topic`/`node` after `ros2`.
pub fn complete(line: &str, cursor: usize) -> (u32, Vec<String>) {
    let before: String = line.chars().take(cursor).collect();
    let start = before.rfind(char::is_whitespace).map_or(0, |i| i + 1);
    let word = &before[start..];
    let earlier: Vec<&str> = before[..start].split_whitespace().collect();
    let pool: &[&str] = match earlier.as_slice() {
        [] => &COMMANDS,
        ["ros2"] => &["node", "topic"],
        _ => &[],
    };
    let start_chars = before[..start].chars().count() as u32;
    (start_chars, pool.iter().filter(|c| c.starts_with(word)).map(|c| (*c).to_owned()).collect())
}

fn lines(items: &[&str]) -> String {
    items.iter().map(|s| format!("{s}\r\n")).collect()
}

fn run(line: &str) -> String {
    let words: Vec<&str> = line.split_whitespace().collect();
    match words.as_slice() {
        [] => String::new(),
        ["help", ..] => lines(&["help  echo <文字>  ls  pwd  date  ros2 topic list  ros2 node list"]),
        ["echo", rest @ ..] => lines(&[&rest.join(" ")]),
        ["ls", ..] => lines(&["bridge.yaml  logs  paths"]),
        ["pwd", ..] => lines(&["/home/demo"]),
        ["date", ..] => lines(&[&utc_now()]),
        ["ros2", "topic", "list"] => lines(&TOPICS),
        ["ros2", "node", "list"] => lines(&NODES),
        ["ros2", ..] => lines(&["demo: ros2: 使えるのは topic list と node list だけです"]),
        [name, ..] => format!("demo: {name}: コマンドがありません\r\n"),
    }
}

fn utc_now() -> String {
    let secs = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs()) as i64;
    let (days, rem) = (secs.div_euclid(86400), secs.rem_euclid(86400));
    // Civil date from days since 1970-01-01 (Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y}-{m:02}-{d:02} {:02}:{:02}:{:02} UTC", rem / 3600, rem % 3600 / 60, rem % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn echo_unknown_and_ctrl_c() {
        let mut s = Shell::default();
        s.open("a").unwrap();
        assert_eq!(s.input("a", b"echo hi there\n").unwrap(), "echo hi there\r\nhi there\r\ndemo$ ");
        assert_eq!(s.input("a", b"foo\n").unwrap(), "foo\r\ndemo: foo: コマンドがありません\r\ndemo$ ");
        assert_eq!(s.input("a", b"ab\x03").unwrap(), "ab^C\r\ndemo$ ");
        assert!(s.replay("a").unwrap().ends_with("^C\r\ndemo$ "));
    }

    #[test]
    fn completion_and_chunking() {
        assert_eq!(complete("ec", 2), (0, vec!["echo".to_string()]));
        assert_eq!(complete("ros2 t", 6), (5, vec!["topic".to_string()]));
        assert_eq!(complete("echo x", 6).1, Vec::<String>::new());
        let big = "あ".repeat(1000);
        assert!(output_msgs("t", &big).iter().all(|m| matches!(m, Msg::ShellOutput { data, .. } if data.len() <= 900)));
        let m = opened_msgs("t", &big);
        assert!(matches!(&m[0], Msg::ShellOpened { replay, .. } if replay.len() == 900));
        assert_eq!(m.len(), 4);
    }

    #[test]
    fn date_is_plausible() {
        assert!(utc_now().starts_with("20"));
    }
}
