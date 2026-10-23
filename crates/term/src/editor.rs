//! Local line editor: edits happen on the phone, only Enter/Tab/control keys are sent.
use crate::history::History;
use crate::keys::{Key, KeyEvent};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EditAction {
    None,
    Submit(String),
    Complete { line: String, cursor: usize },
    SendNow(Vec<u8>),
    ClearScreen,
}

#[derive(Debug, Clone)]
struct Search {
    query: String,
    matched: Option<usize>,
    /// Text of the matched entry (empty when nothing matches).
    text: String,
    /// Char index of the query inside `text`.
    pos: usize,
}

impl Search {
    fn set_match(&mut self, global: &History, i: Option<usize>) {
        self.matched = i;
        self.text = i.map(|i| global.entries()[i].clone()).unwrap_or_default();
        let byte = self.text.find(&self.query).unwrap_or(0);
        self.pos = self.text[..byte].chars().count();
    }
}

#[derive(Debug, Default)]
pub struct LineEditor {
    line: Vec<char>,
    cursor: usize,
    /// Index into the tab history while walking with Up/Down.
    hist_pos: Option<usize>,
    /// The in-progress line, restored when Down walks past the newest entry.
    saved: Vec<char>,
    search: Option<Search>,
}

impl LineEditor {
    pub fn new() -> Self {
        Self::default()
    }

    fn set_line(&mut self, s: &str) {
        self.line = s.chars().collect();
        self.cursor = self.line.len();
    }

    fn clear(&mut self) {
        self.line.clear();
        self.cursor = 0;
        self.hist_pos = None;
        self.saved.clear();
    }

    pub fn handle(
        &mut self,
        ev: KeyEvent,
        tab_history: &mut History,
        global_history: &mut History,
    ) -> EditAction {
        if self.search.is_some() {
            return self.handle_search(ev, tab_history, global_history);
        }
        self.handle_edit(ev, tab_history, global_history)
    }

    fn submit(&mut self, tab: &mut History, global: &mut History) -> EditAction {
        let s: String = self.line.iter().collect();
        tab.push(&s);
        global.push(&s);
        self.clear();
        EditAction::Submit(s)
    }

    fn handle_edit(&mut self, ev: KeyEvent, tab: &mut History, global: &mut History) -> EditAction {
        match ev {
            KeyEvent::Key(Key::Char(c)) => {
                self.line.insert(self.cursor, c);
                self.cursor += 1;
            }
            KeyEvent::Key(Key::Enter) => return self.submit(tab, global),
            KeyEvent::Key(Key::Tab) => {
                return EditAction::Complete {
                    line: self.line.iter().collect(),
                    cursor: self.cursor,
                };
            }
            KeyEvent::Key(Key::Backspace) => {
                if self.cursor > 0 {
                    self.cursor -= 1;
                    self.line.remove(self.cursor);
                }
            }
            KeyEvent::Key(Key::Delete) => {
                if self.cursor < self.line.len() {
                    self.line.remove(self.cursor);
                }
            }
            KeyEvent::Key(Key::Left) => self.cursor = self.cursor.saturating_sub(1),
            KeyEvent::Key(Key::Right) => self.cursor = (self.cursor + 1).min(self.line.len()),
            KeyEvent::Key(Key::Home) => self.cursor = 0,
            KeyEvent::Key(Key::End) => self.cursor = self.line.len(),
            KeyEvent::Key(Key::Esc) => {}
            KeyEvent::Key(Key::Up) => {
                let entries = tab.entries();
                let pos = match self.hist_pos {
                    None if entries.is_empty() => return EditAction::None,
                    None => {
                        self.saved = self.line.clone();
                        entries.len() - 1
                    }
                    Some(i) => i.saturating_sub(1),
                };
                self.hist_pos = Some(pos);
                let s = entries[pos].clone();
                self.set_line(&s);
            }
            KeyEvent::Key(Key::Down) => {
                if let Some(i) = self.hist_pos {
                    let entries = tab.entries();
                    if i + 1 < entries.len() {
                        self.hist_pos = Some(i + 1);
                        let s = entries[i + 1].clone();
                        self.set_line(&s);
                    } else {
                        self.hist_pos = None;
                        self.line = std::mem::take(&mut self.saved);
                        self.cursor = self.line.len();
                    }
                }
            }
            KeyEvent::Ctrl('a') => self.cursor = 0,
            KeyEvent::Ctrl('e') => self.cursor = self.line.len(),
            KeyEvent::Ctrl('u') => {
                self.line.drain(..self.cursor);
                self.cursor = 0;
            }
            KeyEvent::Ctrl('w') => {
                let mut i = self.cursor;
                while i > 0 && self.line[i - 1] == ' ' {
                    i -= 1;
                }
                while i > 0 && self.line[i - 1] != ' ' {
                    i -= 1;
                }
                self.line.drain(i..self.cursor);
                self.cursor = i;
            }
            KeyEvent::Ctrl('l') => return EditAction::ClearScreen,
            KeyEvent::Ctrl('r') => {
                self.search = Some(Search { query: String::new(), matched: None, text: String::new(), pos: 0 });
            }
            KeyEvent::Ctrl('c') => {
                self.clear();
                return EditAction::SendNow(vec![0x03]);
            }
            KeyEvent::Ctrl(c) => {
                return EditAction::SendNow(vec![(c as u32 & 0x1f) as u8]);
            }
            KeyEvent::Alt(c) => {
                let mut out = vec![0x1b];
                let mut buf = [0u8; 4];
                out.extend_from_slice(c.encode_utf8(&mut buf).as_bytes());
                return EditAction::SendNow(out);
            }
        }
        EditAction::None
    }

    fn handle_search(
        &mut self,
        ev: KeyEvent,
        tab: &mut History,
        global: &mut History,
    ) -> EditAction {
        let Some(s) = self.search.as_mut() else {
            return EditAction::None;
        };
        match ev {
            KeyEvent::Key(Key::Char(c)) => {
                s.query.push(c);
                // keep the current match if it still matches, else go older
                let before = s.matched.map(|m| m + 1);
                let found = global.search(&s.query, before);
                if found.is_some() {
                    s.set_match(global, found);
                } else {
                    s.set_match(global, s.matched);
                }
            }
            KeyEvent::Ctrl('r') => {
                let found = global.search(&s.query, s.matched);
                if found.is_some() {
                    s.set_match(global, found);
                }
            }
            KeyEvent::Key(Key::Backspace) => {
                s.query.pop();
                let found = if s.query.is_empty() { None } else { global.search(&s.query, None) };
                s.set_match(global, found);
            }
            KeyEvent::Key(Key::Esc) => self.search = None,
            KeyEvent::Key(Key::Enter) => {
                if let Some(i) = s.matched {
                    let l = global.entries()[i].clone();
                    self.search = None;
                    self.set_line(&l);
                    return self.submit(tab, global);
                }
                self.search = None;
            }
            // anything else: leave search keeping the matched line, then act normally
            other => {
                if let Some(i) = s.matched {
                    let l = global.entries()[i].clone();
                    self.set_line(&l);
                }
                self.search = None;
                return self.handle_edit(other, tab, global);
            }
        }
        EditAction::None
    }

    /// Ctrl+R search is active (Esc then leaves the search instead of being local only).
    pub fn in_search(&self) -> bool {
        self.search.is_some()
    }

    /// Text to draw and the cursor position as a char index.
    pub fn display(&self) -> (String, usize) {
        if let Some(s) = &self.search {
            let prefix = format!("(reverse-i-search)'{}': ", s.query);
            let n = prefix.chars().count() + s.pos;
            return (format!("{prefix}{}", s.text), n);
        }
        (self.line.iter().collect(), self.cursor)
    }

    /// Replace `[start..cursor]` with the completion; returns candidates to show (if several).
    pub fn apply_completion(&mut self, start: usize, candidates: &[String]) -> Vec<String> {
        let start = start.min(self.cursor);
        let replacement: String = match candidates {
            [] => return Vec::new(),
            [one] => {
                if one.ends_with('/') {
                    one.clone()
                } else {
                    format!("{one} ")
                }
            }
            many => common_prefix(many),
        };
        let rep: Vec<char> = replacement.chars().collect();
        let n = rep.len();
        self.line.splice(start..self.cursor, rep);
        self.cursor = start + n;
        if candidates.len() == 1 { Vec::new() } else { candidates.to_vec() }
    }
}

fn common_prefix(items: &[String]) -> String {
    let mut p: Vec<char> = items[0].chars().collect();
    for s in &items[1..] {
        let n = p.iter().zip(s.chars()).take_while(|(a, b)| **a == *b).count();
        p.truncate(n);
    }
    p.into_iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::keys::Key::*;

    struct Env {
        e: LineEditor,
        t: History,
        g: History,
    }
    impl Env {
        fn new() -> Self {
            Self { e: LineEditor::new(), t: History::new(100), g: History::new(100) }
        }
        fn k(&mut self, k: Key) -> EditAction {
            self.e.handle(KeyEvent::Key(k), &mut self.t, &mut self.g)
        }
        fn ctrl(&mut self, c: char) -> EditAction {
            self.e.handle(KeyEvent::Ctrl(c), &mut self.t, &mut self.g)
        }
        fn type_str(&mut self, s: &str) {
            for c in s.chars() {
                self.k(Char(c));
            }
        }
        fn line(&self) -> String {
            self.e.display().0
        }
        fn cur(&self) -> usize {
            self.e.display().1
        }
    }

    #[test]
    fn insert_move_delete_utf8() {
        let mut v = Env::new();
        v.type_str("echo 日本語");
        assert_eq!((v.line().as_str(), v.cur()), ("echo 日本語", 8));
        v.k(Left);
        v.k(Left);
        v.k(Char('x'));
        assert_eq!(v.line(), "echo 日x本語");
        assert_eq!(v.cur(), 7);
        v.k(Backspace);
        assert_eq!(v.line(), "echo 日本語");
        v.k(Delete);
        assert_eq!(v.line(), "echo 日語");
        v.k(Home);
        v.k(Delete);
        assert_eq!(v.line(), "cho 日語");
        v.k(Backspace);
        assert_eq!(v.cur(), 0);
        v.k(End);
        assert_eq!(v.cur(), 6);
        v.k(Right);
        assert_eq!(v.cur(), 6);
    }

    #[test]
    fn ctrl_a_e_w_u() {
        let mut v = Env::new();
        v.type_str("git commit 日本 語");
        v.ctrl('a');
        assert_eq!(v.cur(), 0);
        v.ctrl('e');
        assert_eq!(v.cur(), 15);
        v.ctrl('w');
        assert_eq!(v.line(), "git commit 日本 ");
        v.ctrl('w');
        assert_eq!(v.line(), "git commit ");
        v.k(Left);
        v.k(Left);
        v.ctrl('u');
        assert_eq!(v.line(), "t ");
        assert_eq!(v.cur(), 0);
        assert_eq!(v.ctrl('l'), EditAction::ClearScreen);
        assert_eq!(v.line(), "t ");
    }

    #[test]
    fn enter_submits_and_records() {
        let mut v = Env::new();
        v.type_str("ls -l");
        assert_eq!(v.k(Enter), EditAction::Submit("ls -l".into()));
        assert_eq!(v.line(), "");
        assert_eq!(v.t.entries(), ["ls -l"]);
        assert_eq!(v.g.entries(), ["ls -l"]);
        assert_eq!(v.k(Enter), EditAction::Submit(String::new()));
        assert_eq!(v.t.entries().len(), 1);
    }

    #[test]
    fn up_down_walk_tab_history() {
        let mut v = Env::new();
        v.t.push("one");
        v.t.push("two");
        v.g.push("other");
        v.type_str("ed");
        v.k(Up);
        assert_eq!(v.line(), "two");
        assert_eq!(v.cur(), 3);
        v.k(Up);
        assert_eq!(v.line(), "one");
        v.k(Up);
        assert_eq!(v.line(), "one");
        v.k(Down);
        assert_eq!(v.line(), "two");
        v.k(Down);
        assert_eq!(v.line(), "ed");
        v.k(Down);
        assert_eq!(v.line(), "ed");
    }

    #[test]
    fn ctrl_r_searches_global() {
        let mut v = Env::new();
        v.g.push("make build");
        v.g.push("ls");
        v.g.push("make test");
        v.type_str("zz");
        v.ctrl('r');
        assert_eq!(v.line(), "(reverse-i-search)'': ");
        v.type_str("mak");
        assert!(v.line().starts_with("(reverse-i-search)'mak': make test"));
        v.ctrl('r');
        assert!(v.line().ends_with(": make build"));
        v.k(Backspace);
        assert!(v.line().starts_with("(reverse-i-search)'ma': make test"));
        v.k(Esc);
        assert_eq!(v.line(), "zz");
        assert_eq!(v.cur(), 2);
        v.ctrl('r');
        v.type_str("ls");
        assert_eq!(v.k(Enter), EditAction::Submit("ls".into()));
        assert_eq!(v.line(), "");
        assert_eq!(v.t.entries(), ["ls"]);
    }

    #[test]
    fn ctrl_c_sends_etx_and_clears() {
        let mut v = Env::new();
        v.type_str("abc");
        assert_eq!(v.ctrl('c'), EditAction::SendNow(vec![0x03]));
        assert_eq!(v.line(), "");
        assert_eq!(v.ctrl('d'), EditAction::SendNow(vec![0x04]));
    }

    #[test]
    fn other_ctrl_sends_control_byte() {
        let mut v = Env::new();
        v.type_str("keep");
        assert_eq!(v.ctrl('z'), EditAction::SendNow(vec![0x1a]));
        assert_eq!(v.ctrl('\\'), EditAction::SendNow(vec![0x1c]));
        assert_eq!(v.line(), "keep");
        assert_eq!(
            v.e.handle(KeyEvent::Alt('b'), &mut v.t, &mut v.g),
            EditAction::SendNow(vec![0x1b, b'b'])
        );
    }

    #[test]
    fn tab_requests_completion() {
        let mut v = Env::new();
        v.type_str("cd 日");
        v.k(Left);
        assert_eq!(v.k(Tab), EditAction::Complete { line: "cd 日".into(), cursor: 3 });
    }

    #[test]
    fn completion_single_and_common_prefix() {
        let mut v = Env::new();
        v.type_str("cat fo");
        let r = v.e.apply_completion(4, &["foo.txt".to_string()]);
        assert!(r.is_empty());
        assert_eq!(v.line(), "cat foo.txt ");
        assert_eq!(v.cur(), 12);

        let mut v = Env::new();
        v.type_str("cd sr");
        v.e.apply_completion(3, &["src/".to_string()]);
        assert_eq!(v.line(), "cd src/");

        let mut v = Env::new();
        v.type_str("ls f");
        let c = vec!["foobar".to_string(), "foobaz".to_string()];
        assert_eq!(v.e.apply_completion(3, &c), c);
        assert_eq!(v.line(), "ls fooba");
        assert_eq!(v.cur(), 8);

        assert!(v.e.apply_completion(3, &[]).is_empty());
        assert_eq!(v.line(), "ls fooba");
    }
}
