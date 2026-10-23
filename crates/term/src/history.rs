//! Command history (oldest first).

pub const DEFAULT_MAX: usize = 1000;

#[derive(Debug, Clone)]
pub struct History {
    max: usize,
    entries: Vec<String>,
}

impl History {
    pub fn new(max: usize) -> Self {
        Self { max, entries: Vec::new() }
    }

    /// Blank lines and a repeat of the newest entry are ignored.
    pub fn push(&mut self, line: &str) {
        if line.trim().is_empty() || self.entries.last().is_some_and(|l| l == line) {
            return;
        }
        self.entries.push(line.to_string());
        if self.entries.len() > self.max {
            let over = self.entries.len() - self.max;
            self.entries.drain(..over);
        }
    }

    pub fn entries(&self) -> &[String] {
        &self.entries
    }

    /// Newest entry containing `query` among indices `< before` (`None` = all).
    pub fn search(&self, query: &str, before: Option<usize>) -> Option<usize> {
        let end = before.unwrap_or(self.entries.len()).min(self.entries.len());
        (0..end).rev().find(|&i| self.entries[i].contains(query))
    }

    pub fn to_text(&self) -> String {
        self.entries.join("\n")
    }

    pub fn from_text(text: &str) -> Self {
        let mut h = Self::new(DEFAULT_MAX);
        for l in text.lines() {
            h.push(l);
        }
        h
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn history_skips_blank_and_repeat() {
        let mut h = History::new(3);
        h.push("");
        h.push("   ");
        h.push("ls");
        h.push("ls");
        h.push("pwd");
        h.push("ls");
        assert_eq!(h.entries(), ["ls", "pwd", "ls"]);
        h.push("cd");
        assert_eq!(h.entries(), ["pwd", "ls", "cd"]);
        assert_eq!(h.search("l", None), Some(1));
        assert_eq!(h.search("l", Some(1)), None);
        assert_eq!(h.search("zzz", None), None);
    }

    #[test]
    fn history_text_roundtrip() {
        let mut h = History::new(10);
        h.push("ls -l");
        h.push("echo 日本語");
        let t = h.to_text();
        assert_eq!(t, "ls -l\necho 日本語");
        assert_eq!(History::from_text(&t).entries(), h.entries());
        assert!(History::from_text("").entries().is_empty());
    }
}
