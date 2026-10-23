//! Scrollback buffer with overwrite semantics and per-command folding.

use std::collections::VecDeque;

use crate::ansi::{Cell, Event, Line, Parser, Style, cells_to_line};

/// Blocks longer than this many lines are folded.
const FOLD_OVER: usize = 30;
/// Lines shown from the end of a folded block.
const FOLD_KEEP: usize = 10;
/// Longest line in cells; further output wraps onto a new line.
const MAX_COLS: usize = 4096;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Row {
    Line(Line),
    Folded { block: usize, hidden: usize },
}

#[derive(Debug)]
struct Block {
    id: usize,
    lines: VecDeque<Line>,
    unfolded: bool,
}

#[derive(Debug)]
pub struct Scrollback {
    max_lines: usize,
    parser: Parser,
    blocks: VecDeque<Block>,
    next_id: usize,
    /// Number of finished lines across all blocks.
    total: usize,
    cur: Vec<Cell>,
    col: usize,
    events: Vec<Event>,
}

impl Scrollback {
    pub fn new(max_lines: usize) -> Self {
        let mut s = Scrollback {
            max_lines: max_lines.max(1),
            parser: Parser::new(),
            blocks: VecDeque::new(),
            next_id: 0,
            total: 0,
            cur: Vec::new(),
            col: 0,
            events: Vec::new(),
        };
        s.reset();
        s
    }

    fn reset(&mut self) {
        self.parser = Parser::new();
        self.blocks.clear();
        self.total = 0;
        self.cur.clear();
        self.col = 0;
        self.new_block();
    }

    fn new_block(&mut self) {
        let id = self.next_id;
        self.next_id += 1;
        self.blocks.push_back(Block {
            id,
            lines: VecDeque::new(),
            unfolded: false,
        });
    }

    pub fn clear(&mut self) {
        self.reset();
    }

    /// Discard everything and start over from `bytes`.
    pub fn replace(&mut self, bytes: &[u8]) {
        self.reset();
        self.feed(bytes);
    }

    /// Start a new fold block (call when a command is sent with Enter). The
    /// line currently being written (prompt and echoed command) belongs to
    /// the new block.
    pub fn mark_command(&mut self) {
        let last_empty = self.blocks.back().is_none_or(|b| b.lines.is_empty());
        if !last_empty {
            self.new_block();
        }
    }

    pub fn feed(&mut self, bytes: &[u8]) {
        let mut events = std::mem::take(&mut self.events);
        events.clear();
        self.parser.feed(bytes, &mut events);
        for &e in &events {
            self.apply(e);
        }
        if events.capacity() > 64 * 1024 {
            events.clear();
            events.shrink_to(4096);
        }
        self.events = events;
        self.trim();
    }

    fn apply(&mut self, e: Event) {
        match e {
            Event::Print(ch, style) => self.put(ch, style),
            Event::Newline => self.newline(),
            Event::CarriageReturn => self.col = 0,
            Event::Backspace => self.col = self.col.saturating_sub(1),
            Event::Tab => {
                let n = 8 - self.col % 8;
                for _ in 0..n {
                    self.put(' ', Style::default());
                }
            }
            Event::EraseLine(0) => self.cur.truncate(self.col),
            Event::EraseLine(1) => {
                let end = self.col.min(self.cur.len());
                for c in &mut self.cur[..end] {
                    c.ch = ' ';
                    c.style = Style::default();
                }
            }
            Event::EraseLine(_) => self.cur.clear(),
            Event::CursorRight(n) => self.col = (self.col + n).min(MAX_COLS),
            Event::CursorLeft(n) => self.col = self.col.saturating_sub(n),
        }
    }

    fn put(&mut self, ch: char, style: Style) {
        if self.col >= MAX_COLS {
            self.newline();
        }
        while self.cur.len() < self.col {
            self.cur.push(Cell {
                ch: ' ',
                style: Style::default(),
            });
        }
        let cell = Cell { ch, style };
        if self.col < self.cur.len() {
            self.cur[self.col] = cell;
        } else {
            self.cur.push(cell);
        }
        self.col += 1;
    }

    fn newline(&mut self) {
        let line = cells_to_line(&self.cur);
        self.cur.clear();
        self.col = 0;
        self.blocks.back_mut().unwrap().lines.push_back(line);
        self.total += 1;
    }

    fn trim(&mut self) {
        while self.total > self.max_lines {
            let front = self.blocks.front_mut().unwrap();
            if front.lines.pop_front().is_some() {
                self.total -= 1;
            }
            if front.lines.is_empty() && self.blocks.len() > 1 {
                self.blocks.pop_front();
            }
        }
    }

    pub fn rows(&self) -> Vec<Row> {
        let mut rows = Vec::new();
        let last = self.blocks.len() - 1;
        for (i, b) in self.blocks.iter().enumerate() {
            let partial = (i == last && !self.cur.is_empty()).then(|| cells_to_line(&self.cur));
            let len = b.lines.len() + usize::from(partial.is_some());
            let skip = if len > FOLD_OVER && !b.unfolded {
                rows.push(Row::Folded {
                    block: b.id,
                    hidden: len - FOLD_KEEP,
                });
                len - FOLD_KEEP
            } else {
                0
            };
            rows.extend(
                b.lines
                    .iter()
                    .cloned()
                    .chain(partial)
                    .skip(skip)
                    .map(Row::Line),
            );
        }
        rows
    }

    /// Flip the fold state of the block with the given stable id.
    pub fn toggle_fold(&mut self, block: usize) {
        if let Some(b) = self.blocks.iter_mut().find(|b| b.id == block) {
            b.unfolded = !b.unfolded;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ansi::{Color, Line};

    fn texts(s: &Scrollback) -> Vec<String> {
        s.rows()
            .into_iter()
            .map(|r| match r {
                Row::Line(l) => l.text(),
                Row::Folded { block, hidden } => format!("<folded {block} {hidden}>"),
            })
            .collect()
    }

    fn line(s: &Scrollback, i: usize) -> Line {
        match &s.rows()[i] {
            Row::Line(l) => l.clone(),
            r => panic!("not a line: {r:?}"),
        }
    }

    #[test]
    fn plain_lines() {
        let mut s = Scrollback::new(100);
        s.feed(b"hello\r\nworld\r\npart");
        assert_eq!(texts(&s), ["hello", "world", "part"]);
        s.feed(b"ial\n");
        assert_eq!(texts(&s), ["hello", "world", "partial"]);
    }

    #[test]
    fn cr_overwrites() {
        let mut s = Scrollback::new(100);
        s.feed(b"abc\rX");
        assert_eq!(texts(&s), ["Xbc"]);
    }

    #[test]
    fn backspace() {
        let mut s = Scrollback::new(100);
        s.feed(b"abc\x08\x08X\x08\x08\x08\x08Y");
        assert_eq!(texts(&s), ["YXc"]);
    }

    #[test]
    fn tab_and_cursor_moves() {
        let mut s = Scrollback::new(100);
        s.feed(b"a\tb\n");
        assert_eq!(texts(&s), ["a       b"]);
        let mut s = Scrollback::new(100);
        s.feed(b"abcd\x1b[2D\x1b[CX\x1b[K");
        assert_eq!(texts(&s), ["abcX"]);
    }

    #[test]
    fn sgr_16_256_truecolor() {
        let mut s = Scrollback::new(100);
        s.feed(b"\x1b[31ma\x1b[92mb\x1b[38;5;200mc\x1b[48;2;1;2;3md\x1b[41;39me\n");
        let l = line(&s, 0);
        let got: Vec<_> = l
            .spans
            .iter()
            .map(|x| (x.text.as_str(), x.fg, x.bg))
            .collect();
        assert_eq!(
            got,
            vec![
                ("a", Some(Color::Indexed(1)), None),
                ("b", Some(Color::Indexed(10)), None),
                ("c", Some(Color::Indexed(200)), None),
                ("d", Some(Color::Indexed(200)), Some(Color::Rgb(1, 2, 3))),
                ("e", None, Some(Color::Indexed(1))),
            ]
        );
    }

    #[test]
    fn reset_and_bold() {
        let mut s = Scrollback::new(100);
        s.feed(b"\x1b[1;34mA\x1b[22mB\x1b[0mC\n");
        let l = line(&s, 0);
        assert_eq!(l.spans.len(), 3);
        assert!(l.spans[0].bold && l.spans[0].fg == Some(Color::Indexed(4)));
        assert!(!l.spans[1].bold && l.spans[1].fg == Some(Color::Indexed(4)));
        assert!(!l.spans[2].bold && l.spans[2].fg.is_none());
    }

    #[test]
    fn erase_line() {
        let mut s = Scrollback::new(100);
        s.feed(b"abcdef\r\x1b[3C\x1b[K");
        assert_eq!(texts(&s), ["abc"]);
        s.feed(b"\x1b[2K");
        s.feed(b"\rZ\n");
        assert_eq!(texts(&s), ["Z"]);
        let mut s = Scrollback::new(100);
        s.feed(b"abc\x1b[0K\rxy\x1b[0K\n");
        assert_eq!(texts(&s), ["xy"]);
    }

    #[test]
    fn unknown_sequences_dropped() {
        let mut s = Scrollback::new(100);
        s.feed(b"a\x1b]0;title\x07b\x1b[?25lc\x1b[?2004hd\x1b]2;t\x1b\\e\x1b(Bf\x1b=g\x07h\n");
        assert_eq!(texts(&s), ["abcdefgh"]);
    }

    #[test]
    fn split_escape_across_feeds() {
        let mut s = Scrollback::new(100);
        for b in b"\x1b[3"
            .iter()
            .chain(b"1mR\x1b]0;x".iter())
            .chain(b"\x07\n".iter())
        {
            s.feed(&[*b]);
        }
        let l = line(&s, 0);
        assert_eq!(l.text(), "R");
        assert_eq!(l.spans[0].fg, Some(Color::Indexed(1)));
    }

    #[test]
    fn utf8_split_across_feeds() {
        let mut s = Scrollback::new(100);
        for b in "あ".as_bytes() {
            s.feed(&[*b]);
        }
        s.feed(b"\n");
        assert_eq!(texts(&s), ["あ"]);
    }

    #[test]
    fn invalid_utf8_replaced() {
        let mut s = Scrollback::new(100);
        s.feed(b"a\xffb\xe3\x81c\n");
        assert_eq!(texts(&s), ["a\u{fffd}b\u{fffd}c"]);
    }

    #[test]
    fn max_lines_trims_oldest() {
        let mut s = Scrollback::new(3);
        for i in 0..6 {
            s.feed(format!("l{i}\n").as_bytes());
        }
        assert_eq!(texts(&s), ["l3", "l4", "l5"]);
    }

    fn feed_n(s: &mut Scrollback, n: usize) {
        for i in 0..n {
            s.feed(format!("line{i}\r\n").as_bytes());
        }
    }

    #[test]
    fn fold_long_block() {
        let mut s = Scrollback::new(1000);
        feed_n(&mut s, 40);
        let rows = s.rows();
        assert_eq!(rows.len(), 11);
        assert_eq!(
            rows[0],
            Row::Folded {
                block: 0,
                hidden: 30
            }
        );
        assert_eq!(texts(&s)[1], "line30");
        s.toggle_fold(0);
        assert_eq!(s.rows().len(), 40);
        s.toggle_fold(0);
        assert_eq!(s.rows().len(), 11);
    }

    #[test]
    fn exactly_30_not_folded() {
        let mut s = Scrollback::new(1000);
        feed_n(&mut s, 30);
        assert_eq!(s.rows().len(), 30);
    }

    #[test]
    fn mark_command_splits_blocks_and_ids_survive_trim() {
        let mut s = Scrollback::new(60);
        feed_n(&mut s, 5);
        s.feed(b"$ cmd");
        s.mark_command();
        s.feed(b"\r\n");
        feed_n(&mut s, 45);
        let rows = s.rows();
        let id = rows
            .iter()
            .find_map(|r| match r {
                Row::Folded { block, .. } => Some(*block),
                _ => None,
            })
            .unwrap();
        assert_eq!(id, 1);
        // Trim old lines (drops block 0 entirely); id 1 stays valid.
        feed_n(&mut s, 20);
        s.toggle_fold(1);
        let n = s.rows().len();
        assert!(n > 11, "{n}");
        assert!(!texts(&s).iter().any(|t| t.starts_with("<folded")));
    }

    #[test]
    fn cursor_right_and_long_lines_bounded() {
        let mut s = Scrollback::new(100_000);
        for _ in 0..10_000 {
            s.feed(b"\x1b[1000C");
        }
        s.feed(b"x");
        for r in s.rows() {
            if let Row::Line(l) = r {
                assert!(l.text().chars().count() <= 4096);
            }
        }
        let mut s = Scrollback::new(100_000);
        s.feed(&vec![b'a'; 1_000_000]);
        s.toggle_fold(0);
        let rows = s.rows();
        assert!(rows.len() > 200);
        for r in rows {
            if let Row::Line(l) = r {
                assert!(l.text().chars().count() <= 4096);
            }
        }
    }

    #[test]
    fn unterminated_osc_does_not_swallow_output() {
        let mut s = Scrollback::new(100);
        s.feed(b"\x1b]0;never ended");
        s.feed(b"hello\nworld\n");
        // The OSC payload ("never ended" + "hello") is lost, but the newline
        // aborts it so later output is shown.
        assert_eq!(texts(&s), ["", "world"]);
        let mut s = Scrollback::new(100);
        s.feed(b"\x1b]0;");
        s.feed(&vec![b'z'; 5000]);
        s.feed(b"ok\n");
        assert!(texts(&s)[0].ends_with("ok"));
    }

    #[test]
    fn replace_resets() {
        let mut s = Scrollback::new(100);
        feed_n(&mut s, 50);
        s.replace(b"new\r\n");
        assert_eq!(texts(&s), ["new"]);
        s.clear();
        assert!(s.rows().is_empty());
    }
}
