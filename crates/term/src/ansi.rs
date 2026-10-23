//! Incremental parser for the subset of ANSI/VT output a bash PTY produces.
//!
//! The parser turns bytes into [`Event`]s (printing, cursor movement, erase,
//! style changes). Escape sequences and UTF-8 sequences split across `feed`
//! calls are handled by keeping state between calls.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Color {
    Indexed(u8),
    Rgb(u8, u8, u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Style {
    pub fg: Option<Color>,
    pub bg: Option<Color>,
    pub bold: bool,
}

/// A run of characters with the same style.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Span {
    pub text: String,
    pub fg: Option<Color>,
    pub bg: Option<Color>,
    pub bold: bool,
}

#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct Line {
    pub spans: Vec<Span>,
}

impl Line {
    /// The characters of the line, without styling.
    pub fn text(&self) -> String {
        self.spans.iter().map(|s| s.text.as_str()).collect()
    }
}

/// One screen cell: a character and its style.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cell {
    pub ch: char,
    pub style: Style,
}

/// Group cells into spans of equal style.
pub fn cells_to_line(cells: &[Cell]) -> Line {
    let mut spans: Vec<Span> = Vec::new();
    let mut cur: Option<Style> = None;
    for c in cells {
        if cur == Some(c.style) {
            spans.last_mut().unwrap().text.push(c.ch);
        } else {
            spans.push(Span {
                text: c.ch.to_string(),
                fg: c.style.fg,
                bg: c.style.bg,
                bold: c.style.bold,
            });
            cur = Some(c.style);
        }
    }
    Line { spans }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    Print(char, Style),
    Newline,
    CarriageReturn,
    Backspace,
    Tab,
    /// 0 = cursor to end, 1 = start to cursor, 2 = whole line.
    EraseLine(u8),
    CursorRight(usize),
    CursorLeft(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Ground,
    Esc,
    Csi,
    Osc,
    OscEsc,
}

#[derive(Debug)]
pub struct Parser {
    state: State,
    style: Style,
    csi: Vec<u8>,
    utf8: Vec<u8>,
    utf8_need: usize,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    pub fn new() -> Self {
        Parser {
            state: State::Ground,
            style: Style::default(),
            csi: Vec::new(),
            utf8: Vec::new(),
            utf8_need: 0,
        }
    }

    pub fn feed(&mut self, bytes: &[u8], out: &mut Vec<Event>) {
        for &b in bytes {
            self.byte(b, out);
        }
    }

    fn byte(&mut self, b: u8, out: &mut Vec<Event>) {
        match self.state {
            State::Ground => self.ground(b, out),
            State::Esc => match b {
                b'[' => {
                    self.csi.clear();
                    self.state = State::Csi;
                }
                b']' => self.state = State::Osc,
                0x20..=0x2f => {} // intermediate: keep waiting for the final byte
                0x1b => {}
                _ => self.state = State::Ground,
            },
            State::Csi => match b {
                0x40..=0x7e => {
                    self.finish_csi(b, out);
                    self.state = State::Ground;
                }
                0x1b => self.state = State::Esc,
                0x20..=0x3f if self.csi.len() < 64 => self.csi.push(b),
                _ => {} // stray control byte inside a CSI: ignore
            },
            State::Osc => match b {
                0x07 => self.state = State::Ground,
                0x1b => self.state = State::OscEsc,
                _ => {}
            },
            State::OscEsc => match b {
                b'\\' => self.state = State::Ground,
                0x07 => self.state = State::Ground,
                b'[' => {
                    self.csi.clear();
                    self.state = State::Csi;
                }
                b']' => self.state = State::Osc,
                0x1b => {}
                _ => self.state = State::Ground,
            },
        }
    }

    fn ground(&mut self, b: u8, out: &mut Vec<Event>) {
        if self.utf8_need > 0 {
            if b & 0xc0 == 0x80 {
                self.utf8.push(b);
                if self.utf8.len() == self.utf8_need {
                    let ch = std::str::from_utf8(&self.utf8)
                        .ok()
                        .and_then(|s| s.chars().next())
                        .unwrap_or('\u{fffd}');
                    self.utf8.clear();
                    self.utf8_need = 0;
                    out.push(Event::Print(ch, self.style));
                }
                return;
            }
            // Broken sequence: emit a replacement and reprocess this byte.
            self.utf8.clear();
            self.utf8_need = 0;
            out.push(Event::Print('\u{fffd}', self.style));
        }
        match b {
            0x1b => self.state = State::Esc,
            b'\n' => out.push(Event::Newline),
            b'\r' => out.push(Event::CarriageReturn),
            0x08 => out.push(Event::Backspace),
            b'\t' => out.push(Event::Tab),
            0x00..=0x1f | 0x7f => {} // BEL and other controls dropped
            0x20..=0x7e => out.push(Event::Print(b as char, self.style)),
            0xc2..=0xdf => self.start_utf8(b, 2),
            0xe0..=0xef => self.start_utf8(b, 3),
            0xf0..=0xf4 => self.start_utf8(b, 4),
            _ => out.push(Event::Print('\u{fffd}', self.style)),
        }
    }

    fn start_utf8(&mut self, b: u8, need: usize) {
        self.utf8.clear();
        self.utf8.push(b);
        self.utf8_need = need;
    }

    fn finish_csi(&mut self, fin: u8, out: &mut Vec<Event>) {
        let params = std::mem::take(&mut self.csi);
        // Private (?, <, =, >) and intermediate-bearing sequences are dropped.
        if params
            .iter()
            .any(|&c| !(c.is_ascii_digit() || c == b';' || c == b':'))
        {
            return;
        }
        let nums: Vec<Option<u32>> = params
            .split(|&c| c == b';' || c == b':')
            .map(|p| {
                if p.is_empty() {
                    None
                } else {
                    std::str::from_utf8(p).ok().and_then(|s| s.parse().ok())
                }
            })
            .collect();
        let first = nums.first().copied().flatten();
        match fin {
            b'm' => self.sgr(&nums),
            b'K' => match first.unwrap_or(0) {
                0 => out.push(Event::EraseLine(0)),
                1 => out.push(Event::EraseLine(1)),
                2 => out.push(Event::EraseLine(2)),
                _ => {}
            },
            b'C' => out.push(Event::CursorRight(
                first.unwrap_or(1).clamp(1, 1000) as usize
            )),
            b'D' => out.push(Event::CursorLeft(first.unwrap_or(1).clamp(1, 1000) as usize)),
            _ => {}
        }
    }

    fn sgr(&mut self, nums: &[Option<u32>]) {
        let mut i = 0;
        while i < nums.len() {
            let n = nums[i].unwrap_or(0);
            match n {
                0 => self.style = Style::default(),
                1 => self.style.bold = true,
                22 => self.style.bold = false,
                30..=37 => self.style.fg = Some(Color::Indexed((n - 30) as u8)),
                39 => self.style.fg = None,
                40..=47 => self.style.bg = Some(Color::Indexed((n - 40) as u8)),
                49 => self.style.bg = None,
                90..=97 => self.style.fg = Some(Color::Indexed((n - 90 + 8) as u8)),
                100..=107 => self.style.bg = Some(Color::Indexed((n - 100 + 8) as u8)),
                38 | 48 => {
                    let color = match nums.get(i + 1).copied().flatten() {
                        Some(5) => {
                            let c = nums.get(i + 2).copied().flatten();
                            i += 2;
                            c.map(|v| Color::Indexed(v.min(255) as u8))
                        }
                        Some(2) => {
                            let g = |k: usize| {
                                nums.get(i + k).copied().flatten().unwrap_or(0).min(255) as u8
                            };
                            let c = Color::Rgb(g(2), g(3), g(4));
                            let have = nums.len() >= i + 5;
                            i += 4;
                            have.then_some(c)
                        }
                        _ => {
                            i += 1;
                            None
                        }
                    };
                    if let Some(c) = color {
                        if n == 38 {
                            self.style.fg = Some(c);
                        } else {
                            self.style.bg = Some(c);
                        }
                    }
                }
                _ => {}
            }
            i += 1;
        }
    }
}
