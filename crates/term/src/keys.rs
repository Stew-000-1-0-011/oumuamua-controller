//! On-screen keyboard modifiers.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Key {
    Char(char),
    Enter,
    Tab,
    Backspace,
    Delete,
    Left,
    Right,
    Up,
    Down,
    Home,
    End,
    Esc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyEvent {
    Key(Key),
    /// Lowercase character.
    Ctrl(char),
    Alt(char),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modifier {
    Ctrl,
    Alt,
    Shift,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ModState {
    #[default]
    Off,
    Once,
    Locked,
}

impl ModState {
    fn next(self) -> Self {
        match self {
            ModState::Off => ModState::Once,
            ModState::Once => ModState::Locked,
            ModState::Locked => ModState::Off,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Modifiers {
    ctrl: ModState,
    alt: ModState,
    shift: ModState,
}

fn shifted(c: char) -> char {
    match c {
        '1' => '!',
        '2' => '@',
        '3' => '#',
        '4' => '$',
        '5' => '%',
        '6' => '^',
        '7' => '&',
        '8' => '*',
        '9' => '(',
        '0' => ')',
        '-' => '_',
        '=' => '+',
        '[' => '{',
        ']' => '}',
        '\\' => '|',
        ';' => ':',
        '\'' => '"',
        ',' => '<',
        '.' => '>',
        '/' => '?',
        '`' => '~',
        c if c.is_ascii_lowercase() => c.to_ascii_uppercase(),
        c => c,
    }
}

impl Modifiers {
    pub fn new() -> Self {
        Self::default()
    }

    fn slot(&mut self, m: Modifier) -> &mut ModState {
        match m {
            Modifier::Ctrl => &mut self.ctrl,
            Modifier::Alt => &mut self.alt,
            Modifier::Shift => &mut self.shift,
        }
    }

    /// Off -> Once -> Locked -> Off.
    pub fn tap(&mut self, m: Modifier) {
        let s = self.slot(m);
        *s = s.next();
    }

    pub fn state(&self, m: Modifier) -> ModState {
        match m {
            Modifier::Ctrl => self.ctrl,
            Modifier::Alt => self.alt,
            Modifier::Shift => self.shift,
        }
    }

    /// Turn a pressed key into an event, consuming one-shot modifiers.
    pub fn press(&mut self, k: Key) -> KeyEvent {
        let (ctrl, alt, shift) = (self.ctrl, self.alt, self.shift);
        for s in [&mut self.ctrl, &mut self.alt, &mut self.shift] {
            if *s == ModState::Once {
                *s = ModState::Off;
            }
        }
        let on = |s: ModState| s != ModState::Off;
        match k {
            Key::Char(c) => {
                let c = if on(shift) { shifted(c) } else { c };
                if on(ctrl) {
                    KeyEvent::Ctrl(c.to_ascii_lowercase())
                } else if on(alt) {
                    KeyEvent::Alt(c)
                } else {
                    KeyEvent::Key(Key::Char(c))
                }
            }
            k => KeyEvent::Key(k),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modifiers_once_lock_off() {
        let mut m = Modifiers::new();
        assert_eq!(m.state(Modifier::Ctrl), ModState::Off);
        m.tap(Modifier::Ctrl);
        assert_eq!(m.state(Modifier::Ctrl), ModState::Once);
        assert_eq!(m.press(Key::Char('c')), KeyEvent::Ctrl('c'));
        assert_eq!(m.state(Modifier::Ctrl), ModState::Off);
        assert_eq!(m.press(Key::Char('c')), KeyEvent::Key(Key::Char('c')));
        m.tap(Modifier::Alt);
        m.tap(Modifier::Alt);
        assert_eq!(m.state(Modifier::Alt), ModState::Locked);
        assert_eq!(m.press(Key::Char('x')), KeyEvent::Alt('x'));
        assert_eq!(m.press(Key::Char('y')), KeyEvent::Alt('y'));
        m.tap(Modifier::Alt);
        assert_eq!(m.state(Modifier::Alt), ModState::Off);
        // a non-char key also consumes a one-shot
        m.tap(Modifier::Ctrl);
        assert_eq!(m.press(Key::Enter), KeyEvent::Key(Key::Enter));
        assert_eq!(m.state(Modifier::Ctrl), ModState::Off);
    }

    #[test]
    fn shift_maps_symbols() {
        let mut m = Modifiers::new();
        m.tap(Modifier::Shift);
        assert_eq!(m.press(Key::Char('a')), KeyEvent::Key(Key::Char('A')));
        assert_eq!(m.state(Modifier::Shift), ModState::Off);
        m.tap(Modifier::Shift);
        m.tap(Modifier::Shift);
        let pairs = "1!2@3#4$5%6^7&8*9(0)-_=+[{]}\\|;:'\",<.>/?`~";
        let cs: Vec<char> = pairs.chars().collect();
        for p in cs.chunks(2) {
            assert_eq!(m.press(Key::Char(p[0])), KeyEvent::Key(Key::Char(p[1])));
        }
        assert_eq!(m.state(Modifier::Shift), ModState::Locked);
    }
}
