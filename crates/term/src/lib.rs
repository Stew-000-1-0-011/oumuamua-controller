//! Console logic: on-screen modifiers, local line editor, history,
//! ANSI output parsing and scrollback with folding.
pub mod ansi;
pub mod editor;
pub mod history;
pub mod keys;
pub mod scrollback;

pub use ansi::{Color, Line, Span};
pub use editor::{EditAction, LineEditor};
pub use history::History;
pub use keys::{Key, KeyEvent, ModState, Modifier, Modifiers};
pub use scrollback::{Row, Scrollback};
