//! Console input logic: on-screen modifiers, local line editor, history.
pub mod editor;
pub mod history;
pub mod keys;

pub use editor::{EditAction, LineEditor};
pub use history::History;
pub use keys::{Key, KeyEvent, ModState, Modifier, Modifiers};
