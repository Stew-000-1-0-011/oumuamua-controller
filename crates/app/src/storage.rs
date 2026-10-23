//! Tiny key-file storage in the app's data directory.

use std::path::{Path, PathBuf};
use std::sync::OnceLock;

static OVERRIDE: OnceLock<PathBuf> = OnceLock::new();

/// Fix the data directory (Android passes its internal files directory). First call wins.
pub fn set_data_dir(dir: PathBuf) {
    let _ = OVERRIDE.set(dir);
}

/// Android: the directory given to `set_data_dir`. Desktop: `~/.config/oumuamua-controller`.
pub fn data_dir() -> PathBuf {
    if let Some(d) = OVERRIDE.get() {
        return d.clone();
    }
    let home = std::env::var_os("HOME").map(PathBuf::from).unwrap_or_else(|| PathBuf::from("."));
    home.join(".config").join("oumuamua-controller")
}

pub fn load_from(dir: &Path, name: &str) -> Option<String> {
    std::fs::read_to_string(dir.join(name)).ok()
}

pub fn save_to(dir: &Path, name: &str, value: &str) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let tmp = dir.join(format!("{name}.tmp"));
    std::fs::write(&tmp, value)?;
    std::fs::rename(&tmp, dir.join(name))
}

pub fn load(name: &str) -> Option<String> {
    load_from(&data_dir(), name)
}

/// Failure only costs persistence; it is reported on stderr.
pub fn save(name: &str, value: &str) {
    if let Err(e) = save_to(&data_dir(), name, value) {
        eprintln!("could not save {name}: {e}");
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn save_then_load_round_trips() {
        let dir = tempfile::tempdir().unwrap();
        let d = dir.path().join("nested");
        assert_eq!(super::load_from(&d, "a"), None);
        super::save_to(&d, "a", "x").unwrap();
        super::save_to(&d, "a", "y").unwrap();
        assert_eq!(super::load_from(&d, "a").as_deref(), Some("y"));
    }
}
