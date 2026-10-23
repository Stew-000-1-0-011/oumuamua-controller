//! The bundled Japanese font, made Slint's default font for the controller UI. Some Android
//! system fallbacks have kanji but no kana, which then render as boxes.
use std::path::{Path, PathBuf};

const FONT: &[u8] = include_bytes!("../fonts/NotoSansJP-subset.otf");
const FILE_NAME: &str = "NotoSansJP-subset.otf";

/// Writes the font to `dir` (when missing or of a different size) and returns its path.
pub fn write_to(dir: &Path) -> std::io::Result<PathBuf> {
    let path = dir.join(FILE_NAME);
    let current = std::fs::metadata(&path).map(|m| m.len());
    if current.ok() != Some(FONT.len() as u64) {
        std::fs::create_dir_all(dir)?;
        let tmp = dir.join(format!("{FILE_NAME}.tmp"));
        std::fs::write(&tmp, FONT)?;
        std::fs::rename(&tmp, &path)?;
    }
    Ok(path)
}

/// Points `SLINT_DEFAULT_FONT` at the bundled font unless the user already set it. Must run
/// before any Slint backend or window is created. Failure only costs the nicer font.
pub fn install(dir: &Path) {
    if std::env::var_os("SLINT_DEFAULT_FONT").is_some() {
        return;
    }
    match write_to(dir) {
        // SAFETY: called at startup, before Slint (or any other thread) reads the environment.
        Ok(path) => unsafe { std::env::set_var("SLINT_DEFAULT_FONT", path) },
        Err(e) => eprintln!("bundled font not installed: {e}"),
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn writes_the_font_once_and_repairs_a_truncated_copy() {
        let dir = tempfile::tempdir().unwrap();
        let path = super::write_to(dir.path()).unwrap();
        assert_eq!(std::fs::metadata(&path).unwrap().len(), super::FONT.len() as u64);
        std::fs::write(&path, b"x").unwrap();
        super::write_to(dir.path()).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), super::FONT);
    }
}
