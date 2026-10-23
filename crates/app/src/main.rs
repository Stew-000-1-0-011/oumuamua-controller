fn main() -> Result<(), slint::PlatformError> {
    app::font::install(&app::storage::data_dir().join("fonts"));
    app::run()
}
