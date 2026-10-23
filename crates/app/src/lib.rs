pub mod auto;
pub mod console;
pub mod font;
pub mod glue;
pub mod manual;
pub mod map;
pub mod model;
pub mod storage;

slint::include_modules!();

/// Runs the controller until its window is closed.
pub fn run() -> Result<(), slint::PlatformError> {
    let window = MainWindow::new()?;
    let _glue = glue::start(&window);
    window.run()
}

#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
fn android_main(app: slint::android::AndroidApp) {
    let dir = app.internal_data_path().or_else(|| app.external_data_path()).expect("no data path");
    storage::set_data_dir(dir.clone());
    // Before the Slint backend starts, so its font collection sees the variable.
    font::install(&dir.join("fonts"));
    slint::android::init(app).unwrap();
    run().unwrap();
}
