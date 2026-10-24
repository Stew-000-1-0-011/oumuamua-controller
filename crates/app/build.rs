use slint_build::{CompilerConfiguration, EmbedResourcesKind};

fn main() {
    // Embed the imported fonts in the binary: the build machine's paths do not exist on Android.
    // Debug info (element ids) in non-release builds: the testing backend's ElementHandle needs it.
    let debug = std::env::var("PROFILE").is_ok_and(|p| p != "release");
    let config = CompilerConfiguration::new().embed_resources(EmbedResourcesKind::EmbedFiles).with_debug_info(debug);
    slint_build::compile_with_config("ui/main.slint", config).unwrap();
}
