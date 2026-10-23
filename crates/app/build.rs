use slint_build::{CompilerConfiguration, EmbedResourcesKind};

fn main() {
    // Embed the imported fonts in the binary: the build machine's paths do not exist on Android.
    let config = CompilerConfiguration::new().embed_resources(EmbedResourcesKind::EmbedFiles);
    slint_build::compile_with_config("ui/main.slint", config).unwrap();
}
