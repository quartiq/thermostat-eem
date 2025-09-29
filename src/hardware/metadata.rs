use platform::ApplicationMetadata;

mod build_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

/// Construct the global metadata.
///
/// # Note
/// This may only be called once.
///
/// # Args
/// * `hardware_version` - The hardware version detected.
///
/// # Returns
/// A reference to the global metadata.
pub fn metadata(version: &'static str) -> &'static ApplicationMetadata {
    cortex_m::singleton!(: ApplicationMetadata = ApplicationMetadata {
        firmware_version: build_info::GIT_VERSION.unwrap_or("Unspecified"),
        rust_version: build_info::RUSTC_VERSION,
        profile: build_info::PROFILE,
        git_dirty: build_info::GIT_DIRTY.unwrap_or(false),
        features: build_info::FEATURES_STR,
        hardware_version: version,
        panic_info: panic_persist::get_panic_message_utf8().unwrap_or("None"),
    })
    .unwrap()
}
