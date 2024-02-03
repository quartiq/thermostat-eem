use serde::Serialize;

mod build_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[derive(Serialize)]
pub struct ApplicationMetadata {
    pub app: &'static str,
    pub firmware_version: &'static str,
    pub rust_version: &'static str,
    pub profile: &'static str,
    pub git_dirty: bool,
    pub features: &'static str,
}

impl ApplicationMetadata {
    /// Construct the global metadata.
    ///
    /// # Note
    /// This may only be called once.
    ///
    /// # Returns
    /// A reference to the global metadata.
    pub fn new() -> &'static ApplicationMetadata {
        cortex_m::singleton!(: ApplicationMetadata = ApplicationMetadata {
            app: env!("CARGO_BIN_NAME"),
            firmware_version: build_info::GIT_VERSION.unwrap_or(build_info::PKG_VERSION),
            rust_version: build_info::RUSTC_VERSION,
            profile: build_info::PROFILE,
            git_dirty: build_info::GIT_DIRTY.unwrap_or(false),
            features: build_info::FEATURES_STR,
        })
        .unwrap()
    }
}
