//! Stabilizer network management module
//!
//! # Design
//! The stabilizer network architecture supports numerous layers to permit transmission of
//! telemetry (via MQTT), configuration of run-time settings (via MQTT + Miniconf), and live data
//! streaming over raw UDP/TCP sockets. This module encompasses the main processing routines
//! related to Stabilizer networking operations.
pub use heapless;
pub use miniconf;
pub use serde;

pub mod network_processor;
pub mod telemetry;

use crate::hardware::{
    metadata::ApplicationMetadata, system_timer::SystemTimer, EthernetPhy, NetworkManager,
    NetworkStack,
};
use network_processor::NetworkProcessor;
use telemetry::TelemetryClient;

use core::fmt::Write;
use heapless::String;
use miniconf::{JsonCoreSlash, Tree};
use serde::Serialize;

pub type NetworkReference = smoltcp_nal::shared::NetworkStackProxy<'static, NetworkStack>;

pub struct MqttStorage {
    telemetry: [u8; 2048],
    settings: [u8; 1024],
}

impl Default for MqttStorage {
    fn default() -> Self {
        Self {
            telemetry: [0u8; 2048],
            settings: [0u8; 1024],
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum UpdateState {
    NoChange,
    Updated,
}

pub enum NetworkState {
    SettingsChanged(String<128>),
    Updated,
    NoChange,
}
/// A structure of Stabilizer's default network users.
pub struct NetworkUsers<S, T, const Y: usize>
where
    for<'de> S: Default + JsonCoreSlash<'de, Y> + Clone,
    T: Serialize,
{
    pub miniconf: miniconf::MqttClient<
        'static,
        S,
        NetworkReference,
        SystemTimer,
        miniconf::minimq::broker::NamedBroker<NetworkReference>,
        Y,
    >,
    pub processor: NetworkProcessor,
    pub telemetry: TelemetryClient<T>,
}

impl<S, T, const Y: usize> NetworkUsers<S, T, Y>
where
    for<'de> S: Default + JsonCoreSlash<'de, Y> + Clone,
    T: Serialize,
{
    /// Construct Stabilizer's default network users.
    ///
    /// # Args
    /// * `stack` - The network stack that will be used to share with all network users.
    /// * `phy` - The ethernet PHY connecting the network.
    /// * `clock` - System timer clock.
    /// * `mac` - The MAC address of the network.
    /// * `broker` - The IP address of the MQTT broker to use.
    /// * `id` - The MQTT client ID base to use.
    /// * `settings` - The initial settings value
    /// * `metadata` - The metadata associated with the app.
    ///
    /// # Returns
    /// A new struct of network users.
    pub fn new(
        stack: NetworkStack,
        phy: EthernetPhy,
        clock: SystemTimer,
        id: &str,
        broker: &str,
        settings: S,
        metadata: &'static ApplicationMetadata,
    ) -> Self {
        let stack_manager =
            cortex_m::singleton!(: NetworkManager = NetworkManager::new(stack)).unwrap();

        let processor = NetworkProcessor::new(stack_manager.acquire_stack(), phy);

        let prefix = get_device_prefix(metadata.app, id);

        let store = cortex_m::singleton!(: MqttStorage = MqttStorage::default()).unwrap();

        let named_broker =
            miniconf::minimq::broker::NamedBroker::new(broker, stack_manager.acquire_stack())
                .unwrap();

        let settings = miniconf::MqttClient::new(
            stack_manager.acquire_stack(),
            &prefix,
            clock,
            settings,
            miniconf::minimq::ConfigBuilder::new(named_broker, &mut store.settings)
                .client_id(&get_client_id(id, "settings"))
                .unwrap(),
        )
        .unwrap();

        let telemetry = {
            let named_broker =
                miniconf::minimq::broker::NamedBroker::new(broker, stack_manager.acquire_stack())
                    .unwrap();

            let mqtt = minimq::Minimq::new(
                stack_manager.acquire_stack(),
                clock,
                minimq::ConfigBuilder::new(named_broker, &mut store.telemetry)
                    // The telemetry client doesn't receive any messages except MQTT control packets.
                    // As such, we don't need much of the buffer for RX.
                    .rx_buffer(minimq::config::BufferConfig::Maximum(100))
                    .client_id(&get_client_id(id, "tlm"))
                    .unwrap(),
            );

            TelemetryClient::new(mqtt, &prefix, metadata)
        };

        NetworkUsers {
            miniconf: settings,
            processor,
            telemetry,
        }
    }

    /// Update and process all of the network users state.
    ///
    /// # Returns
    /// An indication if any of the network users indicated a state change.
    pub fn update(&mut self) -> NetworkState {
        // Update the MQTT clients.
        self.telemetry.update();

        // Poll for incoming data.
        let poll_result = match self.processor.update() {
            UpdateState::NoChange => NetworkState::NoChange,
            UpdateState::Updated => NetworkState::Updated,
        };

        let mut settings_path: String<128> = String::new();
        match self.miniconf.handled_update(|path, old, new| {
            settings_path = path.into();
            *old = new.clone();
            Result::<(), &'static str>::Ok(())
        }) {
            Ok(true) => NetworkState::SettingsChanged(settings_path),
            _ => poll_result,
        }
    }
}

/// Get an MQTT client ID for a client.
///
/// # Args
/// * `id` - The base client ID
/// * `mode` - The operating mode of the client. (i.e. tlm, settings)
///
/// # Returns
/// A client ID that may be used for MQTT client identification.
fn get_client_id(id: &str, mode: &str) -> String<64> {
    let mut identifier = String::new();
    write!(&mut identifier, "{id}-{mode}").unwrap();
    identifier
}

/// Get the MQTT prefix of a device.
///
/// # Args
/// * `app` - The name of the application that is executing.
/// * `id` - The MQTT ID of the device.
///
/// # Returns
/// The MQTT prefix used for this device.
pub fn get_device_prefix(app: &str, id: &str) -> String<128> {
    // Note(unwrap): The mac address + binary name must be short enough to fit into this string. If
    // they are defined too long, this will panic and the device will fail to boot.
    let mut prefix: String<128> = String::new();
    write!(&mut prefix, "dt/sinara/{app}/{id}").unwrap();

    prefix
}

/// Miniconf settings for the MQTT alarm.
/// The alarm simply publishes "false" onto its [target] as long as all the channels are
/// within their [temperature_limits] (aka logical OR of all channels).
/// Otherwise it publishes "true" (aka true, there is an alarm).
///
/// The publishing interval is given by [period_ms].
///
/// The alarm is non-latching. If alarm was "true" for a while and the temperatures come within
/// limits again, alarm will be "false" again.
#[derive(Clone, Debug, Tree)]
pub struct Alarm {
    /// Set the alarm to armed (true) or disarmed (false).
    /// If the alarm is armed, the device will publish it's alarm state onto the [target].
    ///
    /// # Value
    /// True to arm, false to disarm.
    pub armed: bool,

    /// Alarm target.
    /// The alarm will publish its state (true or false) onto this mqtt path.
    /// Full path to the desired target. No wildcards.
    ///
    /// # Value
    /// Any string up to 128 characters.
    pub target: String<128>,

    /// Alarm period in milliseconds.
    /// The alarm will publish its state with this period.
    ///
    /// # Value
    /// f32
    pub period: f32,

    /// Temperature limits for the alarm.
    ///
    /// Array of lower and upper limits for the valid temperature range of the alarm.
    /// The alarm will be asserted if any of the enabled input channels goes below its minimum or above its maximum temperature.
    /// The alarm is non latching and clears itself once all channels are in their respective limits.
    ///
    /// # Path
    /// `temperature_limits/<adc>/<channel>`
    /// * <adc> specifies which adc to configure. <adc> := [0, 1, 2, 3]
    /// * <channel> specifies which channel of an ADC to configure. Only the enabled channels for the specific ADC are available.
    ///
    /// # Value
    /// [f32, f32]
    #[tree(depth(4))]
    pub temperature_limits: [[Option<[f32; 2]>; 4]; 4],
}

impl Default for Alarm {
    fn default() -> Self {
        Self {
            armed: false,
            target: Default::default(),
            period: 1.0,
            temperature_limits: [[Some([f32::NEG_INFINITY, f32::INFINITY]); 4]; 4],
        }
    }
}
