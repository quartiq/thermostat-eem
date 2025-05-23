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

pub mod data_stream;
pub mod network_processor;
pub mod telemetry;

use crate::{
    hardware::{metadata::ApplicationMetadata, EthernetPhy, NetworkManager, NetworkStack},
    settings::NetSettings,
    SystemTimer,
};
use data_stream::{DataStream, FrameGenerator, StreamTarget};
use network_processor::NetworkProcessor;
use telemetry::TelemetryClient;

use core::fmt::Write;
use heapless::String;
use miniconf::{Leaf, Tree, TreeDeserializeOwned, TreeKey, TreeSerialize};

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

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum NetworkState {
    SettingsChanged,
    Updated,
    NoChange,
}
/// A structure of Stabilizer's default network users.
pub struct NetworkUsers<S, const Y: usize>
where
    S: Default + TreeDeserializeOwned + TreeSerialize + TreeKey + Clone,
{
    pub miniconf: miniconf_mqtt::MqttClient<
        'static,
        S,
        NetworkReference,
        SystemTimer,
        minimq::broker::NamedBroker<NetworkReference>,
        Y,
    >,
    pub processor: NetworkProcessor,
    stream: DataStream,
    generator: Option<FrameGenerator>,
    pub telemetry: TelemetryClient,
}

impl<S, const Y: usize> NetworkUsers<S, Y>
where
    S: Default + TreeDeserializeOwned + TreeSerialize + TreeKey + Clone,
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
        clock: crate::SystemTimer,
        app: &str,
        net_settings: &NetSettings,
        metadata: &'static ApplicationMetadata,
    ) -> Self {
        let stack_manager =
            cortex_m::singleton!(: NetworkManager = NetworkManager::new(stack)).unwrap();

        let processor = NetworkProcessor::new(stack_manager.acquire_stack(), phy);

        let prefix =
            cortex_m::singleton!(: String<128> = get_device_prefix(app, &net_settings.id)).unwrap();

        let store = cortex_m::singleton!(: MqttStorage = MqttStorage::default()).unwrap();

        let named_broker = miniconf_mqtt::minimq::broker::NamedBroker::new(
            &net_settings.broker,
            stack_manager.acquire_stack(),
        )
        .unwrap();

        let settings = miniconf_mqtt::MqttClient::new(
            stack_manager.acquire_stack(),
            prefix.as_str(),
            clock,
            miniconf_mqtt::minimq::ConfigBuilder::new(named_broker, &mut store.settings)
                .client_id(&get_client_id(&net_settings.id, "settings"))
                .unwrap(),
        )
        .unwrap();

        let telemetry = {
            let named_broker = miniconf_mqtt::minimq::broker::NamedBroker::new(
                &net_settings.broker,
                stack_manager.acquire_stack(),
            )
            .unwrap();

            let mqtt = minimq::Minimq::new(
                stack_manager.acquire_stack(),
                clock,
                minimq::ConfigBuilder::new(named_broker, &mut store.telemetry)
                    // The telemetry client doesn't receive any messages except MQTT control packets.
                    // As such, we don't need much of the buffer for RX.
                    .rx_buffer(minimq::config::BufferConfig::Maximum(100))
                    .session_state(minimq::config::BufferConfig::Maximum(0))
                    .client_id(&get_client_id(&net_settings.id, "tlm"))
                    .unwrap(),
            );

            TelemetryClient::new(mqtt, prefix, metadata)
        };

        let (generator, stream) = data_stream::setup_streaming(stack_manager.acquire_stack());

        NetworkUsers {
            miniconf: settings,
            processor,
            telemetry,
            stream,
            generator: Some(generator),
        }
    }

    /// Enable live data streaming.
    ///
    /// # Args
    /// * `format` - A unique u8 code indicating the format of the data.
    pub fn configure_streaming(&mut self, format: u8) -> FrameGenerator {
        let mut generator = self.generator.take().unwrap();
        generator.configure(format);
        generator
    }

    /// Direct the stream to the provided remote target.
    ///
    /// # Args
    /// * `remote` - The destination for the streamed data.
    pub fn direct_stream(&mut self, remote: StreamTarget) {
        if self.generator.is_none() {
            self.stream.set_remote(remote);
        }
    }

    /// Update and process all of the network users state.
    ///
    /// # Returns
    /// An indication if any of the network users indicated a state change.
    pub fn update(&mut self, settings: &mut S) -> NetworkState {
        // Update the MQTT clients.
        self.telemetry.update();

        // Update the data stream.
        if self.generator.is_none() {
            self.stream.process();
        }

        // Poll for incoming data.
        let poll_result = match self.processor.update() {
            UpdateState::NoChange => NetworkState::NoChange,
            UpdateState::Updated => NetworkState::Updated,
        };

        match self.miniconf.update(settings) {
            Ok(true) => NetworkState::SettingsChanged,
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
/// The alarm simply publishes "false" onto its `target` as long as all the channels are
/// within their `temperature_limits`` (aka logical OR of all channels).
/// Otherwise it publishes "true" (aka true, there is an alarm).
///
/// The publishing interval is given by `period_ms`.
///
/// The alarm is non-latching. If alarm was "true" for a while and the temperatures come within
/// limits again, alarm will be "false" again.
#[derive(Clone, Debug, Tree)]
pub struct Alarm {
    /// Set the alarm to armed (true) or disarmed (false).
    /// If the alarm is armed, the device will publish it's alarm state onto the `target`.
    ///
    /// # Value
    /// True to arm, false to disarm.
    pub armed: Leaf<bool>,

    /// Alarm target.
    /// The alarm will publish its state (true or false) onto this mqtt path.
    /// Full path to the desired target. No wildcards.
    ///
    /// # Value
    /// Any string up to 128 characters.
    pub target: Leaf<String<128>>,

    /// Alarm period in milliseconds.
    /// The alarm will publish its state with this period.
    ///
    /// # Value
    /// f32
    pub period: Leaf<f32>,

    /// Temperature limits for the alarm.
    ///
    /// Array of lower and upper limits for the valid temperature range of the alarm.
    /// The alarm will be asserted if any of the enabled input channels goes below its minimum or above its maximum temperature.
    /// The alarm is non latching and clears itself once all channels are in their respective limits.
    ///
    /// # Path
    /// `temperature_limits/<adc>/<channel>`
    /// * `<adc> := [0, 1, 2, 3]` specifies which adc to configure.
    /// * `<channel>` specifies which channel of an ADC to configure. Only the enabled channels for the specific ADC are available.
    ///
    /// # Value
    /// `[f32, f32]` or `None`
    pub temperature_limits: [[Option<[Leaf<f32>; 2]>; 4]; 4],
}

impl Default for Alarm {
    fn default() -> Self {
        Self {
            armed: false.into(),
            target: Default::default(),
            period: 1.0.into(),
            temperature_limits: Default::default(),
        }
    }
}
