///! Stabilizer network management module
///!
///! # Design
///! The stabilizer network architecture supports numerous layers to permit transmission of
///! telemetry (via MQTT), configuration of run-time settings (via MQTT + Miniconf), and live data
///! streaming over raw UDP/TCP sockets. This module encompasses the main processing routines
///! related to Stabilizer networking operations.
pub use heapless;
pub use miniconf;
pub use serde;

pub mod network_processor;
pub mod telemetry;

use crate::hardware::{system_timer::SystemTimer, EthernetPhy, NetworkManager, NetworkStack};
use minimq::embedded_nal::IpAddr;
use network_processor::NetworkProcessor;
use telemetry::TelemetryClient;

use core::fmt::Write;
use heapless::String;
use miniconf::Miniconf;
use serde::Serialize;

pub type NetworkReference = smoltcp_nal::shared::NetworkStackProxy<'static, NetworkStack>;

/// The default MQTT broker IP address if unspecified.
pub const DEFAULT_MQTT_BROKER: [u8; 4] = [10, 34, 16, 10];

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
pub struct NetworkUsers<S: Default + Miniconf + Clone, T: Serialize> {
    pub miniconf: miniconf::MqttClient<S, NetworkReference, SystemTimer, 512>,
    pub processor: NetworkProcessor,
    pub telemetry: TelemetryClient<T>,
}

impl<S, T> NetworkUsers<S, T>
where
    S: Default + Miniconf + Clone,
    T: Serialize,
{
    /// Construct Stabilizer's default network users.
    ///
    /// # Args
    /// * `stack` - The network stack that will be used to share with all network users.
    /// * `phy` - The ethernet PHY connecting the network.
    /// * `clock` - System timer clock.
    /// * `app` - The name of the application.
    /// * `mac` - The MAC address of the network.
    /// * `broker` - The IP address of the MQTT broker to use.
    ///
    /// # Returns
    /// A new struct of network users.
    pub fn new(
        stack: NetworkStack,
        phy: EthernetPhy,
        clock: SystemTimer,
        app: &str,
        mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
        broker: IpAddr,
    ) -> Self {
        let stack_manager =
            cortex_m::singleton!(: NetworkManager = NetworkManager::new(stack)).unwrap();

        let processor = NetworkProcessor::new(stack_manager.acquire_stack(), phy);

        let prefix = get_device_prefix(app, mac);

        let settings = miniconf::MqttClient::new(
            stack_manager.acquire_stack(),
            &get_client_id(app, "settings", mac),
            &prefix,
            broker,
            clock,
            S::default(),
        )
        .unwrap();

        let telemetry = TelemetryClient::new(
            stack_manager.acquire_stack(),
            clock,
            &get_client_id(app, "tlm", mac),
            &prefix,
            broker,
        );

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

        match self.miniconf.update() {
            Ok(true) => NetworkState::SettingsChanged,
            _ => poll_result,
        }
    }
}

/// Get an MQTT client ID for a client.
///
/// # Args
/// * `app` - The name of the application
/// * `client` - The unique tag of the client
/// * `mac` - The MAC address of the device.
///
/// # Returns
/// A client ID that may be used for MQTT client identification.
fn get_client_id(
    app: &str,
    client: &str,
    mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
) -> String<64> {
    let mut identifier = String::new();
    write!(&mut identifier, "{}-{}-{}", app, mac, client).unwrap();
    identifier
}

/// Get the MQTT prefix of a device.
///
/// # Args
/// * `app` - The name of the application that is executing.
/// * `mac` - The ethernet MAC address of the device.
///
/// # Returns
/// The MQTT prefix used for this device.
pub fn get_device_prefix(
    app: &str,
    mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
) -> String<128> {
    // Note(unwrap): The mac address + binary name must be short enough to fit into this string. If
    // they are defined too long, this will panic and the device will fail to boot.
    let mut prefix: String<128> = String::new();
    write!(&mut prefix, "dt/sinara/{}/{}", app, mac).unwrap();

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
#[derive(Clone, Debug, Miniconf)]
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
    /// u64
    pub period_ms: u64,

    /// Temperature limits for the alarm.
    ///
    /// Array of lower (0) and (1) limits for the valid temperature range of the alarm.
    /// The alarm will be enabled if any of the input channels goes below its minimum or above its maximum temperature.
    /// The alarm is non latching and clears itself once all channels are in their respective limits.
    ///
    /// # Value
    /// [[f32, f32]; 8]
    pub temperature_limits: [[f32; 2]; 8],
}
