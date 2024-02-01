/// Thermostat Telemetry Capabilities
///
/// # Design
/// Telemetry is reported regularly using an MQTT client. All telemetry is reported in SI units
/// using standard JSON format.
///
/// In order to report ADC/DAC codes generated during the DSP routines, a telemetry buffer is
/// employed to track the latest codes. Converting these codes to SI units would result in
/// repetitive and unnecessary calculations within the DSP routine, slowing it down and limiting
/// sampling frequency. Instead, the raw codes are stored and the telemetry is generated as
/// required immediately before transmission. This ensures that any slower computation required
/// for unit conversion can be off-loaded to lower priority tasks.
use heapless::String;
use serde::Serialize;

use super::NetworkReference;
use crate::hardware::{metadata::ApplicationMetadata, system_timer::SystemTimer};

/// Default metadata message if formatting errors occur.
const DEFAULT_METADATA: &str = "{\"message\":\"Truncated\"}";

/// The telemetry client for reporting telemetry data over MQTT.
pub struct TelemetryClient<T: Serialize> {
    mqtt: minimq::Minimq<
        'static,
        NetworkReference,
        SystemTimer,
        minimq::broker::NamedBroker<NetworkReference>,
    >,
    prefix: String<128>,
    meta_published: bool,
    metadata: &'static ApplicationMetadata,
    _telemetry: core::marker::PhantomData<T>,
}

impl<T: Serialize> TelemetryClient<T> {
    /// Construct a new telemetry client.
    ///
    /// # Args
    /// * `mqtt` - The MQTT client
    /// * `prefix` - The device prefix to use for MQTT telemetry reporting
    ///
    /// # Returns
    /// A new telemetry client.
    pub fn new(
        mqtt: minimq::Minimq<
            'static,
            NetworkReference,
            SystemTimer,
            minimq::broker::NamedBroker<NetworkReference>,
        >,
        prefix: &str,
        metadata: &'static ApplicationMetadata,
    ) -> Self {
        Self {
            mqtt,
            prefix: String::from(prefix),
            metadata,
            meta_published: false,
            _telemetry: core::marker::PhantomData,
        }
    }

    /// Publish telemetry over MQTT
    ///
    /// # Note
    /// Telemetry is reported in a "best-effort" fashion. Failure to transmit telemetry will cause
    /// it to be silently dropped.
    ///
    /// # Args
    /// * `telemetry` - The telemetry to report
    pub fn publish(&mut self, telemetry: &T) {
        let mut topic = self.prefix.clone();
        topic.push_str("/telemetry").unwrap();

        self.mqtt
            .client()
            .publish(
                minimq::DeferredPublication::new(|buf| serde_json_core::to_slice(&telemetry, buf))
                    .topic(&topic)
                    .finish()
                    .unwrap(),
            )
            .ok();
    }

    /// A secondary functionality tugged onto the telemetry client that publishes onto another
    /// [alarm_topic].
    pub fn publish_alarm(&mut self, alarm_topic: &String<128>, alarm: &bool) {
        self.mqtt
            .client()
            .publish(
                minimq::DeferredPublication::new(|buf| serde_json_core::to_slice(alarm, buf))
                    .topic(alarm_topic)
                    .finish()
                    .unwrap(),
            )
            .ok();
    }

    /// Update the telemetry client
    ///
    /// # Note
    /// This function is provided to force the underlying MQTT state machine to process incoming
    /// and outgoing messages. Without this, the client will never connect to the broker. This
    /// should be called regularly.
    pub fn update(&mut self) {
        match self.mqtt.poll(|_client, _topic, _message, _properties| {}) {
            Err(minimq::Error::Network(smoltcp_nal::NetworkError::TcpConnectionFailure(
                smoltcp_nal::smoltcp::socket::tcp::ConnectError::Unaddressable,
            ))) => {}

            Err(error) => log::info!("Unexpected error: {:?}", error),
            _ => {}
        }

        if !self.mqtt.client().is_connected() {
            self.meta_published = false;
            return;
        }

        // Publish application metadata
        if !self.meta_published && self.mqtt.client().can_publish(minimq::QoS::AtMostOnce) {
            let Self {
                ref mut mqtt,
                metadata,
                ..
            } = self;

            let mut topic = self.prefix.clone();
            topic.push_str("/meta").unwrap();

            if mqtt
                .client()
                .publish(
                    minimq::DeferredPublication::new(|buf| {
                        serde_json_core::to_slice(&metadata, buf)
                    })
                    .topic(&topic)
                    .finish()
                    .unwrap(),
                )
                .is_err()
            {
                // Note(unwrap): We can guarantee that this message will be sent because we checked
                // for ability to publish above.
                mqtt.client()
                    .publish(
                        minimq::Publication::new(DEFAULT_METADATA.as_bytes())
                            .topic(&topic)
                            .finish()
                            .unwrap(),
                    )
                    .unwrap();
            }

            self.meta_published = true;
        }
    }
}
