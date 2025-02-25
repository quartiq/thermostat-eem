//! Thermostat Telemetry Capabilities
//!
//! # Design
//! Telemetry is reported regularly using an MQTT client. All telemetry is reported in SI units
//! using standard JSON format.
//!
//! In order to report ADC/DAC codes generated during the DSP routines, a telemetry buffer is
//! employed to track the latest codes. Converting these codes to SI units would result in
//! repetitive and unnecessary calculations within the DSP routine, slowing it down and limiting
//! sampling frequency. Instead, the raw codes are stored and the telemetry is generated as
//! required immediately before transmission. This ensures that any slower computation required
//! for unit conversion can be off-loaded to lower priority tasks.
use heapless::String;
use minimq::Publication;
use serde::Serialize;

use super::NetworkReference;
use crate::hardware::{metadata::ApplicationMetadata, SystemTimer};

/// Default metadata message if formatting errors occur.
const DEFAULT_METADATA: &str = "{\"message\":\"Truncated: See USB terminal\"}";

/// The telemetry client for reporting telemetry data over MQTT.
pub struct TelemetryClient {
    mqtt: minimq::Minimq<
        'static,
        NetworkReference,
        SystemTimer,
        minimq::broker::NamedBroker<NetworkReference>,
    >,
    prefix: &'static str,
    meta_published: bool,
    metadata: &'static ApplicationMetadata,
}

impl TelemetryClient {
    /// Construct a new telemetry client.
    ///
    /// # Args
    /// * `mqtt` - The MQTT client
    /// * `prefix` - The device prefix to use for MQTT telemetry reporting.
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
        prefix: &'static str,
        metadata: &'static ApplicationMetadata,
    ) -> Self {
        Self {
            mqtt,
            meta_published: false,
            prefix,
            metadata,
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
    pub fn publish<T: Serialize>(&mut self, telemetry: &T) {
        let mut topic: String<128> = self.prefix.try_into().unwrap();
        topic.push_str("/telemetry").unwrap();

        self.mqtt
            .client()
            .publish(Publication::new(&topic, |buf: &mut [u8]| {
                serde_json_core::to_slice(telemetry, buf)
            }))
            .map_err(|e| log::error!("Telemetry publishing error: {:?}", e))
            .ok();
    }

    /// A secondary functionality tugged onto the telemetry client that publishes onto another
    /// `alarm_topic`.
    pub fn publish_alarm(&mut self, alarm_topic: &String<128>, alarm: &bool) {
        self.mqtt
            .client()
            .publish(Publication::new(alarm_topic, |buf: &mut [u8]| {
                serde_json_core::to_slice(alarm, buf)
            }))
            .map_err(|e| log::error!("Alarm publishing error: {:?}", e))
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

            let mut topic: String<128> = self.prefix.try_into().unwrap();
            topic.push_str("/meta").unwrap();

            if mqtt
                .client()
                .publish(Publication::new(&topic, |buf: &mut [u8]| {
                    serde_json_core::to_slice(&metadata, buf)
                }))
                .is_err()
            {
                // Note(unwrap): We can guarantee that this message will be sent because we checked
                // for ability to publish above.
                mqtt.client()
                    .publish(Publication::new(&topic, DEFAULT_METADATA.as_bytes()))
                    .unwrap();
            }

            self.meta_published = true;
        }
    }
}
