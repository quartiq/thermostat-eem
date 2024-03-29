
# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [v0.2.0](https://github.com/quartiq/thermostat-eem/compare/v0.1.0...v0.2.0)

### Changed
* A DNS name for the MQTT broker of `mqtt` is now used instead of a hard-coded IP address.
* A static IP can be assigned to Thermostat using the `STATIC_IP` environment variable at build
  time. If one is not specified, DHCP is used automatically.
* Miniconf versions have been updated to v0.9.0
* Configuration is now done in terms of PID parameters
* Settings tree has changed its layout

### Added

* Application metadata is now published to the `<prefix>/meta` topic on MQTT connections
* Data streaming
