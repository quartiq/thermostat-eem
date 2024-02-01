
# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased](https://github.com/quartiq/thermostat-eem/compare/v0.1.0...master)

### Changed
* A DNS name for the MQTT broker of `mqtt` is now used instead of a hard-coded IP address.
* A static IP can be assigned to Thermostat using the `STATIC_IP` environment variable at build
  time. If one is not specified, DHCP is used automatically.
* Miniconf versions have been updated to v0.9.0
