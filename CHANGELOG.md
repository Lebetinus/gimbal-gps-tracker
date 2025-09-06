# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2025-01-XX

### Added
- **Dual MAVLink Support**: Separate RX (telemetry) and TX (command) links for robust QGC forwarding scenarios
- **Persistent JSON Settings**: Configuration automatically saved to `gimbal_gps_settings.json`
- **Settings Dialog**: GUI settings panel accessible via Application menu
- **Profile Presets**: Quick setup for SITL/WSL, Operation, and Custom configurations
- **Stream Keepalive Hardening**: Automatic SIYI stream re-enable and probe mechanisms
- **Embedded UI Controls**: Compact jog controls integrated into main panel
- **Enhanced Status Display**: More detailed connection and gimbal status information

### Changed
- **UI Layout**: Redesigned for more compact and intuitive control placement
- **Configuration Management**: Moved from hardcoded values to persistent JSON storage
- **MAVLink Architecture**: Split into RX/TX handlers for better QGC integration
- **Error Handling**: Improved robustness in network and device communications

### Fixed
- **SIYI Stream Interruptions**: Automatic recovery from lost gimbal data streams
- **MAVLink Reconnection**: More reliable auto-reconnect for unstable connections
- **Angle Calculation**: Better handling of mount direction and aircraft heading fusion

### Technical Details
- Added `MAVLINK_TX_ADDRESS` configuration option
- Implemented `SettingsStore` class for JSON persistence
- Enhanced `SiyiGimbal` class with keepalive mechanisms
- Redesigned GUI layout with embedded controls
- Added profile-based configuration presets

## [1.0.0] - 2024-XX-XX

### Initial Release
- Basic SIYI ZR10 gimbal communication via UDP
- MAVLink integration for PX4 telemetry
- Target ground intersection calculation
- SBS/ADSB publisher for QGroundControl
- Dynamic loiter tracking functionality
- Manual gimbal control interface
- Basic Tkinter GUI application

### Features
- Single MAVLink connection handling
- Hardcoded configuration values
- Manual gimbal jog controls in separate window
- Basic target calculation and tracking
- SBS publisher for QGC integration
