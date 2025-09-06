# Gimbal GPS Tracker (SIYI ZR10 → PX4/QGC)

A desktop application (Tkinter) that reads **SIYI ZR10** gimbal angles over UDP, fuses them with **PX4** telemetry (MAVLink), computes the camera ground intersection/target point, and provides:

- **SBS/ADSB** publishing (TCP/30003) for QGroundControl visualization
- **Dynamic loiter tracking** by periodically repositioning aircraft to circle the viewed point
- **Persistent settings** with JSON configuration and profile presets
- **Dual MAVLink links** (RX/TX) for robust QGC forwarding scenarios

## 🎯 Core Features

- **SIYI ZR10** UDP protocol (attitude/config, center, jog) with stream keepalive
- **Mount-aware** angle correction (Normal/UpsideDown)
- **Robust MAVLink** client with auto-reconnect and dual RX/TX links
- **Target calculation** from yaw/pitch + AGL with ground intersection math
- **Sky/horizon fallback** (fixed distance projection when pointing up)
- **SBS/ADSB publisher** for QGC target display
- **Dynamic Loiter Tracker** with configurable radius/interval/movement thresholds
- **Settings persistence** in JSON with profile presets (SITL/Operation/Custom)
- **Clean Tkinter GUI** with embedded controls and settings dialog

## 📋 Requirements

### Software
- **Python 3.10+**
- **OS**: Windows 10/11, Linux (including WSL with X server for GUI)

### Dependencies
```bash
pip install pymavlink
```

**Tkinter** ships with most Python builds on Windows/macOS. On Debian/Ubuntu:
```bash
sudo apt install python3-tk
```

### Hardware/Network
- **SIYI ZR10** reachable at `192.168.144.25:37260` (configurable)
- **PX4/MAVLink** accessible (e.g., via MAVLink Router or QGC forwarding)
- **QGroundControl** can connect to SBS at TCP port 30003

## 🚀 Installation & Setup

### 1. Clone and Setup Environment
```bash
git clone https://github.com/Lebetinus/gimbal-gps-tracker.git
cd gimbal-gps-tracker
python -m venv .venv

# Windows
.venv\Scripts\activate
# Linux/macOS  
source .venv/bin/activate

pip install pymavlink
```

### 2. Linux/WSL GUI Setup
```bash
sudo apt update && sudo apt install -y python3-tk
```
**WSL GUI**: Use Windows 11 WSLg or an X server (GWSL/X410/VcXsrv)

### 3. Run Application
```bash
python gimbal_gps_ui.py
```

**Or use provided launchers:**
- Windows: `start_gimbal.cmd`
- Linux/WSL: `start_gimbal.sh`

## ⚙️ Configuration

Settings are automatically saved to `gimbal_gps_settings.json`. Access via **Application → Settings** menu.

### Profile Presets

| Profile | RX Link | TX Link | Use Case |
|---------|---------|---------|----------|
| **SITL/WSL** | `udpin:127.0.0.1:14540` | `udpout:127.0.0.1:14550` | QGC forwarding setup |
| **Operation** | `udpin:0.0.0.0:14540` | *(same link)* | Field operations |
| **Custom** | *(user defined)* | *(user defined)* | Manual configuration |

### Key Settings
```json
{
  "SIYI_IP": "192.168.144.25",
  "SIYI_PORT": 37260,
  "SBS_BIND": "0.0.0.0", 
  "SBS_PORT": 30003,
  "MAVLINK_ADDRESS": "udpin:127.0.0.1:14540",
  "MAVLINK_TX_ADDRESS": "udpout:127.0.0.1:14550"
}
```

**Notes:**
- `MAVLINK_ADDRESS`: RX link for telemetry listening
- `MAVLINK_TX_ADDRESS`: TX link for commands (empty = use RX link)
- `SBS_PORT`: Leave at 30003 for QGC compatibility
- `MAX_DISTANCE_KM`: Sky/horizon projection limit

## 🎮 User Interface

### Main Panels
1. **MAVLink Status**: Connection status + live position/heading
2. **SIYI Gimbal**: Connect, mount info, embedded jog controls, center
3. **Target Calculation**: Live target coordinates + distance + calculation notes
4. **Dynamic Loiter**: Radius/interval/movement sliders, tracking controls
5. **System Controls**: Calculation rate, SBS publisher toggle

### Control Features
- **Embedded Jog**: Press & hold directional buttons with speed slider
- **Center**: Reset gimbal to center position
- **Dynamic Tracking**: Auto-reposition aircraft to follow target
- **Single GoTo**: One-time loiter at current target
- **Return to Mission**: Switch back to mission mode

## 🔧 Architecture

### Core Classes
- **`SiyiGimbal`**: UDP protocol handler with stream keepalive and angle correction
- **`MAVLinkHandler`**: Dual RX/TX links with auto-reconnect for PX4 communication  
- **`TargetCalculator`**: Ground intersection math from gimbal angles + AGL
- **`SBSPublisher`**: Minimal SBS/ADSB emitter for QGC visualization
- **`DynamicTracker`**: Periodic reposition logic with movement thresholds
- **`SettingsStore`**: JSON persistence with profile management
- **`GimbalGPSApp`**: Tkinter GUI coordinator

### Data Flow
1. **MAVLink** → Aircraft position/attitude
2. **SIYI UDP** → Raw gimbal angles  
3. **Angle Fusion** → Corrected world angles
4. **Ray Casting** → Ground intersection target
5. **SBS Output** → QGC visualization
6. **Tracking Logic** → Aircraft repositioning

## 🔗 QGroundControl Integration

### Setup Steps
1. Start this app and enable **SBS Publisher** (port 30003)
2. In QGC, add a **TCP link** to `127.0.0.1:30003` (or host IP)
3. When gimbal points to ground, target appears as aircraft track

**Troubleshooting:**
- If QGC doesn't show SBS data, disable/enable the TCP link once
- Ensure QGC connects as **client** to this app (server)
- Check firewall settings for TCP/30003

## 🐛 Troubleshooting

### Common Issues

**No GUI Window (Linux/WSL)**
```bash
sudo apt install python3-tk
# Ensure X server is running (WSLg or GWSL/X410/VcXsrv)
```

**SBS Not Visible in QGC**
- Verify QGC connects as TCP client to this app
- Check firewall blocking TCP/30003
- Try disable/enable QGC TCP link

**No Target Computed**
- Requires AGL (relative altitude) > 0
- Gimbal must point downward (pitch > 0°)
- Above horizon → sky projection only

**Loiter Not Working**
- Ensure PX4 accepts `DO_REPOSITION` commands
- Check compatible flight mode
- Verify `NAV_LOITER_RAD` parameter

**SIYI Connection Issues**
- Verify gimbal IP/port in settings
- Check network connectivity to `192.168.144.25:37260`
- Mount direction affects angle correction

## 🔄 Recent Updates (v1.1)

### Major Changes
- ✅ **Dual MAVLink support** (separate RX/TX links)
- ✅ **Persistent JSON settings** with profile presets  
- ✅ **Stream keepalive hardening** for SIYI reliability
- ✅ **Embedded UI controls** for compact layout
- ✅ **Settings dialog** with quick profile switching

### Migration Notes
If upgrading from v1.0, your settings will be migrated to `gimbal_gps_settings.json` automatically.

## 🛠️ Development

### Debug Setup
- Run from VS Code with Python debug configuration
- For SITL: Forward MAVLink to `udp:127.0.0.1:14540`
- Log SBS output: `nc 127.0.0.1 30003`

### Code Structure
```
gimbal_gps_ui.py          # Main application
gimbal_gps_settings.json  # Runtime settings (auto-created)
start_gimbal.cmd          # Windows launcher
start_gimbal.sh           # Linux launcher  
```

## 📄 License

This project is released under the **MIT License**. See `LICENSE` file for details.

## 🤝 Contributing

Issues and pull requests welcome! Please ensure:
- Python 3.10+ compatibility
- Cross-platform support (Windows/Linux)
- Documentation updates for new features

---

**Happy Tracking! 🎯✈️**
