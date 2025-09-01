Düzeltilmiş README.md (tam içerik)
# 🎯 Gimbal GPS Tracker (SIYI ZR10 → PX4 / QGroundControl)

**Purpose:**  
Reads **SIYI ZR10 gimbal** angles over UDP, combines them with **PX4 telemetry (MAVLink)**, computes the **ground target position**, and:
- publishes it in **SBS/ADSB** format for QGroundControl, and
- can drive **Dynamic Loiter Tracking** (periodic DO_REPOSITION).

---

## ✨ Features
- 📡 **SIYI ZR10 UDP** protocol (attitude/config, center, jog)
- 🔄 **Mount-aware** angle correction (Normal / UpsideDown)
- 🛰️ Robust **MAVLink** client (auto-reconnect)
- 📍 Ground-intersection math (yaw + pitch + AGL → Lat/Lon)
- 🖥️ **Tkinter GUI** (gimbal control + tracking panel)
- 📢 **SBS/ADSB Publisher** (TCP/30003) → QGC
- 🔁 **Dynamic Loiter Tracking** (radius / update interval / min movement)

---

## 📦 Requirements
- Python **3.10+**
- Packages:
  - `pymavlink`
- Linux/WSL extra dependency:
  ```bash
  sudo apt install python3-tk


External systems:

SIYI ZR10 gimbal (192.168.144.25:37260)

PX4 MAVLink (udp:127.0.0.1:14540)

QGroundControl (TCP → 30003)

🚀 Quick Start
1) Create & activate venv
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux/macOS
source .venv/bin/activate

2) Install dependencies
pip install -r requirements.txt

3) (Linux/WSL only) Install Tkinter
sudo apt update && sudo apt install -y python3-tk

4) Run the app
python gimbal_gps_ui.py

⚙️ Configuration

Defaults live in the Config class:

class Config:
    SIYI_IP = "192.168.144.25"
    SIYI_PORT = 37260
    SBS_PORT = 30003
    MAVLINK_ADDRESS = 'udp:127.0.0.1:14540'

    GUI_UPDATE_MS = 50
    SBS_UPDATE_S = 0.2
    ATTITUDE_REQUEST_MS = 100
    TRACKING_UPDATE_S = 1.0

    DEFAULT_LOITER_RADIUS = 500.0
    MIN_MOVEMENT_THRESHOLD = 10.0
    MAX_DISTANCE_KM = 5.0

    CALC_THROTTLE_S = 0.1
    ANGLE_CHANGE_THRESHOLD = 0.5


Notes

MAVLINK_ADDRESS: point to your PX4 link (e.g., udp:0.0.0.0:14540, tcp:127.0.0.1:5760)

SBS_PORT: QGC should connect to this app on TCP 30003

MAX_DISTANCE_KM: cap for horizon/sky projection

🖥️ GUI Overview

MAVLink Status: connection + live lat/lon/AGL/heading

Gimbal Panel: connect, mount/mode, press-and-hold jog, center

Target Display: target lat/lon + distance + note

Dynamic Loiter: radius / update interval / min movement; Start/Stop; single GoTo; Return to Mission

System Controls: calc-rate Hz, SBS publisher toggle

Manual Control Window: separate jog UI with speed

🧭 QGroundControl Integration

Run the app and enable SBS Publisher (port 30003).

In QGC: Comm Links → Add → TCP → 127.0.0.1:30003 (or host IP).

When the gimbal points downward, the target will appear on the map.

🛠️ Troubleshooting

Tkinter error / no window (Linux/WSL): install python3-tk.

SBS not visible in QGC: disable/re-enable the TCP link in QGC; check firewall.

No target calculated: AGL must be > 0 and gimbal must look below horizon.

Loiter stuck: ensure PX4 accepts DO_REPOSITION; NAV_LOITER_RAD is set.

📄 License

Released under the MIT License. See LICENSE.
