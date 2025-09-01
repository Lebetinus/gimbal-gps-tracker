# 🎯 Gimbal GPS Tracker (SIYI ZR10 → PX4 / QGroundControl)

📌 **Purpose:**  
Reads **SIYI ZR10 gimbal** angles over UDP,  
combines them with **PX4 telemetry (MAVLink)**,  
calculates the **ground target position**,  
and publishes it in **SBS/ADSB** format for QGroundControl,  
with optional **Dynamic Loiter Tracking** via PX4.

---

## ✨ Features
- 📡 **SIYI ZR10 UDP** protocol support (attitude, config, center, jog)
- 🔄 **Mount-aware** angle correction (Normal / UpsideDown)
- 🛰️ Robust **MAVLink** client (auto reconnect to PX4)
- 📍 **Ground intersection** math (yaw + pitch + AGL → Lat/Lon)
- 🖥️ **Tkinter GUI** (gimbal control + tracking panel)
- 📢 **SBS/ADSB Publisher** (TCP/30003) → QGC integration
- 🔁 **Dynamic Loiter Tracking** (radius, update interval, min movement)

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
1️⃣ Create and activate a virtual environment
bash
Copy code
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux/macOS
source .venv/bin/activate
2️⃣ Install dependencies
bash
Copy code
pip install -r requirements.txt
3️⃣ (Linux/WSL only) Install Tkinter
bash
Copy code
sudo apt update && sudo apt install -y python3-tk
4️⃣ Run the application
bash
Copy code
python gimbal_gps_ui.py
⚙️ Configuration
Default parameters are defined in the Config class:

python
Copy code
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
🖥️ GUI Overview
MAVLink Status: connection + telemetry display

Gimbal Panel: connect, mount/mode info, jog controls, center button

Target Display: calculated target position + distance

Dynamic Loiter: radius, update interval, min movement sliders; start/stop; single GoTo; return to mission

System Controls: calc rate Hz, SBS publisher toggle

Manual Control Window: extra jog UI

🧭 QGroundControl Integration
Run the app and enable SBS Publisher (port 30003)

In QGC → Comm Links → Add → TCP → 127.0.0.1:30003

When the gimbal points downward, the target will appear in QGC.

🛠️ Troubleshooting
❌ Tkinter error / no window:
→ sudo apt install python3-tk (Linux/WSL)

❌ SBS not visible in QGC:
→ disable/re-enable the TCP link in QGC

❌ No target calculated:
→ aircraft AGL altitude must be > 0 and gimbal must point downward

❌ Loiter stuck in place:
→ ensure PX4 accepts DO_REPOSITION and NAV_LOITER_RAD is set

📄 License
This project is licensed under the MIT License.
See the LICENSE file for details.
