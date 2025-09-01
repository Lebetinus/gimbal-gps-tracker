# Gimbal GPS Tracker (SIYI ZR10 → PX4/QGC)

A minimal desktop app (Tkinter) that reads **SIYI ZR10** gimbal angles over UDP, fuses them with **PX4** telemetry (MAVLink), computes the camera **ground intersection / target point**, and can:

- publish the target as **SBS/ADSB (TCP/30003)** for **QGroundControl** to visualize, and
- drive **dynamic loiter tracking** by periodically **DO_REPOSITION** to keep the aircraft circling the viewed point.

> Core file: `gimbal_gps_ui_fixed.py`

---

## ✨ Features
- **SIYI ZR10** UDP protocol (attitude/config, center, jog)
- **Mount-aware** angle correction (Normal / UpsideDown)
- Robust **MAVLink** client (auto-reconnect) for PX4
- Target **ground-intersection** math from yaw/pitch + AGL
- Optional sky/horizon fallback (fixed distance projection)
- **SBS/ADSB** publisher (TCP 30003) for QGC display
- **Dynamic Loiter Tracker** (radius / update interval / min movement)
- Clean **Tkinter GUI** with manual jog window

---

## 📦 Requirements
- Python **3.10+**
- OS: Windows 10/11, Linux (incl. WSL – needs X server for GUI)
- Dependencies (pip):
  - `pymavlink`

> **Tkinter** ships with most Python builds on Windows/macOS. On Debian/Ubuntu install via `sudo apt install python3-tk`.

---

## 🔌 External Systems
- **SIYI ZR10** reachable at `192.168.144.25:37260` (default in code)
- **PX4 / MAVLink** accessible at `udp:127.0.0.1:14540` (default in code) — e.g. via MAVLink Router or QGC forwarding
- **QGroundControl** can connect to SBS at `TCP 30003` (this app listens for a client; QGC should connect)

---

## 🚀 Quick Start

### 1) Create & activate venv
```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux/macOS
source .venv/bin/activate
```

### 2) Install requirements
```bash
pip install -r requirements.txt
```

### 3) (Linux/WSL only) Ensure Tk is present
```bash
sudo apt update && sudo apt install -y python3-tk
```
> **WSL GUI**: use Windows 11 WSLg or an X server (GWSL/X410/VcXsrv) before running the app.

### 4) Run the app
```bash
python gimbal_gps_ui_fixed.py
```

---

## ⚙️ Configuration
Defaults live in class `Config` inside the main file. Adjust and re-run.

```python
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
```

**Notes**
- `MAVLINK_ADDRESS`: point to your PX4 link (e.g., `udp:0.0.0.0:14540`, `tcp:127.0.0.1:5760`, etc.)
- `SBS_PORT`: leave at `30003` and let **QGC** connect to this app
- `MAX_DISTANCE_KM`: cap for sky/horizon projection

---

## 🖥️ GUI Overview
- **MAVLink Status**: connection + live lat/lon/AGL/heading
- **SIYI Panel**: connect, mount/mode, jog (press & hold), center
- **Target Calculation**: live target lat/lon + distance + note (Ground / Sky/Horizon)
- **Dynamic Loiter**: sliders for **Radius**, **Update Interval**, **Min Movement**; Start/Stop; Single **GoTo**; **Return to Mission**
- **System Controls**: calc rate Hz, **SBS publisher** toggle
- **Manual Control Window**: separate jog with speed

---

## 🧠 How It Works (Architecture)
- `SiyiGimbal`: builds/pars\-es SIYI UDP frames, tracks yaw/pitch/roll, mount/mode; exposes `get_corrected_angles(aircraft_heading)`
- `MAVLinkHandler`: PX4 link, `GLOBAL_POSITION_INT` & `ATTITUDE`, helpers for `DO_REPOSITION`, `Auto/Loiter` mode
- `TargetCalculator`: converts yaw/pitch + AGL + heading → NED ray → ground intersection (geodesic to lat/lon)
- `SBSPublisher`: minimalist SBS/ADSB line emitter for QGC (`MSG,1` + `MSG,3`)
- `DynamicTracker`: periodically compares last loiter center with new target; repositions if moved sufficiently
- `GimbalGPSApp`: Tk GUI glue; timers, workers, and display updates

---

## 🧭 QGroundControl Setup (SBS)
1. Start this app and **enable** *SBS Publisher* (port 30003)
2. In QGC, add a **TCP** link to **127.0.0.1:30003** (or the host IP)
3. When the gimbal points to ground, you should see a target track/point

> If QGC requires a restart or re-toggle to see SBS, disable/enable the link in QGC once.

---

## 🛠️ Troubleshooting
- **Tkinter error / no window (WSL/Linux)**: Install `python3-tk`, ensure a GUI/X server is active.
- **SBS shows nothing in QGC**: Make sure QGC is the **client** connecting to this app (server). Firewalls can block TCP/30003.
- **No target computed**: The app needs **AGL** (relative altitude) and a **downward** pitch (> 0° in this convention). Above horizon → horizon/sky projection only.
- **Loiter stays in place**: Ensure PX4 accepts `DO_REPOSITION` and you’re in a compatible mode. Radius is set via `NAV_LOITER_RAD` param.
- **SIYI mount inverted**: Toggle the hardware mount direction or ensure the app reads `UpsideDown` and auto-corrects.

---

## 🧪 Developer Tips
- Run from VS Code; set up a Python debug config
- For simulation: forward SITL MAVLink to `udp:127.0.0.1:14540`
- Log SBS/TCP with `nc` to verify emitted lines

---

## 📄 License
This project is released under the **MIT License**. See `LICENSE`.

---

## 📜 Acknowledgements
- SIYI ZR10 docs & community notes
- PX4/MAVLink ecosystem and QGroundControl
```

---

## 2) `requirements.txt`
```txt
pymavlink>=2.4.40
```

> On Debian/Ubuntu also install the GUI binding: `sudo apt install python3-tk`.

---

## 3) `.gitignore`
```gitignore
# Python
__pycache__/
*.py[cod]
*.egg-info/
*.egg
*.pyo
*.pyd
.env
.venv/
venv/
env/
*.log
logs/
*.sqlite
.ipynb_checkpoints/

# Build / dist
build/
dist/
*.spec

# OS
.DS_Store
Thumbs.db

# VS Code
.vscode/
.history/

# AI / outputs
runs/
outputs/
checkpoints/

# Misc
*.bak
*.tmp
```

---

## 4) `LICENSE` (MIT)
```text
MIT License

Copyright (c) 2025 <Your Name>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 5) `CHANGELOG.md` (optional)
```markdown
# Changelog
All notable changes to this project will be documented in this file.

## [0.1.0] - 2025-09-01
### Added
- Initial public release of `gimbal_gps_ui_fixed.py`
- SIYI ZR10 UDP handler, PX4 MAVLink client, target calculator
- SBS publisher and dynamic loiter tracking GUI
```

---

## 6) Suggested Repo Structure
```
.
├─ gimbal_gps_ui_fixed.py
├─ requirements.txt
├─ README.md
├─ LICENSE
├─ CHANGELOG.md  (optional)
└─ .gitignore
```

---

## 7) Browser Upload How‑To (Recap)
1. GitHub → **New repository** → name it → *Create*
2. **Add file → Upload files** → drag all files above
3. Commit → Done. Then edit `README.md` online if needed.

