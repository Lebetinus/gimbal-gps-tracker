# gimbal-gps-tracker
Gimbal GPS Tracker (SIYI ZR10 → PX4/QGC)

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

Install requirements
pip install -r requirements.txt
(Linux/WSL only) Ensure Tk is present
sudo apt update && sudo apt install -y python3-tk

WSL GUI: use Windows 11 WSLg or an X server (GWSL/X410/VcXsrv) before running the app.

4) Run the app
python gimbal_gps_ui_fixed.py
