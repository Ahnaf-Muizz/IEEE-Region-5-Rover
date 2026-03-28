# Mining Mayhem — Rover navigation (IEEE Region 5)

This repository holds **autonomous navigation** code for the **Mining Mayhem** game: camera, AprilTags, purple “Astral Material” detection, map-aware navigation with **time-based odometry** (or encoder odometry if external encoders are added), and a mission script. **Arm / servo pickup** and **CSC manipulation** are left as integration hooks in `main.py` (comments say where to add them).

Use this README when continuing on a **Raspberry Pi 5** (e.g. edit in Firefox via [code-server](https://github.com/coder/code-server) or another web IDE, or SSH + editor). **Cursor chat history does not transfer**; everything important should live here and in `config.py`.

---

## Hardware (do not assume anything else)

| Item | Notes |
|------|--------|
| **Computer** | Raspberry Pi 5 |
| **Camera** | Arducam 1080P day/night USB (USB2.0). **Not** the Pi CSI module unless you switch `CAMERA_BACKEND` in `config.py`. |
| **Camera placement** | Offset **to the right** of the robot centerline so the claw does not block the view. Steering uses `CAMERA_STEERING_OFFSET_PX` in `config.py` (tune on the field). |
| **Drive** | Tank / differential chassis via **L298N dual H-bridge** on Raspberry Pi GPIO. **No IMU.** |
| **Odometry** | With plain L298N (no encoder feedback), use **time-based** odometry (`ODOMETRY_MODE = "time"`). If you later add encoder hardware, switch back to encoder mode and tune related constants. |
| **Manipulator** | Arm + wrist + claw; **3× MG995-class servos** (user part number may read MG99R). **Not wired in this repo** — you will merge your servo code later. |
| **Sensors intentionally not used** | No ultrasonic, no magnetometer (Geodinium is magnetic; Nebulite is not — vision uses **purple color** only). |

---

## Game rules summary (Mining Mayhem)

Official docs: **Game Manual 1 & 2** (IEEE Region 5), **Field Assembly Guide**.

- **Match length:** 3 minutes (`MATCH_DURATION` in `config.py`).
- **AprilTags:** IDs **5** (north wall), **6** (south), **7** (east). **Telemetry** tag in Beacon Mast: ID **0–4** = preferred **Rendezvous Pad** index (south = 0 … north = 4).
- **Astral Material:** Purple ~40 mm icosahedrons (Nebulite / Geodinium). Detection = **HSV** thresholds in `config.py` (tune under venue lighting; IR LEDs may affect purple).
- **Start:** **Start LED** near Landing Site (12×12 in); code waits for brightness step in `start_light.py` (or use manual start per rules).
- **Field:** ~4'×8' plywood base; **±1 inch** setup tolerance per rules. **`FIELD` coordinates in `config.py` are placeholders** — replace with your measured / CAD frame (meters). Heading convention in code: **0° = east, 90° = north, 180° = west, 270° = south**.

**Strategy intent (team plan):** “Plow” rocks into a pile, then pick with arm and score in **Cosmic Shipping Containers** on the telemetry pad. Most game pieces start in the **cave**; this stack focuses on **open-field** navigation and vision — add cave-specific behavior separately if needed.

---

## Software architecture

| File | Role |
|------|------|
| `config.py` | All tunables: camera, field map, odometry, purple HSV, flags. **Start here on a new machine.** |
| `camera_io.py` | USB (`cv2.VideoCapture`) or optional `picamera2`; frames as **BGR**. |
| `april_tags.py` | tag36h11 detection (`pupil-apriltags` or `apriltag`) + role helpers (beacon/boundary, forward, dropoff, cave). |
| `material_detection.py` | Purple blob + `material_steering_error_px()` using steering center (camera offset). |
| `start_light.py` | Start LED wait. |
| `encoder_state.py` | Encoder tick deltas between reads. |
| `odometry.py` | Differential-drive integration (meters per tick, track width). |
| `motor_control.py` | L298N GPIO backend (`RPi.GPIO`), with the same high-level motor API expected by navigation code. |
| `drive_control.py` | Semantic `forward`/`backward`/`left`/`right`/`stop` (maps to your motor API naming). |
| `pose.py` | Field pose `(x, y)` + heading; `drive_distance_signed`, `rotate_to_heading`, etc. |
| `camera_navigation.py` | AprilTag role actions + telemetry search, `drive_toward_coordinate`, material search/approach, optional wall-tag snap. |
| `manipulator.py` | Servo-based rock pickup + container drop sequences (`pickup_rock()`, `drop_into_container()`). |
| `main.py` | CLI modes + full mission orchestration (now calls manipulator pickup/drop hooks). |
| `keyboard_control.py` | L298N keyboard motor test utility (`1/2/3/4/5/q`). |

**Motor mapping:** `drive_control.py` uses direct semantic mapping
(`forward`→`motor.forward`, `backward`→`motor.reverse`,
`left`→`motor.turn_left`, `right`→`motor.turn_right`).

**L298N pin config:** edit `L298N_*` settings in `config.py` to match your wiring
(BCM pin numbering). If a side spins opposite, set `L298N_LEFT_INVERT` or
`L298N_RIGHT_INVERT` to `True`.

**Material color-order:** this stack now uses your known-good behavior by default:
`MATERIAL_COLOR_ORDER = "rgb"` in `config.py` (RGB->HSV for purple mask).

**Camera mount offset compensation:** pose snaps from AprilTags are corrected from
camera position to rover center using:
- `CAMERA_OFFSET_X_M = 0.07131` (forward of rover center)
- `CAMERA_OFFSET_Y_M = 0.05019` (left of rover center)
Update these in `config.py` if your mount changes.

---

## Raspberry Pi 5 setup (after `git clone`)

```bash
cd rover   # or your clone path
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

**AprilTags on Pi (pick one):**

```bash
pip install pupil-apriltags
# or: pip install apriltag
```

**USB camera:** Ensure the device exists (`ls /dev/video*`). User may need `video` group: `sudo usermod -aG video $USER` then re-login.

**OpenCV:** `opencv-python` wheels usually work on Pi 5; if not, use `pip install opencv-python-headless` for headless runs.

**Optional CSI camera:** Set `CAMERA_BACKEND = "picamera2"` in `config.py` and install system `picamera2` (not needed for Arducam USB).

---

## How to run (quick reference)

From the `rover` directory with venv activated:

```bash
# Full mission (needs real motors + camera + tags on field)
python main.py --mode full

# Subsystems
python main.py --mode camera --show
python main.py --mode tags --show
python main.py --mode material --show
python main.py --mode start_light
python main.py --mode odometry          # no camera; short drive/pose test

# Per-module tests (see docstring in main.py for full list)
python camera_io.py
python april_tags.py
python material_detection.py
python start_light.py
python odometry.py
python encoder_state.py
python pose.py status
python pose.py drive --yes --meters 0.1
python drive_control.py info
python camera_navigation.py preview
python camera_navigation.py preview --show-tags
```

Motion commands that can move the robot require **`--yes`** where documented.

---

## Continuing in Firefox on the Pi

1. Push this repo to **GitHub** from your PC.
2. On the Pi: `git clone <your-repo-url>` and open the folder in a **browser-based editor** (e.g. install **code-server**, open `http://127.0.0.1:8080` in Firefox), or use **GitHub’s web UI** only for browsing and run commands in a terminal.
3. Re-read **`config.py`** and this **README** first — they carry the design decisions that would otherwise live in chat.

---

## Checklist before competition

- [ ] Wire L298N pins to match `L298N_*` in `config.py` and verify motion directions.
- [ ] Calibrate **`METERS_PER_SECOND`** and **`DEGREES_PER_SECOND`** for time-based navigation (`python main.py --mode odometry` + tape measure).
- [ ] Fill **`FIELD`** in `config.py` from your taped / CAD field (and verify start heading when snapping to mast after telemetry).
- [ ] Tune **`CAMERA_STEERING_OFFSET_PX`** and **`PURPLE_*`** HSV with venue lighting.
- [ ] Install **`pupil-apriltags`** (or `apriltag`) on the Pi and verify `python april_tags.py`.
- [ ] Decide **`USE_WALL_TAG_POSE_CORRECTION`**: `False` = dead-reckoning + map only; `True` = correct drift when wall tags visible.
- [ ] Tune manipulator servo angles/timing in `config.py` on real hardware.

---

## License

Add a `LICENSE` file if you open-source; otherwise default copyright applies.
