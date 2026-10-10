# NeuroAim: hand-tracking virtual turret

A camera watches for one human hand, estimates its 3D palm position, predicts where the hand will be a moment later, and aims a **virtual** 2-axis turret (yaw and pitch) at it. The turret is simulated with realistic servo behavior. The program reports how far the barrel line misses the palm. Nothing fires and no hardware is driven yet.

This is phase 1 of the project described in two design specs in this directory:
- `Hand-Tracking Nerf Turret — Architecture Spec v1.md`: the full system (vision, ballistics, fire gate, firmware, safety).
- `Virtual Turret Simulator — Architecture & Design Spec v1.md`: the simulator that replaces the Arduino and servos for this phase.

`log_spec.md` is a short map of what is where and the current status.

## Features

- Hand detection and palm center with MediaPipe, plus metric 3D position from depth.
- Straight-line motion fit over the last 15 samples, with lead compensation for system delay.
- Simulated DS3218MG servos (lag, inertia, deadband, gravity sag), stepped at 1 ms.
- Miss metric: distance from the palm to the barrel line, measured at the frame's capture time, so latency shows up as error.
- Debug view: palm box, barrel crosshair (green when the miss is under 3 cm, red otherwise), a yellow line and ring for the predicted hand position, and a 3D side panel showing the turret moving.
- CSV log of every frame, and a summary on exit.

## Run it

Python 3.11 only. The pinned `mediapipe` and `pyrealsense2` do not support 3.14, which is the system Python. Use the venv in `.venv`.

| Goal | Command |
| --- | --- |
| Self-checks (no camera) | `python aim.py`, `python vision.py`, `python sim.py`, `python view3d.py` |
| Offline demo | `python make_demo.py`, then `python main.py --video demo.mp4 --out out.mp4` (open `out.mp4`) |
| Webcam, fixed depth | `python main.py --webcam [index]`, `q` quits |
| RealSense D435 | `python main.py`, `q` quits |

Use `.venv/bin/python` on WSL/Linux. On Windows use `.venv\Scripts\python.exe`. To recreate it: `uv venv --python 3.11 .venv`, then `uv pip install numpy==1.26.4 mediapipe==0.10.14 opencv-python==4.10.0.84` (add `pyrealsense2==2.55.1.6486` only if a RealSense is attached).

**Two copies exist.** WSL cannot see the laptop webcam, so webcam runs happen on Windows in the Windows copy at `C:\Users\revet\OneDrive\Projects\NeuroAim` (moved there from `C:\Users\revet\NeuroAim`). Its `.venv` survived the move and works (checked with the offline demo). If it ever breaks, recreate it. A moved venv can break because venvs store absolute paths. OneDrive may try to sync the venv's thousands of files, so exclude it if that is a problem. `~/NeuroAim` (WSL) is the main copy. After editing either, copy changed files to the other. The `.py` files were identical when this README was written.

Models and assets: `models/hand_landmarker.task` (downloaded from Google's MediaPipe storage) and `assets/hand.jpg` (sample hand image for the demo).

Output: `sim_log.csv` (columns `t, x, y, z, yaw_cmd, pitch_cmd, yaw, pitch, miss_m`) and a printed summary: frames, hand-seen rate, median and 95th-percentile miss in cm, median latency, median loop time.

## How it works

Per camera frame, `main.py` does:

1. `Camera.read()` returns color, depth in meters and a host-clock capture time.
2. `Vision.process()` finds the hand. Palm center is the mean of landmarks 0, 5, 9, 13 and 17. Depth is the median of valid values in 5x5 windows around those points. The pixel is back-projected to a camera-frame 3D point.
3. `Tracker.update()` converts the point to the turret frame and stores it. `Tracker.fit()` fits a least-squares line to the newest 15 samples. It returns `(p_now, v, t_last, rms, n)`, or None under 9 samples.
4. `aim_point(p_now, v, delay)` pushes the point forward by `delay = (now - t_last) + SERVO_LAG_S`. It returns `(yaw_deg, pitch_deg, predicted_point)`.
5. `main.py` writes the protocol line `A,<yaw_cdeg>,<pitch_cdeg>,0` to `SimTurret.write()`. `SimTurret` stands in for the Arduino serial port. The last field, `fire`, is always 0.
6. `SimTurret` clamps the angles, rounds to whole microseconds like the firmware, latches the setpoint every 20 ms, and runs the servo physics up to the current time.
7. The miss is computed from the barrel angles at the frame's capture time and the palm position. The overlays and the 3D panel are drawn.

**Frames.** Camera frame: +X right, +Y down, +Z forward, origin at the lens. Turret frame: +X right, +Y up, +Z forward, origin at the pivot. `aim.to_turret()` converts between them using `CAMERA_OFFSET_T`. All tracking and aiming math is in the turret frame. Angles inside `sim.py` are radians. Everywhere else they are degrees. Index 0 is yaw and index 1 is pitch.

**Lost hand.** `Tracker.update(None, t)` clears the track after `TRACK_TIMEOUT_S` (0.20 s). Before that, the old fit keeps being extrapolated (coasting). With no fit, `main.py` keeps sending the last angles.

## Files

| File | Responsibility |
| --- | --- |
| `config.py` | All calibration constants and limits. Constants only. |
| `camera.py` | `Camera` (RealSense, aligned color and depth, manual exposure, host-clock timestamps). `VideoCamera` (video file or webcam index, flat depth plane at 2 m, D435-like intrinsics scaled from frame width). Both expose `read()`, `.intr` and `.clock()`. |
| `vision.py` | `Hand` dataclass, `palm_depth()`, `Vision.process() -> Hand \| None`. |
| `aim.py` | `to_turret()`, `Tracker`, `aim_point()`. |
| `sim.py` | `SimTurret`, `barrel_dir`, `miss_m`, `to_px`, and the main self-check that tunes `SERVO_LAG_S`. |
| `view3d.py` | `render()` draws the 3D side panel with OpenCV only. |
| `main.py` | The loop, CLI, overlays, CSV log, summary. |
| `make_demo.py` | Builds `demo.mp4`: the sample hand sliding at 0.4 m/s. |
| `requirements.txt` | Pinned deps (no pyserial: serial is not used yet). |
| `log_spec.md` | Short file map and status. |

Each file with real logic has a runnable `__main__` self-check with asserts. There is no test framework.

## Constants worth knowing (`config.py`)

- `SERVO_LAG_S = 0.047`: how late the barrel trails a moving command. Printed by `python sim.py`. If the crosshair trails the hand live, raise it. If it leads, lower it.
- `SERVO_J`, `SERVO_E_SAT_DEG`, `SERVO_TD_S`, `PITCH_GRAV_NM`: servo estimates, to be fitted to a real step test (Sim spec section 5.4).
- `CAMERA_OFFSET_T = (0, 0.10, 0.02)`: camera lens position relative to the pivot, to be measured on real hardware.
- `FIT_SAMPLES = 15`, `FIT_MIN_SAMPLES = 9`, `TRACK_JUMP_M`, `TRACK_TIMEOUT_S`, the yaw and pitch limits.

## Deviations from the specs

- `Vision.process` returns the hand only, not `(hand, faces)`. The face detector exists only for the fire lockout and is not built.
- No `--sim` flag. The sim is the only mode, because serial is not built.
- `sim.py` `angles_at` has a guard for empty history. The spec version crashes if the first write happens at the sim's start time.
- `aim_point` returns a third value, the predicted point. The sim self-check and the overlays use it.
- `to_px` replaced the spec's `barrel_px`. It projects any turret-frame point to a camera pixel.
- `main.py` uses `cam.clock()` instead of `time.time()`, so offline runs use video time and are deterministic.

## Current status

- Passing: the `aim.py`, `vision.py`, `sim.py` and `view3d.py` self-checks. On the demo video the hand is seen in 100% of frames and the median miss is 1.1 cm. The 95th-percentile miss (about 43 cm) comes only from start-up: nothing is predicted before 9 samples, and the servo starts at 0°.
- Not verified: the RealSense `Camera` class and its live depth. There is no RealSense here, and `pyrealsense2` needs `libusb-1.0-0` and USB passthrough (`usbipd`) in WSL. The webcam path was only smoke-tested and has not been checked on a live hand.
- Webcam and demo runs use a fake flat 2 m depth. They test detection, tracking, servo response and the miss metric, but not depth accuracy or distance. Moving toward or away from the camera is not seen.

## What is left

1. Run live on the RealSense and check Sim spec section 5.2: loop time under 20 ms, static miss at 1, 2 and 3 m, a moving hand, covered-hand cases.
2. Tune `SERVO_LAG_S` from live results. Fit the servo constants once hardware exists.
3. Later phases, per the Nerf Turret spec: `solve_aim` ballistics, `fire_ok` gate, face detector and lockout, serial and Arduino firmware, ARM switch, dead-man button and watchdog. Safety rules in section 0 of that spec are mandatory before any real firing.

## Working conventions

- Use the ponytail approach: smallest solution that works, standard library and existing code first, no speculative abstractions. Ask before making a critical decision or changing the spec.
- Every added behavior should leave one runnable check behind (asserts in `__main__`).
- Keep the two copies (WSL and Windows) in sync, and update `log_spec.md` when files change.
