"""Calibration values and limits. Constants only."""
CAMERA_OFFSET_T = (0.00, 0.10, 0.02)   # m, camera lens in turret frame; calib.json (calibrate.py) overrides these three
CAMERA_SCALE = (1.0, 1.0)              # gain on camera-frame x, y: corrects a wrong lens field of view
CAMERA_ROT = (0.0, 0.0, 0.0)           # rad, rotation vector: camera axes relative to turret axes
try:
    import json, os
    _cal = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json")))
    CAMERA_OFFSET_T = tuple(_cal["camera_offset_t"])
    CAMERA_SCALE = tuple(_cal.get("camera_scale", CAMERA_SCALE))
    CAMERA_ROT = tuple(_cal.get("camera_rot", CAMERA_ROT))
except (OSError, KeyError, ValueError):
    pass
COLOR_EXPOSURE = 80                    # 0.1 ms, manual RGB exposure
FIT_SAMPLES = 6                        # newest samples kept for the line fit
FIT_MIN_SAMPLES = 4                    # fewer = no prediction
TRACK_JUMP_M = 0.25                    # bigger jump between samples clears the track
TRACK_TIMEOUT_S = 0.20                 # no hand this long clears the track
YAW_MIN_DEG, YAW_MAX_DEG = -35, 55    # real turret pan reach (servo 125..35), step 4
PITCH_MIN_DEG, PITCH_MAX_DEG = -10, 45

# Virtual servo (sim.py); J, E_SAT, TD, GRAV are estimates until fitted (Sim spec 5.4)
SERVO_LAG_S = 0.047                    # s, barrel lag behind a moving command
SERVO_J = (0.015, 0.010)               # kg*m^2 (yaw, pitch)
SERVO_E_SAT_DEG = 3.0
SERVO_TD_S = 0.020
PITCH_GRAV_NM = 0.12
