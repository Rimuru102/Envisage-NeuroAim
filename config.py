"""Calibration values and limits. Constants only."""
CAMERA_OFFSET_T = (0.00, 0.10, 0.02)   # m, camera lens in turret frame
COLOR_EXPOSURE = 80                    # 0.1 ms, manual RGB exposure
FIT_SAMPLES = 15                       # newest samples kept for the line fit
FIT_MIN_SAMPLES = 9                    # fewer = no prediction
TRACK_JUMP_M = 0.25                    # bigger jump between samples clears the track
TRACK_TIMEOUT_S = 0.20                 # no hand this long clears the track
YAW_MIN_DEG, YAW_MAX_DEG = -60, 60
PITCH_MIN_DEG, PITCH_MAX_DEG = -10, 45

# Virtual servo (sim.py); J, E_SAT, TD, GRAV are estimates until fitted (Sim spec 5.4)
SERVO_LAG_S = 0.047                    # s, barrel lag behind a moving command
SERVO_J = (0.015, 0.010)               # kg*m^2 (yaw, pitch)
SERVO_E_SAT_DEG = 3.0
SERVO_TD_S = 0.020
PITCH_GRAV_NM = 0.12
