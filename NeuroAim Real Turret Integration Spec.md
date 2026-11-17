# NeuroAim Real Turret Integration Spec

How to replace the virtual turret in Envisage-NeuroAim with the real ESP32 + PCA9685 pan/tilt turret · 4 Nov 2026

Source read: `README.md`, `main.py`, `aim.py`, `config.py` from github.com/Rimuru102/Envisage-NeuroAim (main). `sim.py`, `vision.py` and `camera.py` were not opened, so anything that depends on them is flagged.

## 1. Goal and scope

Drive the real pan/tilt turret from the existing hand-tracking pipeline, with the smallest possible change to the repo. The vision, tracking and aiming code stays as it is. Only the output side changes: the angles that go to `SimTurret` also go to the ESP32.

Out of scope: any firing mechanism. The `fire` field stays 0 in Python and is ignored by the firmware.

## 2. Current setup

| Part | Detail |
| --- | --- |
| Laptop | Windows, Python 3.11 venv (`.venv\Scripts\python.exe`) |
| Microcontroller | ESP32 DevKit V1, Arduino IDE board "DOIT ESP32 DEVKIT V1", CP2102 USB-UART (Silicon Labs driver), appears as a COM port |
| Servo driver | PCA9685 at I2C address 0x40. ESP32 3V3 to VCC, GND to GND, GPIO21 to SDA, GPIO22 to SCL |
| Servo power | Separate PSU into the PCA9685 `V+` / `GND` screw terminal. The ESP32 never powers the servos |
| Servos | Pan on channel 0, tilt on channel 1 (model and range to be confirmed, see section 9) |
| Camera | Laptop webcam (repo assumes a fixed 2 m depth) or a RealSense D435 if attached |
| Centered pose | **Pan = 0, tilt = 90** |

Status: wiring is done. The ESP32 currently runs a test sketch that accepts `X<pan>Y<tilt>`. That protocol is replaced by section 5.

## 3. What the repo does today

- Per frame, `main.py` finds the hand, fits a line to the last 15 samples and calls `aim_point(...)`, which returns `yaw` and `pitch` in degrees.
- It then writes one line to the virtual turret: `ser.write(f"A,{round(yaw*100)},{round(pitch*100)},0\n".encode())`. Format: `A,<yaw_cdeg>,<pitch_cdeg>,<fire>`.
- Angle convention (turret frame): X right, Y up, Z forward. **Yaw 0 and pitch 0 mean straight ahead and level.** Positive yaw is to the right, positive pitch is up.
- `config.py` limits: yaw -60 to 60, pitch -10 to 45.
- `SimTurret` is the stand-in for the Arduino serial port. It also provides `ser.th` and `ser.angles_at(t)`, which the miss metric and the overlays use.

**Design decision:** keep `SimTurret` running as a shadow so the crosshair, 3D panel and miss metric still work. Send the same command line to the real turret in addition. The real hardware has no angle feedback, so the shadow is the only source of the "barrel" angle.

## 4. Centering and angle mapping

One rule ties everything together: **aim (yaw 0, pitch 0) equals servo (pan 0, tilt 90).**

| Space | Meaning | Center |
| --- | --- | --- |
| Aim space (repo, on the wire) | Degrees from straight ahead and level, sent as centidegrees | yaw 0, pitch 0 |
| Servo space (firmware only) | Angle sent to each servo | pan 0, tilt 90 |

The firmware converts between them:

```
pan_servo  = PAN_CENTER  + PAN_SIGN  * yaw     (PAN_CENTER  = 0)
tilt_servo = TILT_CENTER + TILT_SIGN * pitch   (TILT_CENTER = 90)
```

The signs are 1 or -1 and are set during bring-up if a direction is reversed. Each servo angle is clamped to its own safe range before it is written. Python never needs to know about servo angles.

With the default signs, the config pitch limits (-10 to 45) become tilt servo angles 80 to 135.

**Pan caveat:** if pan 0 is a physical end of the servo's travel rather than the middle, negative yaw has nowhere to go and the turret can only pan to one side. In that case either flip `PAN_SIGN` so the reachable side is the one you want, or re-seat the pan horn so the middle of travel faces forward and set `PAN_CENTER` to the new center. Nothing else in this spec changes. After calibration, set `YAW_MIN_DEG` and `YAW_MAX_DEG` in `config.py` to the range that is actually reachable.

## 5. Serial protocol (laptop to ESP32)

- 115200 baud, 8N1, ASCII lines ending in `\n`.
- `A,<yaw_cdeg>,<pitch_cdeg>,<fire>`: aim command, same as today. The firmware ignores `fire`.
- `S,<pan_servo_deg>,<tilt_servo_deg>`: raw servo angles in whole degrees, for calibration. Disables the watchdog.
- Malformed lines are dropped silently. The firmware sends `Ready` once at boot and nothing else.
- Python sends one `A` line per frame, only while a fit exists, which is the existing behavior.

## 6. Firmware

Replace the test sketch with this. Libraries: Adafruit PWM Servo Driver (and its BusIO dependency).

```cpp
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---- Set these during bring-up ----
const int   PAN_CH = 0, TILT_CH = 1;
const float PAN_CENTER = 0,  TILT_CENTER = 90;     // servo angle when centered
const int   PAN_SIGN = +1,   TILT_SIGN = +1;       // flip to -1 if a direction is reversed
const float PAN_MIN = 0,     PAN_MAX = 180;        // safe servo range, narrow after the sweep test
const float TILT_MIN = 80,   TILT_MAX = 135;
const int   US_MIN = 500,    US_MAX = 2500;        // pulse range from the servo datasheet
const float SERVO_RANGE_DEG = 180;                 // 270 for a 270-degree servo
const unsigned long WATCHDOG_MS = 1000;

Adafruit_PWMServoDriver pwm(0x40);
unsigned long lastCmd = 0;
bool autoMode = false;

void servoWrite(int ch, float deg, float lo, float hi) {
  deg = constrain(deg, lo, hi);
  pwm.writeMicroseconds(ch, US_MIN + (US_MAX - US_MIN) * deg / SERVO_RANGE_DEG);
}

void aimWrite(float yaw, float pitch) {
  servoWrite(PAN_CH,  PAN_CENTER  + PAN_SIGN  * yaw,   PAN_MIN,  PAN_MAX);
  servoWrite(TILT_CH, TILT_CENTER + TILT_SIGN * pitch, TILT_MIN, TILT_MAX);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  aimWrite(0, 0);                       // pan 0, tilt 90
  Serial.println("Ready");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    long a, b, f;
    if (sscanf(line.c_str(), "A,%ld,%ld,%ld", &a, &b, &f) == 3) {   // f ignored: no fire hardware
      aimWrite(a / 100.0, b / 100.0);
      autoMode = true; lastCmd = millis();
    } else if (sscanf(line.c_str(), "S,%ld,%ld", &a, &b) == 2) {     // raw servo angles
      servoWrite(PAN_CH, a, PAN_MIN, PAN_MAX);
      servoWrite(TILT_CH, b, TILT_MIN, TILT_MAX);
      autoMode = false;
    }
  }
  if (autoMode && millis() - lastCmd > WATCHDOG_MS) {   // laptop went quiet: re-center
    aimWrite(0, 0);
    autoMode = false;
  }
}
```

Behavior to keep: centered at boot, re-centered after 1 s without `A` lines, every output clamped to the servo's safe range.

## 7. Python changes

**`requirements.txt`:** add `pyserial==3.5`. Install with `uv pip install pyserial==3.5`.

**New file `hw.py`** (about 25 lines, with an assert self-check like the other modules):

```python
"""Real turret over serial. Same line format as SimTurret."""
import time
import serial
from config import YAW_MIN_DEG, YAW_MAX_DEG, PITCH_MIN_DEG, PITCH_MAX_DEG

def fmt(line: bytes) -> bytes:
    """Clamp an A line to the config limits and force fire to 0."""
    _, y, p, _ = line.decode().strip().split(",")
    y = max(YAW_MIN_DEG * 100, min(YAW_MAX_DEG * 100, int(y)))
    p = max(PITCH_MIN_DEG * 100, min(PITCH_MAX_DEG * 100, int(p)))
    return f"A,{y},{p},0\n".encode()

class HwTurret:
    def __init__(self, port, baud=115200):
        self.s = serial.Serial(port, baud, timeout=0, write_timeout=0.1)
        time.sleep(2.0)                      # the ESP32 resets when the port opens
    def write(self, line: bytes):
        self.s.write(fmt(line))
    def close(self):
        self.s.write(b"A,0,0,0\n"); self.s.close()   # center, then release

if __name__ == "__main__":
    assert fmt(b"A,9999,-5000,1\n") == f"A,{YAW_MAX_DEG*100},{PITCH_MIN_DEG*100},0\n".encode()
    print("hw.py OK")
```

**`main.py`** (four edits):

1. Add the argument `ap.add_argument("--port")`.
2. After `ser = SimTurret(now)` add `hw = HwTurret(args.port) if args.port else None` (import `HwTurret` from `hw`).
3. Replace the `ser.write(...)` line with `msg = f"A,{round(yaw*100)},{round(pitch*100)},0\n".encode(); ser.write(msg); hw and hw.write(msg)`.
4. Wrap the main loop in `try: ... finally: hw and hw.close()` so Ctrl-C and crashes still center the turret.

Without `--port`, behavior is identical to today.

**Camera geometry:** `CAMERA_OFFSET_T = (0.00, 0.10, 0.02)` in `config.py` is a placeholder. Measure the camera lens position relative to the pan/tilt pivot in meters (X right, Y up, Z forward) and update it. This replaces the manual parallax calibration in the original hardware guide, because `to_turret()` already corrects for the offset by geometry.

**Where to run:** COM ports exist on Windows, and WSL cannot see them without `usbipd`. Run hardware tests from the Windows copy of the repo with `.venv\Scripts\python.exe`, then copy changed files back to the WSL copy as the README requires.

## 8. Bring-up and acceptance tests

Run in order. Stop and fix before moving on.

1. **No regression.** `python main.py --webcam 0` (no `--port`) behaves as before.
2. **Center.** Power the PSU, upload the firmware, open the Serial Monitor (115200, Newline). The turret centers on boot. Send `S,0,90`, then `A,0,0,0`. Both must give the same physical pose, facing straight ahead and level.
3. **Directions.** Send `A,1000,0,0`: the turret should pan to the right, as seen from behind it looking forward. Send `A,0,1000,0`: the barrel should tilt up. If either is reversed, flip that `*_SIGN` in the firmware.
4. **Limits.** Use `S,<pan>,<tilt>` in small steps to find where each axis binds. Set `PAN_MIN/MAX` and `TILT_MIN/MAX` about 5 degrees inside those points, then set the yaw and pitch limits in `config.py` to match.
5. **Live tracking.** Close the Serial Monitor (only one program can hold the port). Run `python main.py --webcam 0 --port COMx`. The turret follows the hand and the on-screen crosshair roughly matches the real barrel direction.
6. **Fail-safe.** Press Ctrl-C: the turret centers. Kill the Python process abruptly: the turret re-centers within about 1 s.
7. **Pointing check.** Attach a pointer to the barrel and hold a target at a known distance. Compare to the on-screen miss and adjust `CAMERA_OFFSET_T` and `SERVO_LAG_S` as the README describes.

Acceptance: tests 1 to 6 pass, the loop median stays under 20 ms (printed on exit), and the turret does not twitch at its limits.

## 9. Open items and assumptions

- **A1, centered pose.** This spec reads "pan 0, tilt 90" as the servo angles at the centered pose. If it instead means pan is already a signed value around 0, set `PAN_CENTER` to the servo angle that is physically centered (usually 90) and nothing else changes.
- **A2, servo model.** The repo's simulator assumes DS3218MG servos. Those come in 180-degree and 270-degree versions, and `SERVO_RANGE_DEG` and the pulse range must match the one you have. A wrong value makes every angle too large or too small.
- **A3, depth.** The webcam path uses a fixed 2 m depth, so aim is only parallax-correct for a hand at about 2 m. A RealSense removes this. A cheaper option later is to estimate depth from the palm box width, calibrated at two known distances.
- **A4, COM port.** The CP2102 needs the Silicon Labs driver. Use the COM port it gets after installation, not the Bluetooth ports.

## 10. Files changed

| File | Change |
| --- | --- |
| `hw.py` | New |
| `main.py` | `--port` argument, send to hardware, `try/finally` cleanup |
| `requirements.txt` | Add `pyserial==3.5` |
| `config.py` | Update `CAMERA_OFFSET_T` and the yaw/pitch limits after calibration |
| ESP32 firmware | New sketch from section 6 |
| `log_spec.md` | Note the new file and the `--port` flag |

Safety: there is no firing mechanism in this spec. Keep servo power on its own PSU with a common ground, match the PSU voltage to the servo rating, and keep hands clear of the sweep area while testing.
