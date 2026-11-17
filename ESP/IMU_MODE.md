# Handheld IMU mode

A second ESP32 + BNO055/BNO085 is the wand. Where you point it is where the turret points. The two ESPs talk directly over
ESP-NOW (no router, no laptop). While the wand is live it wins over the laptop's `A,` commands; switch it off to use the camera again.

```
wand: BNO055/BNO085 -> aim_math.h (point, zero, drift, filter) -> imu_link.h packet, 50 Hz -> ESP-NOW -> turret ESP -> servos
```

| File | What |
| --- | --- |
| `handheld/handheld.ino` | Wand firmware (second ESP32) |
| `handheld/aim_math.h` | Orientation to aim angles, drift tracker, noise filter (plain C++, tested on a PC) |
| `handheld/imu_link.h`, `turret/imu_link.h` | Radio packet. Identical copies: edit one, copy to the other |
| `turret/turret.ino` | Turret firmware: serial `A,`/`S,` as before, plus the wand link |
| `tests/run_tests.sh` | PC tests: `sh ESP/tests/run_tests.sh` (needs g++) |

## Setup
1. Arduino Library Manager: install **Adafruit BNO055** (and **Adafruit Unified Sensor**), or **Adafruit BNO08x** for a BNO085. BusIO is already there.
2. Wire the IMU to the wand ESP32: 3V3, GND, SDA = GPIO21, SCL = GPIO22. BNO055 address is 0x28 (0x29 if ADR is high); BNO085 is 0x4A.
3. Wand sketch: for a BNO085 set `#define IMU_TYPE 85` at the top of `handheld.ino`. Board "DOIT ESP32 DEVKIT V1", like the turret.
4. Upload `turret.ino` to the turret ESP and `handheld.ino` to the wand ESP.

## Bring-up (stop and fix before moving on)
1. **Turret unchanged.** Serial Monitor on the turret (115200, Newline): `Ready`, then `S,90,150` and `A,0,0,0` still work. Send `L`: you get `L,4294967295,0,0,0` (no packets yet).
2. **Wand alone.** Serial Monitor on the wand. Power up holding it still, pointing straight ahead: after about 2 s it prints `yaw .. pitch ..`. Check:
   - turn it **right**: yaw increases. Reversed: flip `YAW_SIGN`.
   - tilt the nose **up**: pitch increases. Reversed: flip `PITCH_SIGN`.
   - **roll** it about its pointing axis, and point it 90 degrees to either side then tilt: pitch must still equal the nose angle and yaw must not move when you only roll. If it does, the pointing axis is wrong: set `POINT_AXIS` to the sensor axis that actually points at the target.
3. **Link.** With the wand on, send `L` on the turret: the first number (ms since last packet) stays under about 40, the packet count climbs, dropped stays near 0.
4. **Follow.** Clear the sweep area and move the wand slowly. The turret follows. It never moves faster than 400 deg/s (`MAX_SLEW_DPS`), so the first packet cannot slam it.
5. **Fail-safes.** Switch the wand off: the turret holds about 1 s, then eases back to center. With the wand on, `A,` lines from the laptop are ignored.
6. **Drift.** Hold the wand still for a minute: the printed `drift` value settles and yaw stays put.

## Calibration and drift
- **BNO055:** wave it in figure-eights and rest it on each face until `cal` shows 1. It saves its calibration to flash once per boot and reloads it at the next one.
- **Heading anchor:** `USE_MAGNETOMETER 1` (default) ties heading to the magnetometer, so there is no long-term drift, but magnets and motors near the wand bend it. If yaw wanders near the turret, set it to 0: heading then comes from the gyro alone and the drift tracker removes the slow residual.
- **Re-zero:** send `z` over the wand's USB serial, or wire a button from `RECENTER_PIN` to GND.

## Tuning (all in `aim_math.h`)
`OneEuro` `mincut` (default 0.8 Hz): lower = smoother when slow. `beta` (0.12): higher = less lag in fast sweeps. `Pointer::hyst` (0.08 deg): output deadband.

## Security
Packets are broadcast with a magic header, CRC and a shared key (`LINK_KEY`), enough to ignore stray traffic but not to stop a determined sender. Change `LINK_KEY` on both sides to pair your own units.
