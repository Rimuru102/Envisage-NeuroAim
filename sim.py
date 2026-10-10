"""Virtual 2-axis turret: two DS3218MG hobby servos behind the Arduino protocol.
Drop-in for serial.Serial: main.py only calls write(line); write() steps the physics to now."""
import collections, math, time
import numpy as np
from config import (YAW_MIN_DEG, YAW_MAX_DEG, PITCH_MIN_DEG, PITCH_MAX_DEG, CAMERA_OFFSET_T,
                    SERVO_J, SERVO_E_SAT_DEG, SERVO_TD_S, PITCH_GRAV_NM)

DT = 0.001                          # s, physics step
FRAME_S = 0.020                     # s, servo pulse period (Arduino Servo library)
US_PER_DEG = 11.11                  # same as firmware
DB_US = 3                           # us, DS3218MG deadband
TAU_STALL = 20 * 0.0981             # N*m, DS3218MG stall at 6 V
W_NOLOAD = math.radians(60 / 0.16)  # rad/s, DS3218MG no-load speed at 6 V

LO = np.radians([YAW_MIN_DEG, PITCH_MIN_DEG])
HI = np.radians([YAW_MAX_DEG, PITCH_MAX_DEG])
J = np.array(SERVO_J)                                   # kg*m^2, [yaw, pitch]
E_SAT = math.radians(SERVO_E_SAT_DEG)
DB = math.radians(DB_US / US_PER_DEG)
GRAV = np.array([0.0, PITCH_GRAV_NM])
OFF = np.array(CAMERA_OFFSET_T)


class SimTurret:
    def __init__(self, clock=time.time):
        self.clock = clock
        self.t = self.next_frame = clock()
        self.th = np.zeros(2)                    # rad, [yaw, pitch]
        self.w = np.zeros(2)                     # rad/s
        self.sp = np.zeros(2)                    # setpoint the servo is holding
        self.cmd = np.zeros(2)                   # newest command from the PC
        self.hist = collections.deque(maxlen=500)  # (t, yaw, pitch), last 0.5 s

    def write(self, line):                       # same call as serial.Serial.write
        self.advance(self.clock())               # catch physics up to now first
        f = line.decode().strip().split(",")
        if len(f) == 4 and f[0] == "A":          # firmware drops anything else
            deg = np.clip(np.radians([int(f[1]), int(f[2])]) / 100, LO, HI)
            self.cmd = np.radians(np.round(np.degrees(deg) * US_PER_DEG) / US_PER_DEG)  # integer us

    def advance(self, t):
        while self.t < t:
            self.t += DT
            if self.t >= self.next_frame:        # new 20 ms pulse: latch newest command
                self.next_frame += FRAME_S
                self.sp = self.cmd
            e = self.sp - self.th
            u = np.where(np.abs(e) < DB, 0.0, np.clip((e - SERVO_TD_S * self.w) / E_SAT, -1, 1))
            tau = TAU_STALL * (u - self.w / W_NOLOAD) - GRAV * np.cos(self.th)
            self.w += tau / J * DT               # semi-implicit Euler: speed first,
            self.th += self.w * DT               # then angle with the new speed
            self.hist.append((self.t, *self.th))

    def angles_at(self, t):
        if not self.hist:                        # no physics step yet
            return self.th[0], self.th[1]
        h = np.array(self.hist)
        return np.interp(t, h[:, 0], h[:, 1]), np.interp(t, h[:, 0], h[:, 2])


def barrel_dir(yaw, pitch):
    return np.array([math.sin(yaw) * math.cos(pitch), math.sin(pitch), math.cos(yaw) * math.cos(pitch)])


def miss_m(yaw, pitch, p_T):
    """Distance from palm point (turret frame) to the barrel line."""
    return float(np.linalg.norm(np.cross(barrel_dir(yaw, pitch), p_T)))


def to_px(p_T, intr):
    """Camera pixel of a turret-frame point (palm, prediction or a point on the barrel line)."""
    c = np.array(p_T) - OFF
    c[1] = -c[1]                                 # turret +Y up -> camera +Y down
    return int(intr.fx * c[0] / c[2] + intr.ppx), int(intr.fy * c[1] / c[2] + intr.ppy)


if __name__ == "__main__":
    from types import SimpleNamespace
    from aim import Tracker, aim_point
    from config import SERVO_LAG_S

    t = 0.0
    s = SimTurret(lambda: t); s.write(b"A,6000,0,0\n"); s.advance(0.35)
    assert abs(math.degrees(s.th[0]) - 60) < 1, s.th           # 60 deg step settles < 0.35 s

    s, lag = SimTurret(lambda: t), []
    while t < 2:                                                # 0.25 rad/s ramp, commanded at 60 Hz
        t += 1 / 60
        s.write(f"A,{round(math.degrees(0.25 * t) * 100)},0,0\n".encode())
        if t > 1: lag.append((0.25 * t - s.th[0]) / 0.25)
    print(f"servo lag {np.mean(lag) * 1000:.0f} ms (set SERVO_LAG_S to this)")

    def track(speed, lead):
        t, yaw, pitch, miss = 0.0, 0.0, 0.0, []
        rng, s, trk = np.random.default_rng(0), SimTurret(lambda: t), Tracker()
        while t < 4:
            t += 1 / 60; tc = t - 0.030                         # 30 ms vision latency
            p_T = np.array([-1.0 + speed * tc, 0.0, 2.0])       # true palm, 2 m away
            if tc > 0.6: miss.append(miss_m(*s.angles_at(tc), p_T))
            if rng.random() > 0.05:                             # 5 % frames lost
                c = p_T - OFF + rng.normal(0, 0.005, 3); c[1] = -c[1]   # 5 mm noise, camera frame
                trk.update(SimpleNamespace(center_cam=c, bbox=(0, 0, 0, 0)), tc)
            fit = trk.fit()
            if fit:
                p_now, v, t_last, _, _ = fit
                yaw, pitch, _ = aim_point(p_now, v, ((t - t_last) + SERVO_LAG_S) * lead)
            s.write(f"A,{round(yaw * 100)},{round(pitch * 100)},0\n".encode())
        return float(np.sqrt(np.mean(np.square(miss))))

    for v in (0.5, 1.0):
        on, off = track(v, 1), track(v, 0)
        print(f"{v} m/s: RMS miss {on * 100:.1f} cm with lead, {off * 100:.1f} cm without")
        assert on < 0.02 and off > 1.5 * on
    intr = SimpleNamespace(fx=600, fy=600, ppx=424, ppy=240)
    assert to_px(OFF + [0, 0, 2], intr) == (424, 240)                # point straight ahead of lens
    print("sim.py OK")
