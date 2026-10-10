"""Camera->turret frame, line-fit Tracker, track-only aiming."""
import collections
from math import atan2, degrees, hypot
import numpy as np
from config import CAMERA_OFFSET_T, FIT_SAMPLES, FIT_MIN_SAMPLES, TRACK_JUMP_M, TRACK_TIMEOUT_S

OFF = np.array(CAMERA_OFFSET_T)


def to_turret(c):
    """Camera point (+Y down) -> turret frame (+Y up, origin at pivot)."""
    return np.array([c[0], -c[1], c[2]]) + OFF


class Tracker:
    def __init__(self):
        self.samples = collections.deque(maxlen=FIT_SAMPLES)  # (t, p_T)
        self.last_t, self.last_hand = 0.0, None

    def update(self, hand, t):
        if hand is None or hand.center_cam is None:
            if t - self.last_t > TRACK_TIMEOUT_S:
                self.samples.clear()
            return
        p = to_turret(hand.center_cam)
        if self.samples and np.linalg.norm(p - self.samples[-1][1]) > TRACK_JUMP_M:
            self.samples.clear()                              # new hand or bad depth
        self.samples.append((t, p))
        self.last_t, self.last_hand = t, hand

    def fit(self):
        """-> (p_now, v, t_newest, rms, n) or None. Straight line, least squares."""
        if len(self.samples) < FIT_MIN_SAMPLES:
            return None
        t = np.array([s[0] for s in self.samples])
        P = np.array([s[1] for s in self.samples])
        (v, p_now), res, *_ = np.polyfit(t - t[-1], P, 1, full=True)
        return p_now, v, t[-1], np.sqrt(res.sum() / len(t)), len(t)


def aim_point(p_now, v, delay_s):
    """Yaw, pitch (deg) and the predicted point (turret frame) delay_s from now. No ballistics."""
    a = p_now + v * delay_s
    return degrees(atan2(a[0], a[2])), degrees(atan2(a[1], hypot(a[0], a[2]))), a


if __name__ == "__main__":
    from types import SimpleNamespace
    assert np.allclose(to_turret((0.2, 0.1, 2.0)), (0.2, -0.1 + 0.1, 2.02))
    y, p, a = aim_point(np.array([0.2, 0.0, 2.02]), np.array([0.5, 0, 0]), 0.15)
    assert abs(y - degrees(atan2(0.275, 2.02))) < 1e-9 and abs(p) < 1e-9 and np.allclose(a, (0.275, 0, 2.02))
    rng, trk = np.random.default_rng(0), Tracker()
    for i in range(12):                                       # v = (0.5, 0, 0.2) m/s, 5 mm noise
        t = i / 60
        c = np.array([0.5 * t, 0, 2 + 0.2 * t]) + rng.normal(0, 0.005, 3)
        trk.update(SimpleNamespace(center_cam=c, bbox=(0, 0, 0, 0)), t)
    p_now, v, _, rms, n = trk.fit()
    assert n == 12 and np.all(abs(v - (0.5, 0, 0.2)) < 0.1) and rms < 0.02, (v, rms)
    trk.update(None, 1.0)
    assert trk.fit() is None                                  # timeout clears the track
    print("aim.py OK")
