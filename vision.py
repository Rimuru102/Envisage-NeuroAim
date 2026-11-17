"""MediaPipe hand -> 3D palm center in the camera frame (+X right, +Y down, +Z forward, m)."""
from dataclasses import dataclass
import numpy as np

PALM = (0, 5, 9, 13, 17)  # wrist + 4 knuckles; fingertips move relative to palm = jitter


@dataclass
class Hand:
    center_px: tuple[float, float]   # debug drawing only
    center_cam: np.ndarray | None    # (3,) m, camera frame; None = no depth this frame
    bbox: tuple[int, int, int, int]  # x0, y0, x1, y1 px
    palm_px: float = 0.0             # index-to-pinky knuckle width px; shrinks with distance
    n_hands: int = 1                 # hands the detector saw this frame
    others: tuple = ()               # centers (px) of the hands we are not following


def palm_depth(depth, pts):
    """Median depth (m) of 5x5 windows around pts [(u, v)]; None if < 10 valid values."""
    h, w = depth.shape
    vals = np.concatenate([depth[max(int(v) - 2, 0):int(v) + 3, max(int(u) - 2, 0):int(u) + 3].ravel()
                           for u, v in pts])
    vals = vals[(vals >= 0.2) & (vals <= 4.0)]  # 0 = no data
    return float(np.median(vals)) if len(vals) >= 10 else None


TRACK_HOLD_S, MAX_JUMP_FRAC = 1.5, 0.25   # stay with the hand we had; a hand this far away (fraction of image width) is another


def pick(centers, prev, t, w):
    """Index of the hand to follow, or None. With several hands, stay with the one nearest where ours last was;
    one far from there within TRACK_HOLD_S is somebody's other hand, so wait for ours."""
    if prev is None or t - prev[1] > TRACK_HOLD_S:
        return 0
    d = [float(np.hypot(*(np.asarray(c) - prev[0]))) for c in centers]
    k = int(np.argmin(d))
    return k if d[k] < MAX_JUMP_FRAC * w else None


class Vision:
    def __init__(self, intr, model="models/hand_landmarker.task", num_hands=1):
        import mediapipe as mp
        from mediapipe.tasks import python as mpt
        from mediapipe.tasks.python import vision as mpv
        self.mp, self.intr, self.last_ms, self.num_hands, self.prev = mp, intr, -1, num_hands, None
        self.det = mpv.HandLandmarker.create_from_options(mpv.HandLandmarkerOptions(
            base_options=mpt.BaseOptions(model_asset_path=model),
            running_mode=mpv.RunningMode.VIDEO, num_hands=num_hands,
            min_hand_detection_confidence=0.5, min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5))

    def process(self, color_bgr, depth_m, t_frame_s):
        """-> Hand or None"""
        ms = max(int(t_frame_s * 1000), self.last_ms + 1)  # MediaPipe needs increasing ms
        self.last_ms = ms
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB,
                            data=np.ascontiguousarray(color_bgr[:, :, ::-1]))
        res = self.det.detect_for_video(img, ms)
        if not res.hand_landmarks:
            return None
        h, w = color_bgr.shape[:2]
        hands = [np.array([(p.x * w, p.y * h) for p in lm]) for lm in res.hand_landmarks]
        cent = [px[list(PALM)].mean(axis=0) for px in hands]
        k = pick(cent, self.prev, t_frame_s, w) if self.num_hands > 1 else 0
        if k is None:
            return None
        self.prev = (cent[k], t_frame_s)
        px, (u, v) = hands[k], cent[k]
        z = palm_depth(depth_m, px[list(PALM)])
        cam = None if z is None else np.array([(u - self.intr.ppx) / self.intr.fx * z,
                                               (v - self.intr.ppy) / self.intr.fy * z, z])
        x0, y0 = px.min(axis=0); x1, y1 = px.max(axis=0)
        return Hand((u, v), cam, (int(x0), int(y0), int(x1), int(y1)), float(np.linalg.norm(px[5] - px[17])),
                    len(hands), tuple(tuple(c) for i, c in enumerate(cent) if i != k))


if __name__ == "__main__":
    d = np.full((480, 848), 2.0, np.float32)
    d[99:102, 99:102] = 0                       # 3x3 hole at a palm point
    pts = [(100, 100), (200, 100), (300, 100), (400, 100), (500, 100)]
    assert palm_depth(d, pts) == 2.0
    assert palm_depth(np.zeros((480, 848), np.float32), pts) is None
    assert pick([(100, 100), (700, 400)], ((110, 105), 0.0), 0.1, 848) == 0          # stay with the hand we had
    assert pick([(700, 400), (100, 100)], ((110, 105), 0.0), 0.1, 848) == 1          # even if it is listed second
    assert pick([(700, 400)], ((110, 105), 0.0), 0.1, 848) is None                  # only the other hand: wait
    assert pick([(700, 400)], ((110, 105), 0.0), 2.0, 848) == 0                     # ours gone for good: take it
    assert pick([(700, 400)], None, 0.1, 848) == 0
    print("vision.py OK")
