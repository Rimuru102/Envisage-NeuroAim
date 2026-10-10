"""MediaPipe hand -> 3D palm center in the camera frame (+X right, +Y down, +Z forward, m)."""
from dataclasses import dataclass
import numpy as np

PALM = (0, 5, 9, 13, 17)  # wrist + 4 knuckles; fingertips move relative to palm = jitter


@dataclass
class Hand:
    center_px: tuple[float, float]   # debug drawing only
    center_cam: np.ndarray | None    # (3,) m, camera frame; None = no depth this frame
    bbox: tuple[int, int, int, int]  # x0, y0, x1, y1 px


def palm_depth(depth, pts):
    """Median depth (m) of 5x5 windows around pts [(u, v)]; None if < 10 valid values."""
    h, w = depth.shape
    vals = np.concatenate([depth[max(int(v) - 2, 0):int(v) + 3, max(int(u) - 2, 0):int(u) + 3].ravel()
                           for u, v in pts])
    vals = vals[(vals >= 0.2) & (vals <= 4.0)]  # 0 = no data
    return float(np.median(vals)) if len(vals) >= 10 else None


class Vision:
    def __init__(self, intr, model="models/hand_landmarker.task"):
        import mediapipe as mp
        from mediapipe.tasks import python as mpt
        from mediapipe.tasks.python import vision as mpv
        self.mp, self.intr, self.last_ms = mp, intr, -1
        self.det = mpv.HandLandmarker.create_from_options(mpv.HandLandmarkerOptions(
            base_options=mpt.BaseOptions(model_asset_path=model),
            running_mode=mpv.RunningMode.VIDEO, num_hands=1,
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
        px = np.array([(p.x * w, p.y * h) for p in res.hand_landmarks[0]])
        u, v = px[list(PALM)].mean(axis=0)
        z = palm_depth(depth_m, px[list(PALM)])
        cam = None if z is None else np.array([(u - self.intr.ppx) / self.intr.fx * z,
                                               (v - self.intr.ppy) / self.intr.fy * z, z])
        x0, y0 = px.min(axis=0); x1, y1 = px.max(axis=0)
        return Hand((u, v), cam, (int(x0), int(y0), int(x1), int(y1)))


if __name__ == "__main__":
    d = np.full((480, 848), 2.0, np.float32)
    d[99:102, 99:102] = 0                       # 3x3 hole at a palm point
    pts = [(100, 100), (200, 100), (300, 100), (400, 100), (500, 100)]
    assert palm_depth(d, pts) == 2.0
    assert palm_depth(np.zeros((480, 848), np.float32), pts) is None
    print("vision.py OK")
