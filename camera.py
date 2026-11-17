"""RealSense D435: aligned color + depth on the host clock."""
import threading, time
import cv2
import numpy as np
from config import COLOR_EXPOSURE


class Camera:
    clock = staticmethod(time.time)

    def __init__(self):
        import pyrealsense2 as rs
        self.rs = rs
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        self.pipe = rs.pipeline()
        prof = self.pipe.start(cfg)
        dev = prof.get_device()
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        self.intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color)
        cs = dev.first_color_sensor()
        cs.set_option(rs.option.enable_auto_exposure, 0)
        cs.set_option(rs.option.exposure, COLOR_EXPOSURE)
        self.checked = False

    def read(self):
        """-> (color_bgr, depth_m, t_frame_s)"""
        rs = self.rs
        while True:
            f = self.align.process(self.pipe.wait_for_frames(1000))
            c, d = f.get_color_frame(), f.get_depth_frame()
            if c and d:
                break
        if not self.checked:  # host clock, so time.time() - t is true latency
            if c.get_frame_timestamp_domain() != rs.timestamp_domain.global_time:
                for s in self.pipe.get_active_profile().get_device().query_sensors():
                    s.set_option(rs.option.global_time_enabled, 1)
            self.checked = True
        return (np.asanyarray(c.get_data()),
                np.asanyarray(d.get_data()).astype(np.float32) * self.depth_scale,
                c.get_timestamp() / 1000.0)


class VideoCamera:
    """Offline/webcam stand-in: video file path or webcam index, flat depth plane at z m,
    D435-like field of view. File: video-time clock (deterministic). Webcam: wall clock."""

    def __init__(self, src, z=2.0):
        self.live = isinstance(src, int)
        self.cap, self.z, self.t, self.n = cv2.VideoCapture(src), z, 0.0, 0
        if self.live:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)   # a leftover manual exposure leaves the image black
        w, h = (self.cap.get(p) for p in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT))
        assert w and self.cap.isOpened(), f"cannot open {src!r}"
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30
        f = 0.725 * w  # D435 color: fx 615 at 848 px wide
        self.intr = type("Intr", (), dict(fx=f, fy=f, ppx=w / 2, ppy=h / 2))
        if self.live:                       # grab in a thread so read() never returns a stale queued frame
            self.cv, self.latest, self.seq, self.seen = threading.Condition(), None, 0, 0
            threading.Thread(target=self._grab, daemon=True).start()

    def _grab(self):
        while True:
            ok, color = self.cap.read()
            with self.cv:
                self.latest = (color, time.time()) if ok else None
                self.seq += 1
                self.cv.notify_all()
            if not ok:
                return

    def clock(self):
        return time.time() if self.live else self.t

    def read(self):
        if self.live:                       # newest frame only; older ones are dropped
            with self.cv:
                self.cv.wait_for(lambda: self.seq != self.seen, timeout=1.0)
                self.seen = self.seq
                if self.latest is None:
                    return None
                color, self.t = self.latest
        else:
            ok, color = self.cap.read()
            if not ok:
                return None
            self.t = self.n / self.fps
        self.n += 1
        return color, np.full(color.shape[:2], self.z, np.float32), self.t
