"""Side panel: flat-shaded 3D turret in an empty scene, drawn with OpenCV only.
Turret frame: origin at the pivot, +X right, +Y up, +Z forward, meters."""
import math
import cv2
import numpy as np
from config import CAMERA_OFFSET_T

SIZE = 480                                   # px, square panel (same height as camera image)
F = 520                                      # px, panel focal length
EYE = np.array([0.6, 0.6, -1.3])            # m, viewer behind, right of and above the turret
LOOK = np.array([0.0, -0.15, 0.9])           # m, point the view centres on
FLOOR_Y = -0.30                              # m, table top below the pivot
LIGHT = np.array([0.4, 0.8, -0.45]) / np.linalg.norm([0.4, 0.8, -0.45])

_f = (LOOK - EYE) / np.linalg.norm(LOOK - EYE)
_r = np.cross([0, 1, 0], _f); _r /= np.linalg.norm(_r)
_u = np.cross(_f, _r)
VIEW = np.stack([_r, _u, _f])                # rows: right, up, forward

# (part, centre, half-size, BGR colour); part = which rotation it follows
BOXES = [
    ("fixed", (0, -0.27, 0), (0.12, 0.03, 0.12), (90, 90, 90)),      # base plate
    ("fixed", CAMERA_OFFSET_T, (0.045, 0.013, 0.013), (60, 60, 60)),  # RealSense
    ("yaw", (0, -0.20, 0), (0.03, 0.04, 0.03), (120, 120, 120)),      # yaw post
    ("yaw", (0, -0.15, 0), (0.09, 0.01, 0.05), (140, 140, 140)),      # yoke floor
    ("yaw", (-0.075, -0.07, 0), (0.008, 0.08, 0.04), (140, 140, 140)),  # yoke left
    ("yaw", (0.075, -0.07, 0), (0.008, 0.08, 0.04), (140, 140, 140)),   # yoke right
    ("pitch", (0, 0, 0.03), (0.05, 0.06, 0.15), (40, 140, 230)),      # blaster body
    ("pitch", (0, 0.01, 0.24), (0.015, 0.015, 0.06), (40, 40, 40)),   # barrel
]
FACES = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
CORNERS = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])


def rot(yaw, pitch):
    """Same convention as sim.barrel_dir: R_y(yaw) @ R_x(pitch)."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rx = np.array([[1, 0, 0], [0, cp, sp], [0, -sp, cp]])
    return ry @ rx


def px(p):
    v = VIEW @ (np.asarray(p) - EYE)
    return (int(SIZE / 2 + F * v[0] / v[2]), int(SIZE / 2 - F * v[1] / v[2])) if v[2] > 0.05 else None


def line(img, a, b, col, w=1):
    a, b = px(a), px(b)
    if a and b: cv2.line(img, a, b, col, w, cv2.LINE_AA)


def render(yaw, pitch, palm=None, pred=None, trail=(), hit=True):
    img = np.full((SIZE, SIZE, 3), 32, np.uint8)
    for g in np.arange(-2.0, 2.01, 0.25):                        # floor grid, 0.25 m squares
        line(img, (g, FLOOR_Y, -0.5), (g, FLOOR_Y, 3.5), (55, 55, 55))
    for g in np.arange(-0.5, 3.51, 0.25):
        line(img, (-2, FLOOR_Y, g), (2, FLOOR_Y, g), (55, 55, 55))
    rs = {"fixed": np.eye(3), "yaw": rot(yaw, 0), "pitch": rot(yaw, pitch)}
    faces = []
    for part, c, h, col in BOXES:
        R, c = rs[part], np.array(c)
        pts = (c + CORNERS * h) @ R.T
        mid = c @ R.T
        for f in FACES:
            q = pts[list(f)]; fc = q.mean(0); n = fc - mid; n /= np.linalg.norm(n)
            if n @ (EYE - fc) <= 0: continue                         # back face
            shade = 0.45 + 0.55 * max(0.0, n @ LIGHT)
            faces.append((np.linalg.norm(fc - EYE), [px(p) for p in q], tuple(int(k * shade) for k in col)))
    for _, poly, col in sorted(faces, key=lambda f: -f[0]):         # painter's: far first
        if None not in poly: cv2.fillConvexPoly(img, np.array(poly), col, cv2.LINE_AA)
    d = rot(yaw, pitch) @ [0, 0, 1]
    rng = np.linalg.norm(palm) if palm is not None else 3.0
    line(img, d * 0.30, d * rng, (0, 220, 0) if hit else (0, 0, 255), 2)    # barrel line
    for a, b in zip(trail, trail[1:]): line(img, a, b, (200, 200, 200))     # samples in the fit
    if pred is not None:
        line(img, pred[0], pred[1], (255, 255, 0), 2)                        # lead vector
        if px(pred[1]): cv2.circle(img, px(pred[1]), 7, (255, 255, 0), 2, cv2.LINE_AA)
    if palm is not None and px(palm):
        cv2.circle(img, px(palm), 6, (255, 0, 255), -1, cv2.LINE_AA)
        line(img, palm, (palm[0], FLOOR_Y, palm[2]), (90, 90, 90))           # drop line to floor
    cv2.putText(img, f"yaw {math.degrees(yaw):6.1f}  pitch {math.degrees(pitch):5.1f} deg",
                (10, SIZE - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)
    return img


if __name__ == "__main__":
    import time
    palm = np.array([0.45, 0.10, 2.0]); v = np.array([0.5, 0.0, 0.0])
    trail = [palm - v * k / 60 for k in range(14, -1, -1)]
    img = render(math.radians(14), math.radians(3), palm, (palm, palm + v * 0.075), trail)
    assert img.shape == (SIZE, SIZE, 3) and img.std() > 5               # something was drawn
    t0 = time.perf_counter()
    for _ in range(100): render(0.3, 0.1, palm, (palm, palm + v * 0.075), trail)
    print(f"render {(time.perf_counter() - t0) * 10:.2f} ms per frame")
    cv2.imwrite("view3d_demo.png", img)
    print("view3d.py OK")
