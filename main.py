"""Camera + vision -> virtual turret.
   python main.py                     live RealSense, window, q quits
   python main.py --webcam [index]    webcam, fixed 2 m depth
   python main.py --video in.mp4 --out out.mp4   offline, annotated video"""
import argparse, csv, math, time
import cv2
import numpy as np
from aim import Tracker, aim_point, to_turret
from camera import Camera, VideoCamera
from config import SERVO_LAG_S
from sim import SimTurret, barrel_dir, miss_m, to_px
from view3d import render
from vision import Vision

ap = argparse.ArgumentParser()
ap.add_argument("--video"); ap.add_argument("--out")
ap.add_argument("--webcam", type=int, nargs="?", const=0)
args = ap.parse_args()
cam = VideoCamera(args.video) if args.video else VideoCamera(args.webcam) if args.webcam is not None else Camera()
now = cam.clock
vis = Vision(cam.intr); trk = Tracker(); ser = SimTurret(now); writer = None
yaw, pitch = 0.0, 0.0
misses, lat, loop, frames = [], [], [], 0
f = open("sim_log.csv", "w", newline="")
log = csv.writer(f)
log.writerow(["t", "x", "y", "z", "yaw_cmd", "pitch_cmd", "yaw", "pitch", "miss_m"])
while True:
    t0 = time.time()
    r = cam.read()
    if r is None:
        break
    color, depth, t = r
    hand = vis.process(color, depth, t)
    trk.update(hand, t)
    fit, a, p, m = trk.fit(), None, None, 1.0
    if fit:
        p_now, v, t_last, _, _ = fit
        yaw, pitch, a = aim_point(p_now, v, (now() - t_last) + SERVO_LAG_S)
    ser.write(f"A,{round(yaw * 100)},{round(pitch * 100)},0\n".encode())  # fire always 0
    frames += 1; lat.append(now() - t)
    if hand and hand.center_cam is not None:
        p = to_turret(hand.center_cam)
        ya, pa = ser.angles_at(t)                    # barrel at this frame's capture time
        m = miss_m(ya, pa, p); misses.append(m)
        log.writerow([f"{t:.4f}", *np.round(p, 4), yaw, pitch,
                      round(math.degrees(ya), 3), round(math.degrees(pa), 3), round(m, 4)])
        cv2.drawMarker(color, to_px(barrel_dir(*ser.th) * np.linalg.norm(p), cam.intr),
                       (0, 255, 0) if m < 0.03 else (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    if hand:
        cv2.rectangle(color, hand.bbox[:2], hand.bbox[2:], (255, 0, 0), 2)
        cv2.circle(color, tuple(int(c) for c in hand.center_px), 5, (255, 0, 0), -1)
    if fit:                                              # predicted hand position: line + ring
        cv2.line(color, to_px(p_now, cam.intr), to_px(a, cam.intr), (255, 255, 0), 2)
        cv2.circle(color, to_px(a, cam.intr), 9, (255, 255, 0), 2)
    panel = render(*ser.th, p, (p_now, a) if fit else None, [s[1] for s in trk.samples], m < 0.03)
    color = np.hstack([color, cv2.resize(panel, (color.shape[0], color.shape[0]))])
    loop.append(time.time() - t0)
    if args.out:
        writer = writer or cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), cam.fps, color.shape[1::-1])
        writer.write(color)
    else:
        cv2.imshow("sim", color)
        if cv2.waitKey(1) == ord("q"):
            break
f.close()
writer and writer.release()
if misses:
    print(f"frames {frames}, hand seen {len(misses) / frames:.0%}, miss median "
          f"{np.median(misses) * 100:.1f} cm, p95 {np.percentile(misses, 95) * 100:.1f} cm, "
          f"latency median {np.median(lat) * 1000:.0f} ms, loop median {np.median(loop) * 1000:.0f} ms")
