"""Calibrate how the camera sits on the turret: lens field-of-view scale (2), camera rotation (3), offset (3).
   python calibrate.py --webcam 0 --port COM7
   python calibrate.py --port COM7                RealSense: depth is measured
The turret follows your hand. Keep the hand steady; when the barrel is off the palm, tap a/d/w/s to correct it.
A point records itself one second after you stop nudging (it only records points you corrected;
press SPACE to confirm a point where the barrel is already right). The estimate improves live. Move the hand somewhere new
(the screen says where), including the corners of the image, until it shows GOOD, then press c.
Imperfect points are fine: bad ones are down-weighted or dropped, and the +- includes how well the points agree.
Webcam depth: hold the palm at a measured distance (set it with -/=), press l to lock the scale; after that
depth follows palm size. Keys: a/d yaw, s/w pitch (0.5 deg, upper case 2 deg, holding a key speeds up), SPACE record now, u undo,
r auto-record on/off, l lock depth scale, c save calib.json, q quit."""
import argparse, collections, csv, json, os, time
import cv2
import numpy as np
from aim import rot
from camera import Camera, VideoCamera
from config import YAW_MIN_DEG, YAW_MAX_DEG, PITCH_MIN_DEG, PITCH_MAX_DEG
from hw import HwTurret
from vision import Vision

CALIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json")
LOGDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
# parameters: [sx, sy, rx, ry, rz, ox, oy, oz]; camera x,y scaled by exp(sx), exp(sy), rotated by rotation vector r, shifted by o
PRIOR0 = np.array([0, 0, 0, 0, 0, 0.14, 0.20, 0.00])      # no scale error, aligned; camera ~14 cm right, 20 cm above the pivot (from your logs; --offset overrides)
PRIOR_SIG = np.array([0.25, 0.25, 0.20, 0.20, 0.20, 0.30, 0.30, 0.30])   # how far each may plausibly be (log, rad, m)
SIGMA_DEG = 1.0                                  # aiming-by-eye error of one point, before its own hand jitter
HUBER_K, REJECT_K = 2.0, 3.5                     # error (in sigmas) where down-weighting starts / a point is dropped
LIMITS = np.array([0.7, 0.7, 0.50, 0.50, 0.50, 1.0, 1.0, 1.0])   # refuse to adopt a solution beyond these
MAX_RESIDUAL_DEG = 8.0
TRIM_RADIUS = 10.0                               # deg: a nudge fades out as the hand moves this far from where it was set
STEADY_M, NEW_DEG, NEW_Z, MAX_POINTS = 0.015, 6.0, 0.2, 20   # auto-record: steadiness, "new place" thresholds, cap
ADOPT_MIN, BLEND, HOLD_S = 6, 0.15, 1.5          # points before the turret follows the estimate, per-frame blend, hold-last-aim time
MIN_Z, JUMP_M, Z_SLEW = 0.25, 0.12, 0.10         # closest usable hand (m), one-frame teleport that is a glitch (m), max depth change per frame
LO, HI = np.array([YAW_MIN_DEG, PITCH_MIN_DEG]), np.array([YAW_MAX_DEG, PITCH_MAX_DEG])


def predict(th, C):
    """Camera points C (n,3, +Y down, nominal lens) -> (yaw, pitch) in degrees the turret must point at them."""
    P = np.stack([C[:, 0] * np.exp(th[0]), -C[:, 1] * np.exp(th[1]), C[:, 2]], 1) @ rot(th[2:5]).T + th[5:8]
    return np.degrees(np.arctan2(P[:, 0], P[:, 2])), np.degrees(np.arctan2(P[:, 1], np.hypot(P[:, 0], P[:, 2])))


def _resid(th, C, yaw, pitch, sig):
    py, pp = predict(th, C)
    return np.stack([(py - yaw) * np.cos(np.radians(pitch)) / sig, (pp - pitch) / sig], 1)


def _jac(f, th, eps=1e-6):
    J = np.empty((len(f(th)), len(th)))
    for k in range(len(th)):
        d = np.zeros(len(th)); d[k] = eps
        J[:, k] = (f(th + d) - f(th - d)) / (2 * eps)
    return J


def solve(samples):
    """samples [(c_cam, yaw_deg, pitch_deg[, sigma_deg])] -> (params (8,), error_deg per sample, sd (8,), kept mask).
    Levenberg-Marquardt on the angular error of every point, with a weak pull of each parameter toward PRIOR0
    so one the data cannot determine stays sane. Robust: Huber down-weighting (iterated), then points beyond
    REJECT_K sigmas are dropped. sd is scaled up when the points disagree with each other more than their sigmas
    say, so it never claims more than the data supports."""
    C = np.array([s[0] for s in samples], float)
    yaw, pitch = np.array([s[1] for s in samples], float), np.array([s[2] for s in samples], float)
    sig = np.array([s[3] if len(s) > 3 else SIGMA_DEG for s in samples], float)

    def full(th, w):
        return np.concatenate([(_resid(th, C, yaw, pitch, sig) * np.sqrt(w)[:, None]).ravel(), (th - PRIOR0) / PRIOR_SIG])

    def lm(th, w):
        f, lam = (lambda t: full(t, w)), 1e-2
        cost = f(th) @ f(th)
        for _ in range(25):
            r, J = f(th), _jac(f, th)
            H, g = J.T @ J, J.T @ r
            while True:
                step = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-9), -g)
                new = f(th + step); c2 = new @ new
                if c2 < cost:
                    th, cost, lam = th + step, c2, lam / 3
                    break
                lam *= 5
                if lam > 1e8:
                    return th
            if np.abs(step).max() < 1e-7:
                break
        return th

    th, w = PRIOR0.copy(), np.ones(len(samples))
    for _ in range(6):                                          # Huber IRLS
        th = lm(th, w)
        rn = np.linalg.norm(_resid(th, C, yaw, pitch, sig), axis=1)
        w = np.where(rn <= HUBER_K, 1.0, HUBER_K / np.maximum(rn, 1e-9))
    keep = rn <= REJECT_K
    if keep.sum() >= 5:                                         # drop the clearly wrong points and refit
        w = w * keep
        th = lm(th, w)
    else:
        keep[:] = True
    R = _resid(th, C, yaw, pitch, sig); rn = np.linalg.norm(R, axis=1)
    J = _jac(lambda t: full(t, w), th)
    cov = np.linalg.inv(J.T @ J) * max(1.0, np.sum(rn[keep] ** 2) / max(1, 2 * keep.sum() - 8))
    return th, rn * sig, np.sqrt(np.diag(cov)), keep


def loo_deg(samples):
    """Leave-one-out check: error (deg) of each point predicted by a fit that never saw it."""
    out = []
    for i in range(len(samples)):
        th = solve(samples[:i] + samples[i + 1:])[0]
        c, y, p = samples[i][:3]
        py, pp = predict(th, np.array([c], float))
        out.append(float(np.hypot((py[0] - y) * np.cos(np.radians(p)), pp[0] - p)))
    return np.array(out)


def trim_weight(d, trim_dir):
    """How much of a nudge still applies when the hand is in camera direction d: 1 where it was set, fading with angle."""
    if trim_dir is None:
        return 1.0
    return float(np.exp(-0.5 * (np.degrees(np.arccos(np.clip(d @ trim_dir, -1, 1))) / TRIM_RADIUS) ** 2))


def hint(samples):
    """Where the hand should go next, or None when the points cover enough directions and distances."""
    if not samples:
        return "hold your hand still in view"
    ang = np.array([[np.degrees(np.arctan2(c[0], c[2])), np.degrees(np.arctan2(-c[1], np.hypot(c[0], c[2])))]
                    for c, *_ in samples])
    z = np.array([c[2] for c, *_ in samples])
    for word, missing in (("to the LEFT edge", ang[:, 0].min() > -18), ("to the RIGHT edge", ang[:, 0].max() < 18),
                          ("to the BOTTOM edge", ang[:, 1].min() > -10), ("to the TOP edge", ang[:, 1].max() < 10),
                          ("CLOSER or FARTHER (change the distance)", z.max() / z.min() < 1.8)):
        if missing:
            return "move hand " + word
    return None


def selftest():
    true = np.array([0.15, -0.20, 0.03, -0.04, 0.02, 0.03, 0.12, -0.05])     # lens 16% / 18% off, rotated, offset
    rng = np.random.default_rng(1)

    def points(n, az=30, el=18):
        z = rng.uniform(0.6, 2.2, n)
        return np.stack([z * np.tan(np.radians(rng.uniform(-az, az, n))), z * np.tan(np.radians(rng.uniform(-el, el, n))), z], 1)

    def sample(C, noise, bias=0.0, th=true):
        yaw, pitch = predict(th, C[None])
        return (C, yaw[0] + rng.normal(0, noise) + bias, pitch[0] + rng.normal(0, noise))

    pts = points(14)
    th, err, sd, keep = solve([sample(c, 0) for c in pts])
    assert np.allclose(th, true, atol=[0.03] * 2 + [0.02] * 3 + [0.04] * 3) and err.max() < 0.3, (th, err)   # clean data
    th, err, sd, keep = solve([sample(c, 0.3) for c in pts])                                  # 0.3 deg aiming noise
    assert abs(th[0] - true[0]) < 0.05 and abs(th[1] - true[1]) < 0.05 and np.all(abs(th[2:5] - true[2:5]) < 0.03) \
        and np.all(abs(th[5:7] - true[5:7]) < 0.05) and err.max() < 1.5, (th, err, sd)
    clean_sd = sd
    bad = [sample(c, 0.3) for c in pts] + [sample(pts[0], 0, 12.0), sample(pts[3], 0, -9.0)]
    th, err, sd, keep = solve(bad)                                                            # two wildly wrong points
    assert abs(th[0] - true[0]) < 0.06 and abs(th[1] - true[1]) < 0.06 and not keep[-2:].any() and keep[:14].sum() >= 12, (th, keep)
    th, err, sd, _ = solve([sample(c, 1.5) for c in pts])                                     # sloppy aiming: wider sd
    assert abs(th[0] - true[0]) < 0.15 and abs(th[1] - true[1]) < 0.15 and sd[:2].max() > clean_sd[:2].max(), (th, sd)
    th, *_ = solve([sample(c, 0.5) for c in points(6, 4, 3)])                                 # near-axis points: must not explode
    assert np.all(np.abs(th) < LIMITS), th
    th, *_ = solve([sample(c, 0, th=PRIOR0) for c in pts])                                    # perfect lens: stays at no-error
    assert np.allclose(th, PRIOR0, atol=0.03), th
    far = np.array([0, 0, 0, 0, 0, 0.14, 0.28, -0.15])                                        # camera well off to one side, high, behind
    th, *_ = solve([sample(c, 0.3, th=far) for c in points(10, 25, 15)])
    assert np.all(abs(th[5:8] - far[5:8]) < 0.08), th
    d0 = np.array([0, 0, 1.0]); d10 = np.array([np.sin(np.radians(10)), 0, np.cos(np.radians(10))])
    assert trim_weight(d0, None) == 1 and trim_weight(d0, d0) == 1 and abs(trim_weight(d10, d0) - np.exp(-0.5)) < 1e-9
    assert trim_weight(np.array([1.0, 0, 0]), d0) < 1e-6                                       # far away: nudge gone
    flat = [(np.array([z * np.tan(np.radians(a)), 0.0, z]), 0, 0) for a, z in
            ((-25, .5), (-12, .7), (0, .6), (12, .9), (25, 1.2), (5, 1.1))]                  # all on one horizontal line
    need = hint(flat)
    assert need and "BOTTOM" in need and hint(flat + [(np.array([0.0, 0.0, 0.8]), 0, 0)]) == need   # more of the same: no help
    assert hint(flat + [(np.array([0.0, 0.3, 0.8]), 0, 0)]) != need                                   # a point below the line: helps
    loo = loo_deg([sample(c, 0.3) for c in pts])
    assert loo.mean() < 1.5, loo
    print("calibrate.py OK, noisy fit", np.round(th := solve([sample(c, 0.3) for c in pts])[0], 3), "leave-one-out", f"{loo.mean():.2f} deg")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--webcam", type=int, nargs="?", const=0)
    ap.add_argument("--port")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="camera lens position relative to the turret pivot in m: right, up, forward (measure with a ruler)")
    args = ap.parse_args()
    if args.selftest:
        selftest(); raise SystemExit
    assert args.port, "--port is required"
    if args.offset:
        PRIOR0[5:8] = args.offset
    cam = VideoCamera(args.webcam) if args.webcam is not None else Camera()
    webcam = args.webcam is not None
    vis, hw = Vision(cam.intr, num_hands=2), HwTurret(args.port)
    os.makedirs(LOGDIR, exist_ok=True)
    stamp, T0 = time.strftime("%Y%m%d_%H%M%S"), time.time()
    logf = open(os.path.join(LOGDIR, f"calib_{stamp}.csv"), "w", newline=""); logw = csv.writer(logf)
    evf = open(os.path.join(LOGDIR, f"calib_{stamp}_events.log"), "w")
    logw.writerow(["t", "hands", "u_px", "v_px", "palm_px", "cx", "cy", "cz", "state", "aim_yaw", "aim_pitch", "trim_yaw",
                   "trim_pitch", "cmd_yaw", "cmd_pitch", "at_limit", "points", "kept", *[f"th{i}" for i in range(8)]])

    def event(text, show=False):
        line = f"{time.time() - T0:8.2f}s {text}"
        evf.write(line + "\n"); evf.flush()
        if show: print(line)
    event(f"start offset seed={PRIOR0[5:8].tolist()} webcam={webcam} port={args.port} limits yaw {YAW_MIN_DEG}..{YAW_MAX_DEG} pitch {PITCH_MIN_DEG}..{PITCH_MAX_DEG}")
    was_seen = was_multi = was_hold = False
    th, th_goal, est, z, lock, wbuf = PRIOR0.copy(), PRIOR0.copy(), None, 1.0, None, collections.deque(maxlen=5)
    trim, recent, samples = np.zeros(2), [], []
    auto, last_key, last_rec, last_c = True, 0.0, 0.0, 0.0
    msg = "nudge the barrel onto your palm (a/d/w/s); SPACE if it is already on target"
    last_cmd, last_seen, at_limit = None, 0.0, False
    trim_dir, cur_dir, w_cur = None, None, 1.0               # where the nudge was set, hand direction now, nudge weight now
    nudged, decay, rep, last_nudge = False, False, 0, 0.0   # corrected since last record / trim easing out / key-repeat count, time
    prev_c, prev_t, pend, prev_z = None, 0.0, [], None

    def resolve():
        """Re-fit all points; adopt the result unless it is implausible."""
        global th_goal, est, msg
        if len(samples) < 5:
            est = None; return
        new, err, sd, keep = solve(samples)
        if np.any(np.abs(new) > LIMITS) or err[keep].max() > MAX_RESIDUAL_DEG:
            msg = "estimate rejected as implausible; add more points spread out, or press u"
            event(f"FIT REJECTED n={len(samples)} params={np.round(new, 3).tolist()} worst_err={err[keep].max():.2f}deg")
            return
        est = (new, err, sd, keep)
        adopt = len(samples) >= ADOPT_MIN
        event(f"FIT n={len(samples)} kept={int(keep.sum())} dropped={np.where(~keep)[0].tolist()} params={np.round(new, 3).tolist()} "
              f"err_deg={np.round(err, 2).tolist()} adopted={adopt}")
        if adopt:                                      # too few points extrapolate badly: show, but do not steer by it yet
            th_goal = new

    def record(force=False):
        """Add a point from the last 0.5 s of hand data. Auto-record only when steady and somewhere new."""
        global last_rec, msg, nudged, decay
        if len(recent) < (5 if force else 8) or not (force or nudged):
            return
        if webcam and not lock:                        # every earlier run recorded unlocked points at one fixed depth
            msg = "lock depth first: palm at a measured distance, set with - / =, then press l"
            return
        if any(r[3] for r in recent):                  # barrel was stopped at a limit: the commanded angle is not where it pointed
            msg = "turret at its limit here: move the hand back toward the middle"
            return
        C = np.array([r[1] for r in recent]); c = C.mean(axis=0)
        if not force:
            if C.std(axis=0).max() > STEADY_M or len(samples) >= MAX_POINTS:
                return
            d = lambda a, b: np.degrees(np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1)))
            if not all(d(c, s[0]) > NEW_DEG or abs(c[2] - s[0][2]) > NEW_Z * c[2] for s in samples):
                return
            need = hint(samples)
            if need and len(samples) >= 5 and hint(samples + [(c, 0, 0)]) == need:   # this spot would not fill the gap
                msg = f"that area is covered: {need} (SPACE records it anyway)"
                return
        jitter = np.degrees(np.linalg.norm(C.std(axis=0)) / c[2])        # this point's own hand shake
        samples.append((c, *np.mean([r[2] for r in recent], axis=0), float(np.hypot(SIGMA_DEG, jitter))))
        event(f"RECORD #{len(samples)} {'forced' if force else 'auto'} cam={np.round(c, 3).tolist()} aim={np.round(samples[-1][1:3], 2).tolist()} "
              f"sigma={samples[-1][3]:.2f}deg hand_std_cm={np.round(C.std(axis=0) * 100, 1).tolist()} trim={np.round(trim, 1).tolist()}")
        last_rec = time.time(); msg = f"recorded point {len(samples)}"
        decay, nudged = True, False                    # ease the nudge out over ~1 s instead of snapping the barrel away
        resolve()

    try:
        while True:
            color, depth, t = cam.read()
            hand = vis.process(color, depth, t)
            c, cr, st, lu, lv, lp, mang, cmdl = None, None, "nohand", "", "", "", ("", ""), ("", "")
            if bool(hand) != was_seen:
                event("hand FOUND" if hand else "hand LOST", True); was_seen = bool(hand)
            if bool(hand and hand.n_hands > 1) != was_multi:
                was_multi = bool(hand and hand.n_hands > 1)
                event("2+ hands in view " + (f"(following {hand.center_px[0]:.0f},{hand.center_px[1]:.0f}; others {np.round(hand.others).tolist()})"
                      if was_multi else "-> back to one"), True)
            if hand:
                u, v = hand.center_px; lu, lv, lp, st = round(u, 1), round(v, 1), round(hand.palm_px, 1), "ok"
                if webcam:
                    wbuf.append(hand.palm_px)
                    zc = float(np.clip(lock[0] * lock[1] / np.median(wbuf), 0.2, 4.0)) if lock and len(wbuf) >= 3 else z
                    if lock and prev_z:                                  # palm-size depth cannot change this fast for real
                        zc = float(np.clip(zc, prev_z * (1 - Z_SLEW), prev_z * (1 + Z_SLEW)))
                    prev_z = zc if lock else None
                    c = np.array([(u - cam.intr.ppx) / cam.intr.fx * zc, (v - cam.intr.ppy) / cam.intr.fy * zc, zc])
                else:
                    c = hand.center_cam
                cr = c                                                   # logged even if rejected below
                if c is not None and c[2] < MIN_Z:                       # too close: detection is flaky, parallax is huge
                    msg = f"hand too close ({c[2]:.2f} m): keep it beyond {MIN_Z} m"; c = None; st = "close"
                if c is not None and prev_c is not None and t - prev_t < 0.3 and np.linalg.norm(c - prev_c) > JUMP_M:
                    pend = pend + [c] if pend and np.linalg.norm(c - pend[-1]) < JUMP_M else [c]
                    if len(pend) >= 3:                                   # it stayed there: the hand really moved
                        pend = []
                    else:                                                # one-off teleport = detection glitch: ignore it
                        event(f"GLITCH ignored: jumped {np.linalg.norm(c - prev_c):.2f} m to px ({u:.0f},{v:.0f}) palm {hand.palm_px:.0f}px "
                              f"hands={hand.n_hands} others={np.round(hand.others).tolist()} z {c[2]:.2f}", True)
                        c = None; st = "glitch"
                elif c is not None:
                    pend = []
                if c is not None:
                    prev_c, prev_t = c, t
                    yaw, pitch = (a[0] for a in predict(th, c[None]))
                    cur_dir = c / np.linalg.norm(c); w_cur = trim_weight(cur_dir, trim_dir)
                    want = np.array([yaw, pitch]) + trim * w_cur
                    cmd = np.clip(want, LO, HI); at_limit = bool(np.any(cmd != want))
                    if last_cmd is not None and t - last_seen < 0.15 and np.abs(cmd - last_cmd).max() > 6:
                        event(f"JUMP aim moved {np.round(cmd - last_cmd, 1).tolist()} deg in one frame: px ({u:.0f},{v:.0f}) palm {hand.palm_px:.0f}px "
                              f"hands={hand.n_hands} others={np.round(hand.others).tolist()} z {c[2]:.2f} trim={np.round(trim, 1).tolist()}", True)
                    mang, cmdl = (round(yaw, 2), round(pitch, 2)), tuple(np.round(cmd, 2))
                    last_cmd, last_seen = cmd, t
                    hw.write(f"A,{round(cmd[0] * 100)},{round(cmd[1] * 100)},0\n".encode())
                    recent = [r for r in recent if t - r[0] < 0.5] + [(t, c, cmd, at_limit)]
                cv2.circle(color, (int(u), int(v)), 6, (255, 0, 0), -1)
                for o in hand.others:                                            # a second hand: shown red, not followed
                    cv2.circle(color, (int(o[0]), int(o[1])), 12, (0, 0, 255), 2)
            holding = c is None and last_cmd is not None and t - last_seen < HOLD_S
            if holding:                                                          # hand lost briefly: hold the aim, do not re-center
                hw.write(f"A,{round(last_cmd[0] * 100)},{round(last_cmd[1] * 100)},0\n".encode())
                st += "+hold"
            if holding != was_hold:
                event(("holding last aim " + str(np.round(last_cmd, 1).tolist())) if holding else
                      "stopped holding (firmware re-centers about 1 s after the last command)", True)
                was_hold = holding
            if decay:
                trim *= 0.9
                if np.abs(trim).max() < 0.05:
                    trim[:] = 0; decay = False
            th = th + BLEND * (th_goal - th)                                     # ease into a new estimate instead of jumping
            if auto and c is not None and time.time() - last_key > 0.8 and time.time() - last_rec > 1.0:
                record()
            kept = int(est[3].sum()) if est else 0
            logw.writerow([f"{t:.3f}", hand.n_hands if hand else 0, lu, lv, lp, *(np.round(cr, 4).tolist() if cr is not None else ["", "", ""]),
                           st, *mang, *np.round(trim, 1).tolist(), *cmdl, int(at_limit), len(samples), kept, *np.round(th, 4).tolist()])
            good = est is not None and kept >= 10 and est[2][5:7].max() < 0.025 and est[2][:2].max() < 0.04 \
                and est[2][2:5].max() < 0.03 and hint(samples) is None
            lines = [f"points {len(samples)} (used {kept})  trim yaw {trim[0]:+.1f} pitch {trim[1]:+.1f}"
                     + (f"  z {c[2]:.2f} m {'[palm-size]' if lock else '[manual]'}" if webcam and c is not None else "")
                     + ("  auto" if auto else "  manual"), msg]
            if est:
                p, _, sd, _ = est
                lines.append(f"lens scale x{np.exp(p[0]):.2f} y{np.exp(p[1]):.2f}  cam rot {np.degrees(p[2:5]).round(1)} deg")
                lines.append(f"offset x{p[5]:+.3f} y{p[6]:+.3f} z{p[7]:+.3f} m  +-{sd[5]*100:.1f}/{sd[6]*100:.1f}/{sd[7]*100:.1f} cm")
            lines.append("LOCK DEPTH FIRST: palm at a measured distance, set with -/=, press l" if webcam and not lock else
                         "GOOD: press c to save" if good else (hint(samples) or "add a few more points"))
            lines.append("click window | a/d w/s nudge  -/= dist  l lock  SPACE rec  u undo  r auto  c save  q quit")
            for i, s in enumerate(lines):
                cv2.putText(color, s, (10, 24 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.imshow("calibrate", color)
            k = cv2.waitKey(1) & 0xFF
            ch = chr(k) if k != 255 else ""
            step = 2.0 if ch.isupper() else 0.5
            if k == ord("q"):
                break
            elif ch.lower() in ("a", "d", "s", "w"):
                rep = rep + 1 if time.time() - last_nudge < 0.4 else 0
                step *= min(6.0, 1 + 0.25 * rep)                                 # holding a key speeds up
                last_nudge, nudged, decay = time.time(), True, False
                trim *= w_cur; trim_dir = cur_dir                                # keep what applies here, set the nudge at this spot
                trim += step * np.array([{"a": -1, "d": 1}.get(ch.lower(), 0), {"s": -1, "w": 1}.get(ch.lower(), 0)])
                last_key = time.time(); event(f"NUDGE {ch} trim={np.round(trim, 1).tolist()}")
            elif k in (ord("-"), ord("=")) and webcam:
                dz = 0.05 * (1 if k == ord("=") else -1)
                if lock: lock = (max(0.2, lock[0] + dz), lock[1])
                else: z = max(0.2, z + dz)
                event(f"DEPTH key {chr(k)}: manual z={z:.2f} locked={lock}")
            elif ch == "l" and webcam:
                lock = None if lock else ((z, float(np.median(wbuf))) if len(wbuf) >= 3 else None)
                event(f"LOCK key: locked={lock} (manual z={z:.2f})")
            elif ch == "r":
                auto = not auto
            elif k == ord(" "):
                record(force=True)
            elif ch == "u" and samples:
                samples.pop(); msg = "undid last point"; event(f"UNDO -> {len(samples)} points"); resolve()
            elif ch == "c" and est and not good and time.time() - last_c > 3.0:
                last_c = time.time()
                msg = f"NOT GOOD yet: {hint(samples) or 'uncertainty still high'}. Press c again within 3 s to save anyway"
                event("SAVE asked for confirmation: " + msg, True)
            elif ch == "c" and est:
                p, err, sd, keep = est
                loo = loo_deg(samples)
                print(f"lens scale x{np.exp(p[0]):.3f} y{np.exp(p[1]):.3f}, camera rotation {np.degrees(p[2:5]).round(2)} deg, "
                      f"offset {p[5:8].round(3)} m, +- cm {np.round(sd[5:8] * 100, 1)}\n"
                      f"error per point (deg) {np.round(err, 2)}, dropped {np.where(~keep)[0].tolist()}\n"
                      f"leave-one-out error: mean {loo.mean():.2f} deg, worst {loo.max():.2f} deg (held-out accuracy)")
                json.dump({"camera_offset_t": [round(float(x), 4) for x in p[5:8]],
                           "camera_scale": [round(float(np.exp(x)), 4) for x in p[:2]],
                           "camera_rot": [round(float(x), 5) for x in p[2:5]]}, open(CALIB, "w"))
                msg = f"saved calib.json (held-out error {loo.mean():.1f} deg)"; print("saved", CALIB)
                event(f"SAVED good={good} points={len(samples)} params={np.round(p, 3).tolist()} sd={np.round(sd, 3).tolist()} "
                      f"loo_mean={loo.mean():.2f}deg", True)
    finally:
        hw.close(); logf.close(); evf.close()
        print(f"logs: {logf.name} and {evf.name}")
