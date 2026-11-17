// Handheld pointer math: IMU quaternion -> filtered turret aim (yaw, pitch). Plain C++ (no Arduino), so tests/ can run it on a PC.
//
//   pointing direction  rotate the sensor's "forward" axis by the quaternion; world frame is Z up (BNO055 and BNO085 both are)
//   zero                at power-up, hold still: the heading held for 1.5 s becomes "straight ahead" (yaw 0). Pitch is gravity-referenced.
//   drift               while the wand is held still, any slow change in heading is drift: measure it, keep subtracting it
//   noise               One Euro filter (heavy smoothing when slow, light when fast) + a small output hysteresis
#pragma once
#include <math.h>

namespace aim {

const double RAD = 3.14159265358979323846 / 180.0;

struct Quat { double w, x, y, z; };

inline double wrap180(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

// Angle (deg) of the rotation that takes a to b. The atan2 form stays accurate for very small angles.
inline double angleBetween(const Quat &a, const Quat &b) {
  double rw = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;                 // r = conj(a) * b
  double rx = a.w * b.x - a.x * b.w - a.y * b.z + a.z * b.y;
  double ry = a.w * b.y + a.x * b.z - a.y * b.w - a.z * b.x;
  double rz = a.w * b.z - a.x * b.y + a.y * b.x - a.z * b.w;
  return 2.0 * atan2(sqrt(rx * rx + ry * ry + rz * rz), fabs(rw)) / RAD;
}

// World direction of the sensor-frame axis v, then its heading (deg, counter-clockwise from world +X seen from above)
// and elevation (deg, + up).
inline void pointing(const Quat &q, const float v[3], double *headingDeg, double *elevDeg) {
  double tx = 2 * (q.y * v[2] - q.z * v[1]), ty = 2 * (q.z * v[0] - q.x * v[2]), tz = 2 * (q.x * v[1] - q.y * v[0]);
  double dx = v[0] + q.w * tx + (q.y * tz - q.z * ty);
  double dy = v[1] + q.w * ty + (q.z * tx - q.x * tz);
  double dz = v[2] + q.w * tz + (q.x * ty - q.y * tx);
  *headingDeg = atan2(dy, dx) / RAD;
  *elevDeg = atan2(dz, sqrt(dx * dx + dy * dy)) / RAD;
}

// One Euro filter: cutoff rises with speed, so a still hand is smooth and a fast sweep keeps up.
struct OneEuro {
  double mincut = 0.8, beta = 0.12, dcut = 1.0;           // Hz, Hz per (deg/s), Hz
  double x = 0, dx = 0; bool init = false;
  static double alpha(double cutoff, double dt) { double tau = 1.0 / (2 * 3.14159265358979323846 * cutoff); return 1.0 / (1.0 + tau / dt); }
  double filter(double v, double dt) {
    if (!init) { init = true; x = v; dx = 0; return v; }
    dx += alpha(dcut, dt) * ((v - x) / dt - dx);
    x += alpha(mincut + beta * fabs(dx), dt) * (v - x);
    return x;
  }
};

// True when the last `window` seconds of orientation all lie within `thr` degrees of the newest sample.
// Feed at 60 Hz or less (the buffer holds 64 samples).
struct StillDetector {
  static const int N = 64;
  double t[N]; Quat q[N]; int head = 0, count = 0;
  double window = 1.0, thr = 0.8;
  void push(double now, const Quat &qq) { t[head] = now; q[head] = qq; head = (head + 1) % N; if (count < N) count++; }
  bool still(double now) const {
    if (count < 2) return false;
    const Quat &newest = q[(head - 1 + N) % N];
    double oldest = now;
    for (int i = 1; i < count; i++) {
      int k = (head - 1 - i + 2 * N) % N;
      if (now - t[k] > window) break;
      if (angleBetween(newest, q[k]) > thr) return false;
      oldest = t[k];
    }
    return now - oldest >= 0.9 * window;
  }
};

// Heading drift. While still, the heading should not change, so its slow change IS the drift. Measure it over `window`
// seconds, average the measurements, and keep subtracting it. A moving wand is never used to learn.
struct DriftTracker {
  double window = 3.0, alpha = 0.3, maxRate = 0.5;        // s, averaging weight per window, deg/s limit
  double rate = 0, acc = 0;                               // learned drift (deg/s), correction applied so far (deg)
  bool have = false, inWin = false;
  double prevRaw = 0, unw = 0, lastT = 0, t0 = 0, y0 = 0;
  void update(double t, double raw, bool still) {
    if (!have) { have = true; prevRaw = raw; unw = raw; lastT = t; }
    else { unw += wrap180(raw - prevRaw); prevRaw = raw; acc += rate * (t - lastT); lastT = t; }
    if (!still) { inWin = false; return; }
    if (!inWin) { inWin = true; t0 = t; y0 = unw; return; }
    if (t - t0 >= window) {
      rate += alpha * ((unw - y0) / (t - t0) - rate);
      if (rate > maxRate) rate = maxRate; else if (rate < -maxRate) rate = -maxRate;
      t0 = t; y0 = unw;
    }
  }
  double correct(double raw) const { return raw - acc; }
};

struct Pointer {
  float axis[3] = {1, 0, 0};          // which sensor axis points at the target (set to match how the board is mounted)
  int yawSign = -1;                   // turning right (clockwise) must give + yaw; flip to +1 if it is reversed
  double zeroStillS = 1.5, hyst = 0.08;
  StillDetector sd; DriftTracker drift; OneEuro fy, fp;
  enum State { WAIT_STILL, RUN } state = WAIT_STILL;
  double heading0 = 0, sumS = 0, sumC = 0, stillSince = -1, lastT = -1, lastRel = 0;
  double outYaw = 0, outPitch = 0; bool outInit = false;

  bool ready() const { return state == RUN; }
  void rezero() { state = WAIT_STILL; stillSince = -1; sumS = sumC = 0; }

  // One orientation sample at time t (s). Returns true once zeroed, with the filtered aim in degrees.
  bool update(double t, const Quat &q, double *yaw, double *pitch) {
    sd.push(t, q);
    double heading, elev;
    pointing(q, axis, &heading, &elev);
    bool still = sd.still(t);
    if (state == WAIT_STILL) {
      if (!still) { stillSince = -1; sumS = sumC = 0; return false; }
      if (stillSince < 0) stillSince = t;
      sumS += sin(heading * RAD); sumC += cos(heading * RAD);
      if (t - stillSince < zeroStillS) return false;
      heading0 = atan2(sumS, sumC) / RAD;                  // circular mean of the still period
      state = RUN; drift = DriftTracker(); fy = OneEuro(); fp = OneEuro(); lastT = -1; lastRel = 0; outInit = false;
    }
    double dt = lastT < 0 ? 0.02 : t - lastT;
    lastT = t;
    if (dt < 0.001) dt = 0.001; else if (dt > 0.1) dt = 0.1;
    double rel = fabs(elev) > 80 ? lastRel : yawSign * wrap180(heading - heading0);   // straight up/down: heading is undefined, hold
    lastRel = rel;
    drift.update(t, rel, still);
    double ys = fy.filter(drift.correct(rel), dt), ps = fp.filter(elev, dt);
    if (!outInit || fabs(ys - outYaw) > hyst) outYaw = ys;                               // ignore sub-0.08 deg wiggles
    if (!outInit || fabs(ps - outPitch) > hyst) outPitch = ps;
    outInit = true;
    *yaw = outYaw; *pitch = outPitch;
    return true;
  }
};

}  // namespace aim
