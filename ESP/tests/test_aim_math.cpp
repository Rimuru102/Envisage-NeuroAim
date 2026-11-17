// Host-side tests for the handheld math and the radio packet. Build and run with tests/run_tests.sh
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>
#include "../handheld/aim_math.h"
#include "../handheld/imu_link.h"
using namespace aim;

// Orientation whose sensor +X axis points at heading h (deg, counter-clockwise from above) and elevation e (deg, up).
static Quat pose(double h, double e) {
  double a = h * RAD / 2, b = -e * RAD / 2;
  Quat z{cos(a), 0, 0, sin(a)}, y{cos(b), 0, sin(b), 0};
  return Quat{z.w * y.w - z.x * y.x - z.y * y.y - z.z * y.z, z.w * y.x + z.x * y.w + z.y * y.z - z.z * y.y,
              z.w * y.y - z.x * y.z + z.y * y.w + z.z * y.x, z.w * y.z + z.x * y.y - z.y * y.x + z.z * y.w};
}

struct Sim {
  Pointer p; std::mt19937 rng{7}; std::normal_distribution<double> n{0, 1};
  double t = 0, yaw = 0, pitch = 0; bool ok = false;
  // hold or move to (h, e) with sensor noise sigma (deg) and a heading drift (deg/s), for `secs` seconds at 50 Hz
  std::vector<double> run(double h, double e, double secs, double sigma = 0.0, double drift = 0.0) {
    std::vector<double> ys; double h0 = hh, e0 = ee; int steps = (int)(secs * 50);
    for (int i = 1; i <= steps; i++) {
      t += 0.02; hh = h0 + (h - h0) * i / steps; ee = e0 + (e - e0) * i / steps; dh += drift * 0.02;
      ok = p.update(t, pose(hh + dh + sigma * n(rng), ee + sigma * n(rng)), &yaw, &pitch);
      ys.push_back(yaw);
    }
    return ys;
  }
  double hh = 40, ee = 10, dh = 0;
};
static double stdev(const std::vector<double> &v, size_t from) {
  double m = 0, s = 0; size_t n = v.size() - from;
  for (size_t i = from; i < v.size(); i++) m += v[i];
  m /= n;
  for (size_t i = from; i < v.size(); i++) s += (v[i] - m) * (v[i] - m);
  return sqrt(s / n);
}

int main() {
  // geometry sanity: the helpers agree with each other
  Quat a = pose(40, 10), b = pose(40, 10.5);
  assert(fabs(angleBetween(a, b) - 0.5) < 1e-6);
  double hd, el; float fwd[3] = {1, 0, 0}; pointing(pose(33, -12), fwd, &hd, &el);
  assert(fabs(hd - 33) < 1e-9 && fabs(el + 12) < 1e-9);

  { // same convention as the Adafruit BNO055 / BNO08x libraries: their yaw formula must equal our heading, their pitch our -elevation
    std::mt19937 g(3); std::normal_distribution<double> n(0, 1);
    for (int i = 0; i < 1000; i++) {
      Quat q{n(g), n(g), n(g), n(g)}; double m = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z); q = {q.w / m, q.x / m, q.y / m, q.z / m};
      double libYaw = atan2(2 * (q.x * q.y + q.z * q.w), q.x * q.x - q.y * q.y - q.z * q.z + q.w * q.w) / RAD;
      double libPitch = asin(-2 * (q.x * q.z - q.y * q.w)) / RAD;
      pointing(q, fwd, &hd, &el);
      assert(fabs(wrap180(hd - libYaw)) < 1e-6 && fabs(el + libPitch) < 1e-6);
    }
  }
  { // zeroing: waits for stillness, then the held heading is straight ahead whatever it was
    Sim s; s.p.yawSign = -1; auto y = s.run(40, 10, 1.0, 0.05); assert(!s.ok);                       // not zeroed yet
    y = s.run(40, 10, 2.5, 0.05); assert(s.ok && fabs(s.yaw) < 0.3 && fabs(s.pitch - 10) < 0.3);
  }
  { // directions and step response
    Sim s; s.run(40, 10, 3.0, 0.05);
    s.run(10, 10, 1.0, 0.05); s.run(10, 10, 1.0, 0.05); assert(fabs(s.yaw - 30) < 1.0);              // turned RIGHT 30 deg: yaw +30
    s.run(10, 40, 1.0, 0.05); s.run(10, 40, 1.0, 0.05); assert(fabs(s.pitch - 40) < 1.0);            // tilted UP: pitch +40
    s.run(70, 40, 1.0, 0.05); s.run(70, 40, 1.0, 0.05); assert(s.yaw < -29);                         // turned LEFT: yaw negative
    Sim f; f.run(40, 10, 3.0, 0.05); f.run(10, 10, 0.1, 0.05);                                       // 30 deg flick in 0.1 s
    auto tail = f.run(10, 10, 0.3, 0.05); assert(f.yaw > 26);                                        // follows within 0.3 s
  }
  { // noise: a still wand with 0.15 deg of sensor noise gives a nearly constant output
    Sim s; s.run(40, 10, 3.0, 0.15); auto y = s.run(40, 10, 10.0, 0.15);
    std::vector<double> raw; Sim r; std::normal_distribution<double> g(0, 0.15); for (int i = 0; i < 500; i++) raw.push_back(g(r.rng) * 1.0);
    printf("  noise: raw std %.3f deg -> output std %.3f deg\n", stdev(raw, 0), stdev(y, 0));
    assert(stdev(y, 0) < stdev(raw, 0) / 3);
  }
  { // drift: 0.2 deg/s of heading drift while held still is learned and removed
    Sim s; s.run(40, 10, 3.0, 0.05);
    auto y = s.run(40, 10, 60.0, 0.05, 0.2); double raw60 = 0.2 * 60;
    printf("  drift: raw would be %.1f deg after 60 s, output %.2f deg, learned %.3f deg/s\n", raw60, fabs(s.yaw), fabs(s.p.drift.rate));
    assert(fabs(s.yaw) < 3.0 && fabs(fabs(s.p.drift.rate) - 0.2) < 0.06);
    s.run(40, 10, 60.0, 0.05, 0.2); assert(fabs(s.yaw) < 3.0);                                       // stays bounded, not growing
  }
  { // moving never teaches drift: wave +-30 deg at 0.5 Hz, then hold
    Sim s; s.run(40, 10, 3.0, 0.05);
    for (int i = 0; i < 10; i++) { s.run(70, 10, 1.0, 0.05); s.run(10, 10, 1.0, 0.05); }
    assert(fabs(s.p.drift.rate) < 0.05);
  }
  { // pointing straight up: heading is undefined, yaw must hold instead of spinning
    Sim s; s.run(40, 10, 3.0, 0.05); s.run(10, 10, 1.0, 0.05); double y0 = s.yaw; s.run(10, 88, 1.0, 0.05);
    auto y = s.run(200, 88.5, 1.0, 0.05); assert(fabs(s.yaw - y0) < 3.0);
  }
  { // rezero(): pointing elsewhere and holding makes that the new straight ahead
    Sim s; s.run(40, 10, 3.0, 0.05); s.run(90, 10, 1.0, 0.05); s.p.rezero(); s.run(90, 10, 3.0, 0.05); assert(s.ok && fabs(s.yaw) < 0.5);
  }
  { // radio packet: size, round trip, and every corruption caught
    assert(sizeof(LinkPacket) == 10);
    LinkPacket p{}; p.seq = 1234; p.yaw_cdeg = -3550; p.pitch_cdeg = 4500; p.flags = 3; linkSeal(p);
    uint8_t buf[sizeof(LinkPacket)]; memcpy(buf, &p, sizeof(p));
    assert(linkValid(buf, sizeof(buf)) && !linkValid(buf, sizeof(buf) - 1));
    for (size_t i = 0; i < sizeof(buf); i++) for (int bit = 0; bit < 8; bit++) { uint8_t c[sizeof(buf)]; memcpy(c, buf, sizeof(c)); c[i] ^= 1 << bit; assert(!linkValid(c, sizeof(c))); }
    LinkPacket q; memcpy(&q, buf, sizeof(q)); assert(q.seq == 1234 && q.yaw_cdeg == -3550 && q.pitch_cdeg == 4500);
  }
  printf("aim_math + imu_link OK\n");
}
