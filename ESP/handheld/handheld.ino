// Handheld IMU pointer. Reads a BNO055 or BNO085, turns its orientation into turret aim angles (aim_math.h) and sends
// them to the turret ESP over ESP-NOW (imu_link.h), 50 times a second. Wiring: IMU 3V3 / GND / SDA = GPIO21 / SCL = GPIO22.
//
// Power it on holding the wand still, pointing straight ahead (toward where the turret faces): that direction becomes
// yaw 0 after 1.5 s. Pitch needs no zeroing (gravity). Drift is measured and removed while you hold still.
// USB serial (115200): prints status 5x a second; send 'z' to re-zero.
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "imu_link.h"
#include "aim_math.h"

#ifndef IMU_TYPE
#define IMU_TYPE 55                 // 55 = BNO055, 85 = BNO085 (or compile with -DIMU_TYPE=85)
#endif
#ifndef USE_MAGNETOMETER
#define USE_MAGNETOMETER 1        // 1: heading anchored to the magnetometer (no long-term drift, but magnets/motors nearby bend it)
                                    // 0: gyro-only heading (immune to magnets, slow drift that the drift tracker removes)
#endif

// ---- Set these during bring-up ----
const int   SDA_PIN = 21, SCL_PIN = 22;
const float POINT_AXIS[3] = {1, 0, 0};   // which sensor axis points at the target (board X by default; change to match your mounting)
const int   YAW_SIGN = -1;               // turning right must give + yaw; flip to +1 if it is reversed
const int   PITCH_SIGN = +1;             // tilting up must give + pitch; flip to -1 if it is reversed
const int   SEND_HZ = 50;
const int   RECENTER_PIN = -1;           // optional: a button to GND here re-zeros (-1 = none; 'z' over USB always works)

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
aim::Pointer ptr;

// ---------------------------------------------------------------- IMU glue: one begin() and one read() per sensor type
#if IMU_TYPE == 55
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Preferences.h>
Adafruit_BNO055 bno(55, 0x28, &Wire);    // 0x29 if the ADR pin is pulled high
Preferences prefs;
bool offsetsSaved = false;

bool imuBegin() {
  if (!bno.begin(USE_MAGNETOMETER ? OPERATION_MODE_NDOF : OPERATION_MODE_IMUPLUS)) return false;
  adafruit_bno055_offsets_t off;
  prefs.begin("bno055", false);
  if (prefs.getBytes("off", &off, sizeof(off)) == sizeof(off)) bno.setSensorOffsets(off);   // calibration kept from last time
  bno.setExtCrystalUse(true);
  return true;
}

bool imuRead(aim::Quat &q, bool &calibrated) {
  imu::Quaternion v = bno.getQuat();
  double n = v.w() * v.w() + v.x() * v.x() + v.y() * v.y() + v.z() * v.z();
  if (n < 0.5) return false;                                                 // not producing orientation yet
  q = aim::Quat{v.w(), v.x(), v.y(), v.z()};
  static unsigned long lastCal = 0; static bool cal = false;
  if (millis() - lastCal > 500) {
    lastCal = millis(); cal = bno.isFullyCalibrated();
    if (cal && !offsetsSaved) {                                              // save once per boot when fully calibrated
      adafruit_bno055_offsets_t off;
      if (bno.getSensorOffsets(off)) { prefs.putBytes("off", &off, sizeof(off)); offsetsSaved = true; Serial.println("IMU calibration saved"); }
    }
  }
  calibrated = cal;
  return true;
}

#elif IMU_TYPE == 85
#include <Adafruit_BNO08x.h>
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sv;
#if USE_MAGNETOMETER
#define REPORT SH2_ROTATION_VECTOR
#else
#define REPORT SH2_GAME_ROTATION_VECTOR
#endif

bool imuBegin() {
  if (!bno08x.begin_I2C()) return false;                                     // address 0x4A
  return bno08x.enableReport(REPORT, 10000);                                 // 100 Hz
}

bool imuRead(aim::Quat &q, bool &calibrated) {
  if (bno08x.wasReset()) bno08x.enableReport(REPORT, 10000);
  bool got = false;
  static bool cal = false;
  while (bno08x.getSensorEvent(&sv)) {                                       // drain the queue, keep the newest
    if (sv.sensorId != REPORT) continue;
#if USE_MAGNETOMETER
    q = aim::Quat{sv.un.rotationVector.real, sv.un.rotationVector.i, sv.un.rotationVector.j, sv.un.rotationVector.k};
#else
    q = aim::Quat{sv.un.gameRotationVector.real, sv.un.gameRotationVector.i, sv.un.gameRotationVector.j, sv.un.gameRotationVector.k};
#endif
    cal = (sv.status & 0x03) >= 2;                                           // the chip's own accuracy estimate, 0..3
    got = true;
  }
  calibrated = cal;
  return got;
}
#else
#error "IMU_TYPE must be 55 (BNO055) or 85 (BNO085)"
#endif

// ----------------------------------------------------------------
double yaw = 0, pitch = 0;
bool valid = false, calibrated = false;
unsigned long lastFeed = 0, nextSend = 0, lastImuOk = 0, lastPrint = 0;
uint16_t seq = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  memcpy(ptr.axis, POINT_AXIS, sizeof(ptr.axis));
  ptr.yawSign = YAW_SIGN;
  if (RECENTER_PIN >= 0) pinMode(RECENTER_PIN, INPUT_PULLUP);
  while (!imuBegin()) { Serial.println("IMU not found: check wiring (SDA 21, SCL 22, 3V3, GND) and the I2C address"); delay(1000); }
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); while (true) delay(1000); }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = LINK_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.println("Ready: hold still, pointing straight ahead, for 2 s");
}

void loop() {
  unsigned long now = millis();
  if (Serial.available() && Serial.read() == 'z') { ptr.rezero(); valid = false; Serial.println("re-zeroing: hold still"); }
  if (RECENTER_PIN >= 0 && digitalRead(RECENTER_PIN) == LOW) { ptr.rezero(); valid = false; delay(300); }

  if (now - lastFeed >= 1000 / SEND_HZ) {                                    // feed the filters at a steady 50 Hz
    lastFeed = now;
    aim::Quat q; bool cal;
    if (imuRead(q, cal)) {
      lastImuOk = now; calibrated = cal;
      double y, p;
      valid = ptr.update(now * 0.001, q, &y, &p);
      if (valid) { yaw = y; pitch = PITCH_SIGN * p; }
    }
  }

  if (now >= nextSend) {
    nextSend = now + 1000 / SEND_HZ;
    LinkPacket pk = {};
    pk.seq = seq++;
    pk.flags = ((valid && now - lastImuOk < 200) ? LINK_FLAG_VALID : 0) | (calibrated ? LINK_FLAG_CALIBRATED : 0);
    pk.yaw_cdeg = (int16_t)lroundf((float)constrain(yaw, -180.0, 180.0) * 100);
    pk.pitch_cdeg = (int16_t)lroundf((float)constrain(pitch, -180.0, 180.0) * 100);
    linkSeal(pk);
    esp_now_send(BCAST, (const uint8_t *)&pk, sizeof(pk));
  }

  if (now - lastPrint >= 200) {
    lastPrint = now;
    if (valid) Serial.printf("yaw %7.2f  pitch %7.2f  cal %d  drift %+.3f deg/s\n", yaw, pitch, calibrated, ptr.drift.rate);
    else Serial.println(ptr.ready() ? "IMU stalled" : "waiting: hold still");
  }
}
