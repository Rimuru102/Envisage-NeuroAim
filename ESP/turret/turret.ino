#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_PWMServoDriver.h>
#include "imu_link.h"

// ---- Set these during bring-up ----
const int   PAN_CH = 0, TILT_CH = 1;
const float PAN_CENTER = 90, TILT_CENTER = 150;    // servo angle when centered: pan 90 forward, tilt 150 level (measured, step 3)
const int   PAN_SIGN = -1,   TILT_SIGN = -1;       // both flipped in step 3 (pan went left, tilt went down)
const float PAN_MIN = 35,    PAN_MAX = 125;        // measured stops 30..130, 5 deg inside (step 4)
const float TILT_MIN = 105,  TILT_MAX = 160;       // measured stops 100..165, 5 deg inside (step 4)
const int   US_MIN = 500,    US_MAX = 2500;        // DS3218MG pulse range
const float SERVO_RANGE_DEG = 180;                 // DS3218MG 180-degree version
const unsigned long WATCHDOG_MS = 1000;

// ---- Handheld IMU unit (ESP-NOW) ----
const unsigned long IMU_TIMEOUT_MS = 300;          // no packet this long = handheld lost: hold, then ease home after WATCHDOG_MS
const float MAX_SLEW_DPS = 400;                    // fastest the barrel may move while following the handheld

Adafruit_PWMServoDriver pwm(0x40);
unsigned long lastCmd = 0;
bool autoMode = false;                             // laptop ("A,") control, with its watchdog

float yawLo, yawHi, pitchLo, pitchHi;              // aim range the servos can reach, computed in setup from the limits above
float curYaw = 0, curPitch = 0;                    // handheld mode: slewed output
float tgtYaw = 0, tgtPitch = 0;                    // handheld mode: where the handheld points
bool imuMode = false;
unsigned long lastImuTick = 0, lastImuMs = 0;      // last output step, arrival of the newest valid packet

portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
LinkPacket rxPkt;                                  // newest accepted packet, written by the radio callback
volatile bool rxNew = false;
volatile unsigned long rxMs = 0;
uint16_t rxLastSeq = 0;
bool rxHaveSeq = false;
uint32_t rxCount = 0, rxDropped = 0;               // diagnostics only (the "L" command)

void servoWrite(int ch, float deg, float lo, float hi) {
  deg = constrain(deg, lo, hi);
  pwm.writeMicroseconds(ch, US_MIN + (US_MAX - US_MIN) * deg / SERVO_RANGE_DEG);
}

void aimWrite(float yaw, float pitch) {
  servoWrite(PAN_CH,  PAN_CENTER  + PAN_SIGN  * yaw,   PAN_MIN,  PAN_MAX);
  servoWrite(TILT_CH, TILT_CENTER + TILT_SIGN * pitch, TILT_MIN, TILT_MAX);
}

bool imuFresh() { return lastImuMs != 0 && millis() - lastImuMs < IMU_TIMEOUT_MS; }

// Radio callback (runs in the WiFi task): validate, drop old or repeated packets, keep the newest.
static void handlePacket(const uint8_t *data, int len) {
  if (!linkValid(data, len)) { rxDropped++; return; }
  LinkPacket p;
  memcpy(&p, data, sizeof(p));
  unsigned long now = millis();
  portENTER_CRITICAL(&rxMux);
  bool fresh = !rxHaveSeq || (int16_t)(p.seq - rxLastSeq) > 0 || now - rxMs > IMU_TIMEOUT_MS;   // a rebooted handheld restarts at 0
  if (fresh) { rxPkt = p; rxLastSeq = p.seq; rxHaveSeq = true; rxMs = now; rxNew = true; rxCount++; }
  else rxDropped++;
  portEXIT_CRITICAL(&rxMux);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { handlePacket(data, len); }
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) { handlePacket(data, len); }
#endif

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  aimWrite(0, 0);                       // pan 90, tilt 150: forward and level
  float a = (PAN_MIN - PAN_CENTER) / PAN_SIGN, b = (PAN_MAX - PAN_CENTER) / PAN_SIGN;
  yawLo = fminf(a, b); yawHi = fmaxf(a, b);
  a = (TILT_MIN - TILT_CENTER) / TILT_SIGN; b = (TILT_MAX - TILT_CENTER) / TILT_SIGN;
  pitchLo = fminf(a, b); pitchHi = fmaxf(a, b);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() == ESP_OK) esp_now_register_recv_cb(onRecv);
  Serial.println("Ready");
}

void loop() {
  // 1. newest packet from the handheld
  if (rxNew) {
    LinkPacket p; unsigned long ms;
    portENTER_CRITICAL(&rxMux); p = rxPkt; ms = rxMs; rxNew = false; portEXIT_CRITICAL(&rxMux);
    lastImuMs = ms;
    if (p.flags & LINK_FLAG_VALID) {
      tgtYaw = constrain(p.yaw_cdeg / 100.0f, yawLo, yawHi);
      tgtPitch = constrain(p.pitch_cdeg / 100.0f, pitchLo, pitchHi);
      if (!imuMode) lastImuTick = millis();
      imuMode = true; autoMode = false;   // the handheld is in charge; its own timeout handles silence
    }
  }
  // 2. laptop commands (ignored while the handheld is live: it wins)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    long a, b, f;
    if (sscanf(line.c_str(), "A,%ld,%ld,%ld", &a, &b, &f) == 3) {   // f ignored: no fire hardware
      if (!imuFresh()) {
        aimWrite(a / 100.0, b / 100.0);
        curYaw = constrain(a / 100.0f, yawLo, yawHi); curPitch = constrain(b / 100.0f, pitchLo, pitchHi);
        imuMode = false; autoMode = true; lastCmd = millis();
      }
    } else if (sscanf(line.c_str(), "S,%ld,%ld", &a, &b) == 2) {     // raw servo angles
      if (!imuFresh()) {
        servoWrite(PAN_CH, a, PAN_MIN, PAN_MAX);
        servoWrite(TILT_CH, b, TILT_MIN, TILT_MAX);
        curYaw = constrain((a - PAN_CENTER) / PAN_SIGN, yawLo, yawHi);
        curPitch = constrain((b - TILT_CENTER) / TILT_SIGN, pitchLo, pitchHi);
        imuMode = false; autoMode = false;
      }
    } else if (line.startsWith("L")) {                               // link status: ms since last packet, seq, accepted, dropped
      Serial.printf("L,%lu,%u,%lu,%lu\n", lastImuMs ? millis() - lastImuMs : 0xFFFFFFFFUL, rxLastSeq, (unsigned long)rxCount, (unsigned long)rxDropped);   // (the counters are not atomic: fine for a status line)
    }
  }
  // 3. handheld mode: step toward the target at a limited speed
  unsigned long now = millis();
  if (imuMode && now - lastImuTick >= 10) {
    float dt = min((now - lastImuTick) * 0.001f, 0.05f);
    lastImuTick = now;
    float step = MAX_SLEW_DPS * dt;
    curYaw += constrain(tgtYaw - curYaw, -step, step);
    curPitch += constrain(tgtPitch - curPitch, -step, step);
    aimWrite(curYaw, curPitch);
    if (!imuFresh() && now - lastImuMs > WATCHDOG_MS) {              // handheld gone: ease home, then leave handheld mode
      tgtYaw = 0; tgtPitch = 0;
      if (fabsf(curYaw) < 0.05f && fabsf(curPitch) < 0.05f) imuMode = false;
    }
  }
  if (autoMode && millis() - lastCmd > WATCHDOG_MS) {   // laptop went quiet: re-center
    aimWrite(0, 0);
    curYaw = curPitch = 0;
    autoMode = false;
  }
}
