#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const int PAN_CH  = 0;
const int TILT_CH = 1;

// Pulse range in microseconds. Narrow these if a servo buzzes at its ends.
const int US_MIN = 500;
const int US_MAX = 2500;

// Safe mechanical limits for your turret (adjust later during calibration)
const int PAN_MIN = 0,  PAN_MAX = 180;
const int TILT_MIN = 0, TILT_MAX = 180;

void setServo(int channel, int angle) {
  int us = map(angle, 0, 180, US_MIN, US_MAX);
  pwm.writeMicroseconds(channel, us);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);                 // SDA = GPIO21, SCL = GPIO22
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);                 // 50 Hz for analog servos
  delay(10);
  setServo(PAN_CH, 90);               // start centered
  setServo(TILT_CH, 90);
  Serial.println("Ready");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    int x, y;
    if (sscanf(line.c_str(), "X%dY%d", &x, &y) == 2) {
      x = constrain(x, PAN_MIN, PAN_MAX);
      y = constrain(y, TILT_MIN, TILT_MAX);
      setServo(PAN_CH, x);
      setServo(TILT_CH, y);
    }
  }
}