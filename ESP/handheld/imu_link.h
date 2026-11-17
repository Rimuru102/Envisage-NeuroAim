// Radio packet from the handheld IMU unit to the turret (ESP-NOW).
// This file is identical in ESP/handheld and ESP/turret: edit one, copy to the other (tests/run_tests.sh checks they match).
#pragma once
#include <stdint.h>

#define LINK_CHANNEL 1          // both ESPs must use the same WiFi channel
#define LINK_MAGIC0  0xA5
#define LINK_MAGIC1  0x5A
#define LINK_KEY     0x6B       // shared secret mixed into the checksum; change on BOTH sides to pair differently

struct __attribute__((packed)) LinkPacket {
  uint8_t  magic0, magic1;
  uint16_t seq;                 // +1 per packet; the receiver drops old or repeated ones
  int16_t  yaw_cdeg;            // aim yaw in centidegrees, + = right (same convention as the "A," line)
  int16_t  pitch_cdeg;          // aim pitch in centidegrees, + = up
  uint8_t  flags;               // bit0: orientation valid (zeroed and fresh), bit1: IMU fully calibrated
  uint8_t  crc;                 // CRC-8 (poly 0x07) of the bytes above, seeded with LINK_KEY
};

static const uint8_t LINK_FLAG_VALID = 1, LINK_FLAG_CALIBRATED = 2;

inline uint8_t linkCrc(const uint8_t *d, int n) {
  uint8_t c = LINK_KEY;
  for (int i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

inline void linkSeal(LinkPacket &p) {
  p.magic0 = LINK_MAGIC0; p.magic1 = LINK_MAGIC1;
  p.crc = linkCrc((const uint8_t *)&p, sizeof(LinkPacket) - 1);
}

// True only for a packet of exactly the right size, with the right magic bytes and checksum.
inline bool linkValid(const uint8_t *d, int len) {
  if (len != (int)sizeof(LinkPacket)) return false;
  const LinkPacket *p = (const LinkPacket *)d;
  return p->magic0 == LINK_MAGIC0 && p->magic1 == LINK_MAGIC1 && p->crc == linkCrc(d, len - 1);
}
