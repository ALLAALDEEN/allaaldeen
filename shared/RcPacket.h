#pragma once

#include <stdint.h>
#include <stddef.h>

struct __attribute__((packed)) RcPacket {
  uint32_t sequence;
  uint16_t throttle;   // 0..1000
  int16_t roll;        // -500..500
  int16_t pitch;       // -500..500
  int16_t yaw;         // -500..500
  int16_t aux1;        // -500..500 (custom)
  int16_t aux2;        // -500..500 (custom)
  uint8_t buttons;     // bitmask buttons
  uint8_t switches;    // bitmask switches / stick presses
  uint16_t checksum;
};

inline uint16_t computeChecksum(const RcPacket &packet) {
  const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&packet);
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(RcPacket) - sizeof(packet.checksum); ++i) {
    sum += ptr[i];
  }
  return sum;
}
