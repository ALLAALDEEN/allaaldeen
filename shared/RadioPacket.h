#pragma once

#include <stdint.h>

namespace DroneLink {

// Simple packet header/check bytes for validation.
constexpr uint16_t kPacketMagic = 0xA55A;

// 5-byte addresses for nRF24L01 (LSByte first as used by RF24).
constexpr uint64_t kRadioAddress = 0xE8E8F0F0E1ULL;

// Each packet carries a monotonic counter that allows the receiver to detect stale data.
struct __attribute__((packed)) RadioPacket {
  uint16_t magic;          // Must be kPacketMagic
  uint16_t throttle;       // 0..1000
  int16_t roll;            // -500..500
  int16_t pitch;           // -500..500
  int16_t yaw;             // -500..500
  uint16_t switches;       // bit0: SW1, bit1: SW2, bit2-5: buttons 1..4, bit6: left stick press, bit7: right stick press
  uint16_t knobs[2];       // 0..1023 (POT_1, POT_2)
  uint16_t batteryMv;      // Optional transmitter battery measurement (set to 0 if unused)
  uint16_t frameId;        // Incrementing counter
  uint16_t checksum;       // Simple checksum over previous fields
};

inline uint16_t computeChecksum(const RadioPacket &packet) {
  const uint16_t *ptr = reinterpret_cast<const uint16_t *>(&packet);
  constexpr size_t words = sizeof(RadioPacket) / sizeof(uint16_t) - 1;
  uint32_t sum = 0;
  for (size_t i = 0; i < words; ++i) {
    sum += ptr[i];
  }
  // Fold down to 16 bits
  while (sum >> 16) {
    sum = (sum & 0xFFFF) + (sum >> 16);
  }
  return static_cast<uint16_t>(~sum);
}

inline void finalizePacket(RadioPacket &packet) {
  packet.magic = kPacketMagic;
  packet.checksum = 0;
  packet.checksum = computeChecksum(packet);
}

inline bool validatePacket(const RadioPacket &packet) {
  if (packet.magic != kPacketMagic) {
    return false;
  }
  return computeChecksum(packet) == packet.checksum;
}

}  // namespace DroneLink

