// rc_protocol.h
// Shared packet definitions for the DIY drone project.
// Designed to keep flight controller and RC transmitter in sync.

#pragma once

#include <Arduino.h>

namespace rc {

// Update version when protocol changes.
static constexpr uint16_t kProtocolVersion = 1;
static constexpr uint32_t kPacketMagic = 0x52433130; // "RC10"

struct ControlPacket {
  uint32_t magic = kPacketMagic;
  uint16_t version = kProtocolVersion;
  int16_t throttle = 1000;
  int16_t roll = 0;
  int16_t pitch = 0;
  int16_t yaw = 0;
  int16_t aux1 = 0;
  int16_t aux2 = 0;
  uint16_t switches = 0;
  uint16_t buttons = 0;
  uint16_t dial1 = 0;
  uint16_t dial2 = 0;
  uint16_t checksum = 0;
};

struct TelemetryPacket {
  uint32_t magic = kPacketMagic;
  uint16_t version = kProtocolVersion;
  int16_t roll_deg = 0;
  int16_t pitch_deg = 0;
  int16_t yaw_rate_dps = 0;
  int16_t altitude_cm = 0;
  uint16_t status_flags = 0;
  uint16_t battery_mv = 0;
  uint16_t checksum = 0;
};

inline uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

inline void finalize(ControlPacket& packet) {
  packet.checksum = 0;
  packet.checksum =
      crc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(ControlPacket));
}

inline bool validate(const ControlPacket& packet) {
  if (packet.magic != kPacketMagic || packet.version != kProtocolVersion) {
    return false;
  }
  uint16_t expected =
      crc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(ControlPacket));
  return expected == packet.checksum;
}

inline void finalize(TelemetryPacket& packet) {
  packet.checksum = 0;
  packet.checksum =
      crc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(TelemetryPacket));
}

inline bool validate(const TelemetryPacket& packet) {
  if (packet.magic != kPacketMagic || packet.version != kProtocolVersion) {
    return false;
  }
  uint16_t expected =
      crc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(TelemetryPacket));
  return expected == packet.checksum;
}

}  // namespace rc
