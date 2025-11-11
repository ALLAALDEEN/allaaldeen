/// @file comm_protocol.h
/// Shared definitions for the RF link between the RC transmitter and the flight
/// controller. All payloads must remain <= 32 bytes to fit inside a single
/// NRF24L01 packet.
#pragma once

#include <Arduino.h>

namespace drone
{

// Radio configuration -------------------------------------------------------

static constexpr uint8_t RADIO_PIPE_ADDRESS[6] = {'D', 'R', 'N', 'E', '1'};
static constexpr uint8_t RADIO_ACK_PIPE_ADDRESS[6] = {'D', 'R', 'N', 'E', '2'};
static constexpr uint8_t RADIO_CHANNEL = 92;          // 2.492 GHz
static constexpr uint8_t RADIO_DATARATE = 1;          // 1Mbps (RF24_1MBPS)
static constexpr uint8_t RADIO_TX_POWER = 3;          // RF24_PA_HIGH
static constexpr uint8_t RADIO_PAYLOAD_SIZE = 32;

// Control flags -------------------------------------------------------------

enum ActionFlags : uint8_t
{
    ACTION_NONE = 0,
    ACTION_REQUEST_CALIBRATION = 1 << 0,   ///< Button 1 pressed: calibrate IMU & ESCs.
    ACTION_REQUEST_IDLE_SPIN = 1 << 1,     ///< Button 2 pressed: smooth spool.
    ACTION_ACKNOWLEDGE_STEP = 1 << 2,      ///< Reserved for future handshake acks.
};

enum ArmMode : uint8_t
{
    ARM_MODE_KILL = 0,     ///< Motors guaranteed off.
    ARM_MODE_SAFE = 1,     ///< Ready to arm (toggle switch armed position).
};

// Telemetry flags -----------------------------------------------------------

enum StatusFlags : uint8_t
{
    STATUS_NONE = 0,
    STATUS_CONNECTED = 1 << 0,
    STATUS_ARMED = 1 << 1,
    STATUS_MOTORS_SPINNING = 1 << 2,
    STATUS_CALIBRATING = 1 << 3,
    STATUS_FAILSAFE = 1 << 4,
    STATUS_ESC_CALIBRATED = 1 << 5,
    STATUS_IMU_READY = 1 << 6,
};

enum GuidePhase : uint8_t
{
    GUIDE_RADIO_LINK = 0,
    GUIDE_SWITCH_KILL = 1,
    GUIDE_CALIBRATE_PROMPT = 2,
    GUIDE_CALIBRATING = 3,
    GUIDE_ARM_PROMPT = 4,
    GUIDE_IDLE_SPIN_PROMPT = 5,
    GUIDE_READY_TO_FLY = 6,
};

// Payload definitions -------------------------------------------------------

struct ControlFrame
{
    uint16_t throttle;          ///< 1000-2000 us (1000 = motors off)
    int16_t roll;               ///< -500 .. +500 us
    int16_t pitch;              ///< -500 .. +500 us
    int16_t yaw;                ///< -500 .. +500 us
    uint8_t arm_mode;           ///< ArmMode
    uint8_t action_flags;       ///< ActionFlags bitfield (edge triggered)
    uint8_t sequence;           ///< Rolling counter for link supervision
    uint8_t reserved;           ///< Alignment / future use
    uint16_t checksum;          ///< CRC16-CCITT over preceding bytes
} __attribute__((packed));
static_assert(sizeof(ControlFrame) <= RADIO_PAYLOAD_SIZE, "ControlFrame too large");

struct TelemetryFrame
{
    int16_t roll;               ///< deg * 100
    int16_t pitch;              ///< deg * 100
    int16_t yaw;                ///< deg * 100
    int16_t vertical_velocity;  ///< cm/s
    uint16_t throttle;          ///< last commanded value (us)
    uint16_t altitude_cm;       ///< Estimated altitude (cm)
    uint16_t battery_mv;        ///< Battery voltage in millivolts
    uint8_t status_flags;       ///< StatusFlags bitfield
    uint8_t guide_phase;        ///< GuidePhase hint for transmitter UI
    uint8_t rf_channel;         ///< Active NRF channel index
    uint8_t sequence;           ///< Rolling counter for ordering
    uint16_t checksum;          ///< CRC16-CCITT over preceding bytes
} __attribute__((packed));
static_assert(sizeof(TelemetryFrame) <= RADIO_PAYLOAD_SIZE, "TelemetryFrame too large");

// Helper utilities ---------------------------------------------------------

namespace detail
{
    constexpr uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data)
    {
        crc ^= static_cast<uint16_t>(data) << 8;
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
        return crc;
    }
} // namespace detail

template <typename T>
uint16_t computeCrc(const T &payload)
{
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&payload);
    constexpr size_t payload_size = sizeof(T) - sizeof(uint16_t);
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < payload_size; ++i)
    {
        crc = detail::crc16_ccitt_update(crc, raw[i]);
    }
    return crc;
}

template <typename T>
void finalizeFrame(T &payload)
{
    payload.checksum = computeCrc(payload);
}

template <typename T>
bool validateFrame(const T &payload)
{
    return computeCrc(payload) == payload.checksum;
}

inline uint16_t constrainMicroseconds(int32_t value)
{
    return static_cast<uint16_t>(constrain(value, 1000, 2000));
}

inline int16_t applyDeadband(int16_t value, int16_t deadband)
{
    if (abs(value) < deadband)
    {
        return 0;
    }
    return value;
}

} // namespace drone

