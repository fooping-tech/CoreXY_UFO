#pragma once

#include <stddef.h>
#include <stdint.h>

// ESP-NOW packets between the Atom JoyStick (AtomS3) and the Core2.
// Shared by both firmwares; keep the layout stable and bump the version on
// any change.

constexpr uint32_t JOY_PROTOCOL_MAGIC = 0x4A4F4655;  // bytes "UFOJ"
// v3: status carries X/Y position. v4: 5 speed levels (3-bit field, MAX added),
// left button = HOME (was TEST_MODE toggle), status HOMING / UNHOMED_JOG flags.
constexpr uint8_t JOY_PROTOCOL_VERSION = 4;
constexpr int16_t JOY_AXIS_FULL_SCALE = 1000;
// Wi-Fi channel used by both sides (no access point is joined).
constexpr uint8_t JOY_ESPNOW_CHANNEL = 1;

enum JoyAxisIndex : uint8_t {
  JOY_AXIS_LEFT_X = 0,   // XY: +X
  JOY_AXIS_LEFT_Y = 1,   // XY: +Y
  JOY_AXIS_RIGHT_X = 2,  // arm (servo ID 2)
  JOY_AXIS_RIGHT_Y = 3,  // Z drum (servo ID 1)
  JOY_AXIS_COUNT = 4,
};

// JoyCommandPacket::flags bits 0-2: XY speed level (left stick press cycles
// SLOW -> FAST -> VERY_FAST -> ULTRA -> MAX -> SLOW).
enum JoySpeedLevel : uint8_t {
  JOY_SPEED_SLOW = 0,
  JOY_SPEED_FAST = 1,
  JOY_SPEED_VERY_FAST = 2,
  JOY_SPEED_ULTRA = 3,
  JOY_SPEED_MAX = 4,
  JOY_SPEED_LEVEL_COUNT = 5,
};
constexpr uint8_t JOY_FLAG_SPEED_MASK = 0x07;

// Speed level from packet flags; out-of-range values count as the top level.
inline uint8_t joySpeedLevel(uint8_t flags) {
  const uint8_t level = flags & JOY_FLAG_SPEED_MASK;
  return level < JOY_SPEED_LEVEL_COUNT ? level : JOY_SPEED_LEVEL_COUNT - 1;
}

inline const char* joySpeedLevelName(uint8_t level) {
  switch (joySpeedLevel(level)) {
    case JOY_SPEED_SLOW: return "SLOW";
    case JOY_SPEED_FAST: return "FAST";
    case JOY_SPEED_VERY_FAST: return "VERY FAST";
    case JOY_SPEED_ULTRA: return "ULTRA";
    default: return "MAX";
  }
}

// Atom -> Core2, sent at 50 Hz.
// Button actions are counters, not edges: the Atom increments a counter per
// press and repeats it in every packet, so a lost packet cannot drop or
// duplicate an action. The Core2 acts when a counter changes.
struct __attribute__((packed)) JoyCommandPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t flags;
  uint16_t seq;
  int16_t axes[JOY_AXIS_COUNT];  // -1000..1000 after centering and deadzone
  uint8_t estop_count;
  uint8_t home_count;  // left button long press -> HOME
  uint8_t alarm_clear_count;
  uint8_t reserved;
  uint16_t battery_mv;
  uint16_t crc;
};

// JoyStatusPacket::flags
constexpr uint8_t JOY_STATUS_TEST_MODE = 0x01;
constexpr uint8_t JOY_STATUS_ALARM = 0x02;
constexpr uint8_t JOY_STATUS_HOMED = 0x04;
constexpr uint8_t JOY_STATUS_JOG_ACTIVE = 0x08;
constexpr uint8_t JOY_STATUS_TMC_READY = 0x10;
constexpr uint8_t JOY_STATUS_HOMING = 0x20;
// Jog without HOME: XY is slow and has no soft limit.
constexpr uint8_t JOY_STATUS_UNHOMED_JOG = 0x40;

// Core2 -> Atom, sent at 10 Hz to the paired Atom.
struct __attribute__((packed)) JoyStatusPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t flags;
  uint16_t ack_seq;
  // Position estimate [mm] from the home origin. Stays valid after test-mode
  // moves clear HOMED (it keeps counting steps); not valid before any HOME.
  float x_mm;
  float y_mm;
  char last_result[40];
  uint16_t crc;
};

// CRC-16/CCITT-FALSE over the packet without its trailing crc field.
inline uint16_t joyCrc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

template <typename Packet>
void joyFinalizePacket(Packet& packet) {
  packet.magic = JOY_PROTOCOL_MAGIC;
  packet.version = JOY_PROTOCOL_VERSION;
  packet.crc = joyCrc16(reinterpret_cast<const uint8_t*>(&packet),
                        sizeof(Packet) - sizeof(packet.crc));
}

template <typename Packet>
bool joyPacketValid(const uint8_t* data, size_t length, Packet& packet) {
  if (length != sizeof(Packet)) return false;
  const Packet* incoming = reinterpret_cast<const Packet*>(data);
  if (incoming->magic != JOY_PROTOCOL_MAGIC ||
      incoming->version != JOY_PROTOCOL_VERSION) {
    return false;
  }
  if (joyCrc16(data, sizeof(Packet) - sizeof(incoming->crc)) != incoming->crc) {
    return false;
  }
  packet = *incoming;
  return true;
}
