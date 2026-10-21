#pragma once

#include <stdint.h>
#include "JoystickProtocol.h"

// Core 0 owned ESP-NOW link to the Atom JoyStick (simple test input only;
// production commands come over USB Serial). The receive callback only
// validates and stores the latest packet; joystickTask consumes it.
class JoystickLink {
 public:
  bool begin();
  bool ready() const { return ready_; }
  // Latest valid packet and its age. false if none was received yet.
  bool latest(JoyCommandPacket& packet, uint32_t& age_ms) const;
  bool paired() const;
  void pairedMac(uint8_t mac[6]) const;
  bool sendStatus(JoyStatusPacket& packet);
  static void localMac(uint8_t mac[6]);

 private:
  bool ready_ = false;
  bool peer_added_ = false;
};

// Formats a MAC as "AA:BB:CC:DD:EE:FF" into out (>= 18 bytes).
void formatMac(const uint8_t mac[6], char* out);
