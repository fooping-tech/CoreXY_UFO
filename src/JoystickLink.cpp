#include "JoystickLink.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <stdio.h>
#include <string.h>
#include "PlotterConfig.h"

namespace {
portMUX_TYPE link_mux = portMUX_INITIALIZER_UNLOCKED;
JoyCommandPacket latest_packet{};
uint32_t latest_ms = 0;
bool has_packet = false;
bool peer_locked = false;
uint8_t peer_mac[6] = {};

bool macIsZero(const uint8_t mac[6]) {
  for (int i = 0; i < 6; ++i) {
    if (mac[i] != 0) return false;
  }
  return true;
}

// Runs in the Wi-Fi task. Validate and store only.
void onReceive(const uint8_t* mac, const uint8_t* data, int length) {
  JoyCommandPacket packet{};
  if (mac == nullptr || length < 0 ||
      !joyPacketValid(data, static_cast<size_t>(length), packet)) {
    return;
  }
  portENTER_CRITICAL(&link_mux);
  if (!peer_locked) {
    // JOY_ALLOWED_MAC all zero: pair with the first Atom heard after boot.
    if (macIsZero(JOY_ALLOWED_MAC) || memcmp(mac, JOY_ALLOWED_MAC, 6) == 0) {
      memcpy(peer_mac, mac, 6);
      peer_locked = true;
    }
  }
  if (peer_locked && memcmp(mac, peer_mac, 6) == 0) {
    latest_packet = packet;
    latest_ms = millis();
    has_packet = true;
  }
  portEXIT_CRITICAL(&link_mux);
}
}  // namespace

void formatMac(const uint8_t mac[6], char* out) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3],
           mac[4], mac[5]);
}

bool JoystickLink::begin() {
  // Enabled in SIMULATION_MODE too: the radio link moves nothing by itself.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  // Low latency: no modem sleep between packets.
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (esp_wifi_set_channel(JOY_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    return false;
  }
  if (esp_now_init() != ESP_OK) return false;
  if (esp_now_register_recv_cb(onReceive) != ESP_OK) return false;
  ready_ = true;
  return true;
}

bool JoystickLink::latest(JoyCommandPacket& packet, uint32_t& age_ms) const {
  portENTER_CRITICAL(&link_mux);
  const bool available = has_packet;
  packet = latest_packet;
  const uint32_t received_ms = latest_ms;
  portEXIT_CRITICAL(&link_mux);
  if (!available) return false;
  age_ms = millis() - received_ms;
  return true;
}

bool JoystickLink::paired() const {
  portENTER_CRITICAL(&link_mux);
  const bool locked = peer_locked;
  portEXIT_CRITICAL(&link_mux);
  return locked;
}

void JoystickLink::pairedMac(uint8_t mac[6]) const {
  portENTER_CRITICAL(&link_mux);
  memcpy(mac, peer_mac, 6);
  portEXIT_CRITICAL(&link_mux);
}

bool JoystickLink::sendStatus(JoyStatusPacket& packet) {
  if (!ready_ || !paired()) return false;
  uint8_t mac[6];
  pairedMac(mac);
  if (!peer_added_) {
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = JOY_ESPNOW_CHANNEL;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;
    if (esp_now_add_peer(&peer) != ESP_OK) return false;
    peer_added_ = true;
  }
  joyFinalizePacket(packet);
  return esp_now_send(mac, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet)) ==
         ESP_OK;
}

void JoystickLink::localMac(uint8_t mac[6]) { esp_wifi_get_mac(WIFI_IF_STA, mac); }
