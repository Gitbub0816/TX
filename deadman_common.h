#pragma once

#include <Arduino.h>

// Shared definitions for the wireless deadman switch system used by both TX and RX.
// Keep this file dependency-light and deterministic for Arduino builds.

// Compile-time defaults that may be overridden via NVS configuration at runtime.
constexpr uint32_t DEFAULT_PAIR_ID   = 0xA5B6C7D8;  // Unique per TX/RX pair
constexpr uint32_t DEFAULT_SECRET_KEY = 0x1F2E3D4C; // Shared secret for MAC calculation

// Packet structure sent over the air. Packed to avoid padding differences.
#pragma pack(push, 1)
struct DeadmanPacket {
  uint32_t pairId;       // Pairing ID used to match TX/RX pairs
  uint16_t counter;      // Rolling counter incremented on every TX packet
  uint8_t  flags;        // bit0: trigger pressed, others reserved for future use
  uint16_t batteryMv;    // Transmitter battery voltage in millivolts
  uint16_t mac;          // Authentication MAC/CRC16 over fields above using SECRET_KEY
};
#pragma pack(pop)

static_assert(sizeof(DeadmanPacket) <= 32, "DeadmanPacket must fit inside nRF24 payload");

// Simple CRC16-CCITT (0x1021) implementation with optional seed for MAC generation.
static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t seed = 0xFFFF) {
  uint16_t crc = seed;
  for (size_t i = 0; i < len; ++i) {
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

// Computes a lightweight MAC over the packet fields (excluding the mac itself).
// The secret key is folded into the CRC seed to add a shared-key dependency.
static inline uint16_t computeMac(const DeadmanPacket &pkt, uint32_t secretKey) {
  uint16_t seed = (uint16_t)(secretKey ^ (secretKey >> 16));
  return crc16_ccitt(reinterpret_cast<const uint8_t *>(&pkt), offsetof(DeadmanPacket, mac), seed);
}

static inline bool validateMac(const DeadmanPacket &pkt, uint32_t secretKey) {
  return computeMac(pkt, secretKey) == pkt.mac;
}

// Utility to determine whether a new counter value is plausibly fresh compared to the last seen
// counter, accounting for 16-bit wraparound. Values that are far older are rejected to prevent replay.
static inline bool isCounterTooOld(uint16_t last, uint16_t current, uint16_t window) {
  // If no prior value has been recorded, accept by default.
  if (last == 0xFFFF) {
    return false;
  }
  // If current is newer or equal (within half-range), it's not old.
  if ((uint16_t)(current - last) < 0x8000) {
    return false;
  }
  // Otherwise the current value is behind last. Reject if it's outside the replay window.
  uint16_t delta = (uint16_t)(last - current);
  return delta > window;
}
