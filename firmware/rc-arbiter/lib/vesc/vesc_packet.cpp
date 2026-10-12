#include "vesc_packet.h"

namespace vesc {

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static size_t frame(const uint8_t* payload, uint8_t len, uint8_t* out) {
  size_t i = 0;
  out[i++] = 0x02;
  out[i++] = len;
  for (uint8_t k = 0; k < len; ++k) out[i++] = payload[k];
  uint16_t crc = crc16(payload, len);
  out[i++] = static_cast<uint8_t>(crc >> 8);
  out[i++] = static_cast<uint8_t>(crc & 0xff);
  out[i++] = 0x03;
  return i;
}

static size_t command_milliamps(Command cmd, float amps, uint8_t* out) {
  // Round to the nearest milliamp, as the VESC divides by 1000.
  float ma = amps * 1000.0f;
  int32_t v = static_cast<int32_t>(ma >= 0.0f ? ma + 0.5f : ma - 0.5f);
  uint32_t u = static_cast<uint32_t>(v);
  uint8_t payload[5] = {
      cmd,
      static_cast<uint8_t>(u >> 24),
      static_cast<uint8_t>(u >> 16),
      static_cast<uint8_t>(u >> 8),
      static_cast<uint8_t>(u),
  };
  return frame(payload, sizeof(payload), out);
}

size_t set_current(float amps, uint8_t* out) {
  return command_milliamps(COMM_SET_CURRENT, amps, out);
}

size_t set_brake_current(float amps, uint8_t* out) {
  return command_milliamps(COMM_SET_CURRENT_BRAKE, amps, out);
}

size_t alive(uint8_t* out) {
  uint8_t payload[1] = {COMM_ALIVE};
  return frame(payload, 1, out);
}

}  // namespace vesc
