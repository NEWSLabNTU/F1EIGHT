// VESC UART packets, built into a byte buffer with no dependency on Arduino,
// so the host tests can check them byte for byte.
//
// Format, from vedderb/bldc comm/packet.c (short form, payload up to 255):
//   0x02, length, payload..., crc16 high, crc16 low, 0x03
// with the CRC over the payload only (CRC-16/XMODEM: poly 0x1021, init 0;
// util/crc.c). Command ids and scaling are from datatypes.h and
// comm/commands.c: currents are int32 milliamps, big-endian. Every one of
// these commands resets the VESC's UART timeout.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace vesc {

enum Command : uint8_t {
  COMM_SET_CURRENT = 6,
  COMM_SET_CURRENT_BRAKE = 7,
  COMM_ALIVE = 30,
};

// Longest packet built here: 5 payload bytes plus 5 bytes of framing.
constexpr size_t kMaxPacket = 10;

uint16_t crc16(const uint8_t* data, size_t len);

// Each returns the packet length written to out (at least kMaxPacket bytes).
size_t set_current(float amps, uint8_t* out);
size_t set_brake_current(float amps, uint8_t* out);
size_t alive(uint8_t* out);

}  // namespace vesc
