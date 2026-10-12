// VESC packets byte for byte. The CRC check value is the published
// CRC-16/XMODEM one; the framing and scaling follow vedderb/bldc
// comm/packet.c and comm/commands.c.
#include <unity.h>

#include "vesc_packet.h"

void setUp() {}
void tearDown() {}

static void test_crc_check_value() {
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x31C3, vesc::crc16(data, sizeof(data)));
}

static void test_set_current_framing() {
  uint8_t buf[vesc::kMaxPacket];
  size_t n = vesc::set_current(1.5f, buf);
  // 1.5 A is 1500 mA = 0x000005DC.
  const uint8_t payload[] = {6, 0x00, 0x00, 0x05, 0xDC};
  uint16_t crc = vesc::crc16(payload, sizeof(payload));
  const uint8_t expected[] = {0x02, 5, 6, 0x00, 0x00, 0x05, 0xDC,
                              static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc), 0x03};
  TEST_ASSERT_EQUAL_UINT(sizeof(expected), n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, buf, n);
}

static void test_brake_current_rounds_to_milliamps() {
  uint8_t buf[vesc::kMaxPacket];
  vesc::set_brake_current(20.0004f, buf);
  TEST_ASSERT_EQUAL_HEX8(7, buf[2]);
  // 20000 mA = 0x00004E20
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x4E, buf[5]);
  TEST_ASSERT_EQUAL_HEX8(0x20, buf[6]);
}

static void test_negative_current_is_twos_complement() {
  uint8_t buf[vesc::kMaxPacket];
  vesc::set_current(-1.0f, buf);
  // -1000 mA = 0xFFFFFC18
  TEST_ASSERT_EQUAL_HEX8(0xFF, buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0xFC, buf[5]);
  TEST_ASSERT_EQUAL_HEX8(0x18, buf[6]);
}

static void test_alive() {
  uint8_t buf[vesc::kMaxPacket];
  size_t n = vesc::alive(buf);
  const uint8_t payload[] = {30};
  uint16_t crc = vesc::crc16(payload, 1);
  const uint8_t expected[] = {0x02, 1, 30, static_cast<uint8_t>(crc >> 8),
                              static_cast<uint8_t>(crc), 0x03};
  TEST_ASSERT_EQUAL_UINT(sizeof(expected), n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, buf, n);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_crc_check_value);
  RUN_TEST(test_set_current_framing);
  RUN_TEST(test_brake_current_rounds_to_milliamps);
  RUN_TEST(test_negative_current_is_twos_complement);
  RUN_TEST(test_alive);
  return UNITY_END();
}
