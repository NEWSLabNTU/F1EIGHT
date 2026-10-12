// XIAO RP2350 pin map, as GPIO numbers. The table with the wiring is in
// docs/design/rc-override-arbiter.md, "Wiring"; change both together.
#pragma once

#include <stdint.h>

namespace pins {

constexpr uint8_t kRxSteer = 26;       // D0, receiver steer in
constexpr uint8_t kRxThrottle = 27;    // D1, receiver throttle in
constexpr uint8_t kOrinThrottle = 5;   // D3, PCA9685 ch0 through switch pole A
constexpr uint8_t kOrinSteer = 6;      // D4, PCA9685 ch1 through switch pole B
constexpr uint8_t kHeartbeat = 7;      // D5, PCA9685 ch2
constexpr uint8_t kSwitchSense = 2;    // D8, internal pull-up, low = ON
constexpr uint8_t kVescTx = 0;         // D6, UART0 TX to VESC RX
constexpr uint8_t kVescRx = 1;         // D7, UART0 RX from VESC TX
constexpr uint8_t kServo = 4;          // D9, steering servo out
constexpr uint8_t kAutoLine = 3;       // D10, AUTO state to the Orin
constexpr uint8_t kLedData = 22;       // onboard RGB LED data
constexpr uint8_t kLedPower = 23;      // onboard RGB LED power enable
constexpr uint8_t kExtLedData = 20;    // D12, optional external LED

}  // namespace pins
