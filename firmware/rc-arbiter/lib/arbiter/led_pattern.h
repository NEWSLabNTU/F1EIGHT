// The LED colour for each mode, as a function of time so the blink is
// testable. The table is in docs/design/rc-override-arbiter.md, "Modes".
#pragma once

#include <stdint.h>

#include "arbiter.h"

namespace arbiter {

struct Rgb {
  uint8_t r, g, b;
};

inline bool operator==(const Rgb& a, const Rgb& b) {
  return a.r == b.r && a.g == b.g && a.b == b.b;
}

constexpr Rgb kOff = {0, 0, 0};
constexpr Rgb kGreen = {0, 255, 0};
constexpr Rgb kAmber = {255, 120, 0};
constexpr Rgb kRed = {255, 0, 0};
constexpr Rgb kDimRed = {40, 0, 0};

inline Rgb led_pattern(Mode m, uint32_t now_ms) {
  bool on_1hz = (now_ms % 1000) < 500;
  bool on_4hz = (now_ms % 250) < 125;
  switch (m) {
    case Mode::Manual: return kOff;
    case Mode::Arming: return on_1hz ? kGreen : kOff;
    case Mode::Auto: return kGreen;
    case Mode::DisengagedTakeover: return on_4hz ? kAmber : kOff;
    case Mode::DisengagedOrinLost: return on_4hz ? kRed : kDimRed;
  }
  return kOff;
}

}  // namespace arbiter
