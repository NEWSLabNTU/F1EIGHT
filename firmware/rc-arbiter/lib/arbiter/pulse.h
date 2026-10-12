// One captured pulse channel, and what the arbiter reads from it.
#pragma once

#include <stdint.h>

#include "config.h"

namespace arbiter {

// What the capture layer reports for one input.
struct Pulse {
  uint16_t width_us = 0;  // width of the latest complete pulse
  uint32_t age_ms = UINT32_MAX;  // time since that pulse ended
  uint32_t seq = 0;  // increments on every complete pulse
};

// A channel is valid when its latest pulse is recent and in range.
bool pulse_valid(const Pulse& p);

// Signed distance from a centre, in us.
int32_t offset_us(const Pulse& p, uint16_t centre_us);

// A stick within cfg::kCentredUs of the receiver centre.
bool stick_centred(const Pulse& p);

// A stick past cfg::kTakeoverUs from the receiver centre.
bool stick_moved(const Pulse& p);

// Normalised throttle in [-1, 1]: positive drives, negative brakes, zero
// inside the deadband.
float throttle(const Pulse& p, const cfg::ThrottleMap& map);

}  // namespace arbiter
