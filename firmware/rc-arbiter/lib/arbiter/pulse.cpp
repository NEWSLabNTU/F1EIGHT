#include "pulse.h"

namespace arbiter {

bool pulse_valid(const Pulse& p) {
  return p.seq != 0 && p.age_ms <= cfg::kPulseTimeoutMs && p.width_us >= cfg::kPulseMinUs &&
         p.width_us <= cfg::kPulseMaxUs;
}

int32_t offset_us(const Pulse& p, uint16_t centre_us) {
  return static_cast<int32_t>(p.width_us) - static_cast<int32_t>(centre_us);
}

static uint32_t abs_offset(const Pulse& p) {
  int32_t d = offset_us(p, cfg::kRxCentreUs);
  return static_cast<uint32_t>(d < 0 ? -d : d);
}

bool stick_centred(const Pulse& p) {
  return pulse_valid(p) && abs_offset(p) <= cfg::kCentredUs;
}

bool stick_moved(const Pulse& p) {
  return pulse_valid(p) && abs_offset(p) > cfg::kTakeoverUs;
}

float throttle(const Pulse& p, const cfg::ThrottleMap& map) {
  int32_t d = offset_us(p, map.centre_us);
  if (d > -static_cast<int32_t>(map.deadband_us) && d < static_cast<int32_t>(map.deadband_us)) {
    return 0.0f;
  }
  float t;
  if (d > 0) {
    t = static_cast<float>(d) / static_cast<float>(map.forward_full_us - map.centre_us);
  } else {
    t = static_cast<float>(d) / static_cast<float>(map.centre_us - map.brake_full_us);
  }
  if (t > 1.0f) return 1.0f;
  if (t < -1.0f) return -1.0f;
  return t;
}

}  // namespace arbiter
