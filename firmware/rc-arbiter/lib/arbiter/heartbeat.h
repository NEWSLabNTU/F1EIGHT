// The Orin heartbeat on PCA9685 ch2: a pulse width that alternates between
// 1200 us and 1800 us at 20 Hz. Alive means it changed recently.
#pragma once

#include <stdint.h>

#include "pulse.h"

namespace arbiter {

class Heartbeat {
 public:
  // Feed the latest heartbeat pulse once per arbiter step.
  void update(const Pulse& p, uint32_t now_ms);

  // A toggle within cfg::kHeartbeatTimeoutMs.
  bool alive(uint32_t now_ms) const;

 private:
  uint32_t seen_seq_ = 0;
  int8_t level_ = -1;  // -1 unknown, 0 low, 1 high
  bool toggled_ = false;
  uint32_t last_toggle_ms_ = 0;
};

}  // namespace arbiter
