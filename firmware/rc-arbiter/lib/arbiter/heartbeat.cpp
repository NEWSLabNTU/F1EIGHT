#include "heartbeat.h"

namespace arbiter {

void Heartbeat::update(const Pulse& p, uint32_t now_ms) {
  if (p.seq == seen_seq_) return;
  seen_seq_ = p.seq;
  if (!pulse_valid(p)) return;

  int8_t level = p.width_us >= cfg::kHeartbeatSplitUs ? 1 : 0;
  if (level_ >= 0 && level != level_) {
    toggled_ = true;
    last_toggle_ms_ = now_ms;
  }
  level_ = level;
}

bool Heartbeat::alive(uint32_t now_ms) const {
  return toggled_ && now_ms - last_toggle_ms_ <= cfg::kHeartbeatTimeoutMs;
}

}  // namespace arbiter
