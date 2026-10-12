// A level that must hold for a while before it counts.
#pragma once

#include <stdint.h>

namespace arbiter {

class Debounce {
 public:
  explicit Debounce(uint32_t hold_ms) : hold_ms_(hold_ms) {}

  // Feed the raw level. Returns the debounced level. The first sample is taken
  // as the stable level at once and reports no edge, so a switch that is
  // already ON at power-up does not look like it was just turned ON.
  bool update(bool raw, uint32_t now_ms) {
    if (!started_) {
      started_ = true;
      stable_ = raw;
      candidate_ = raw;
      since_ms_ = now_ms;
      return stable_;
    }
    if (raw != candidate_) {
      candidate_ = raw;
      since_ms_ = now_ms;
    }
    if (candidate_ != stable_ && now_ms - since_ms_ >= hold_ms_) {
      stable_ = candidate_;
    }
    return stable_;
  }

  bool level() const { return stable_; }

 private:
  uint32_t hold_ms_;
  bool started_ = false;
  bool stable_ = false;
  bool candidate_ = false;
  uint32_t since_ms_ = 0;
};

}  // namespace arbiter
