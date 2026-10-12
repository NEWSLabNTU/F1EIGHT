// The arbiter: who drives, given the switch, the receiver, the Orin and the
// heartbeat. Pure logic with no clock and no hardware, so the tests can drive
// it through any scenario in simulated time.
//
// Modes and transitions follow docs/design/rc-override-arbiter.md, "Modes".
#pragma once

#include <stdint.h>

#include "debounce.h"
#include "heartbeat.h"
#include "pulse.h"

namespace arbiter {

enum class Mode : uint8_t {
  Manual,              // switch OFF; joystick drives
  Arming,              // switch just turned ON; joystick drives
  Auto,                // Orin drives, joystick watched
  DisengagedTakeover,  // latched; joystick drives
  DisengagedOrinLost,  // latched; brake, then joystick
};

const char* mode_name(Mode m);

struct Inputs {
  bool switch_on = false;  // raw level from the auto-mode switch sense pin
  Pulse rx_steer;
  Pulse rx_throttle;
  Pulse orin_steer;
  Pulse orin_throttle;
  Pulse heartbeat;
};

struct VescCommand {
  enum class Kind : uint8_t { Current, Brake };
  Kind kind = Kind::Brake;
  float amps = 0.0f;
};

struct Outputs {
  Mode mode = Mode::Manual;
  uint16_t servo_us = 0;  // 0 until a valid steering source has been seen
  VescCommand vesc;
  bool auto_line = false;  // high only in AUTO
};

class Arbiter {
 public:
  Arbiter() : switch_(cfg::kSwitchDebounceMs) {}

  Outputs step(const Inputs& in, uint32_t now_ms);

  Mode mode() const { return mode_; }

 private:
  void enter(Mode m, uint32_t now_ms);
  bool radio_ok(const Inputs& in) const;
  bool orin_ok(const Inputs& in, uint32_t now_ms) const;
  void joystick_drive(const Inputs& in, Outputs& out);

  Debounce switch_;
  Heartbeat heartbeat_;
  Mode mode_ = Mode::Manual;
  bool started_ = false;
  bool prev_switch_ = false;

  uint32_t arming_since_ms_ = 0;
  bool holding_ = false;  // ARMING: heartbeat alive and sticks centred
  uint32_t holding_since_ms_ = 0;

  uint32_t rx_frame_seq_ = 0;
  uint8_t moved_frames_ = 0;

  bool throttle_released_ = false;  // Orin lost: the driver has moved the throttle
  uint16_t last_servo_us_ = 0;
};

}  // namespace arbiter
