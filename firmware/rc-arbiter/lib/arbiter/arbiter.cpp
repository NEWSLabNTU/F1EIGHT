#include "arbiter.h"

namespace arbiter {

const char* mode_name(Mode m) {
  switch (m) {
    case Mode::Manual: return "MANUAL";
    case Mode::Arming: return "ARMING";
    case Mode::Auto: return "AUTO";
    case Mode::DisengagedTakeover: return "DISENGAGED_TAKEOVER";
    case Mode::DisengagedOrinLost: return "DISENGAGED_ORIN_LOST";
  }
  return "?";
}

static VescCommand from_throttle(float t, float max_current_a) {
  VescCommand c;
  if (t >= 0.0f) {
    c.kind = VescCommand::Kind::Current;
    c.amps = t * max_current_a;
  } else {
    c.kind = VescCommand::Kind::Brake;
    c.amps = -t * cfg::kMaxBrakeCurrentA;
  }
  return c;
}

static VescCommand hold_brake() {
  VescCommand c;
  c.kind = VescCommand::Kind::Brake;
  c.amps = cfg::kHoldBrakeCurrentA;
  return c;
}

void Arbiter::enter(Mode m, uint32_t now_ms) {
  mode_ = m;
  switch (m) {
    case Mode::Arming:
      arming_since_ms_ = now_ms;
      holding_ = false;
      break;
    case Mode::Auto:
      moved_frames_ = 0;
      break;
    case Mode::DisengagedOrinLost:
      throttle_released_ = false;
      break;
    default:
      break;
  }
}

bool Arbiter::radio_ok(const Inputs& in) const {
  return pulse_valid(in.rx_steer) && pulse_valid(in.rx_throttle);
}

bool Arbiter::orin_ok(const Inputs& in, uint32_t now_ms) const {
  return heartbeat_.alive(now_ms) && pulse_valid(in.orin_steer) && pulse_valid(in.orin_throttle);
}

void Arbiter::joystick_drive(const Inputs& in, Outputs& out) {
  if (!radio_ok(in)) {
    // Nobody can steer or throttle: brake and leave the wheels where they are.
    out.vesc = hold_brake();
    return;
  }
  last_servo_us_ = in.rx_steer.width_us;
  out.vesc = from_throttle(throttle(in.rx_throttle, cfg::kRxThrottle), cfg::kRxMaxCurrentA);
}

Outputs Arbiter::step(const Inputs& in, uint32_t now_ms) {
  // ---- Inputs that carry state ---------------------------------------------
  bool sw = switch_.update(in.switch_on, now_ms);
  if (!started_) {
    // A switch already ON at power-up is not an OFF to ON edge.
    started_ = true;
    prev_switch_ = sw;
  }
  bool switched_on = sw && !prev_switch_;
  prev_switch_ = sw;

  heartbeat_.update(in.heartbeat, now_ms);

  // The steer channel is the receiver's frame clock: one new steer pulse is
  // one frame, so a takeover needs kTakeoverFrames frames, not steps.
  if (in.rx_steer.seq != rx_frame_seq_) {
    rx_frame_seq_ = in.rx_steer.seq;
    bool moved = stick_moved(in.rx_steer) || stick_moved(in.rx_throttle);
    moved_frames_ = moved ? (moved_frames_ < 255 ? moved_frames_ + 1 : 255) : 0;
  }

  // ---- Transitions ---------------------------------------------------------
  if (!sw) {
    if (mode_ != Mode::Manual) enter(Mode::Manual, now_ms);
  } else if (switched_on && mode_ == Mode::Manual) {
    enter(Mode::Arming, now_ms);
  }

  switch (mode_) {
    case Mode::Arming: {
      bool ready = orin_ok(in, now_ms) && stick_centred(in.rx_steer) &&
                   stick_centred(in.rx_throttle);
      if (ready) {
        if (!holding_) {
          holding_ = true;
          holding_since_ms_ = now_ms;
        }
        if (now_ms - holding_since_ms_ >= cfg::kArmingHoldMs) enter(Mode::Auto, now_ms);
      } else {
        holding_ = false;
        if (!heartbeat_.alive(now_ms) && now_ms - arming_since_ms_ >= cfg::kArmingTimeoutMs) {
          enter(Mode::DisengagedOrinLost, now_ms);
        }
      }
      break;
    }
    case Mode::Auto:
      // A lost radio is Orin lost, not a takeover: nobody can take over.
      if (!orin_ok(in, now_ms) || !radio_ok(in)) {
        enter(Mode::DisengagedOrinLost, now_ms);
      } else if (moved_frames_ >= cfg::kTakeoverFrames) {
        enter(Mode::DisengagedTakeover, now_ms);
      }
      break;
    default:
      break;
  }

  // ---- Outputs -------------------------------------------------------------
  Outputs out;
  out.mode = mode_;
  out.auto_line = mode_ == Mode::Auto;

  switch (mode_) {
    case Mode::Auto:
      last_servo_us_ = in.orin_steer.width_us;
      out.vesc = from_throttle(throttle(in.orin_throttle, cfg::kOrinThrottle),
                               cfg::kOrinMaxCurrentA);
      break;
    case Mode::DisengagedOrinLost:
      // Steering passes through at once; throttle stays braked until the
      // driver moves it, so the vehicle never rolls off on its own.
      if (radio_ok(in)) {
        last_servo_us_ = in.rx_steer.width_us;
        if (throttle(in.rx_throttle, cfg::kRxThrottle) != 0.0f) throttle_released_ = true;
      }
      if (radio_ok(in) && throttle_released_) {
        joystick_drive(in, out);
      } else {
        out.vesc = hold_brake();
      }
      break;
    default:  // Manual, Arming, DisengagedTakeover
      joystick_drive(in, out);
      break;
  }
  out.servo_us = last_servo_us_;
  return out;
}

}  // namespace arbiter
