// Every timing and threshold the arbiter uses, in one place.
//
// The timings come from docs/design/rc-override-arbiter.md. The throttle and
// current values marked BENCH are placeholders until the receiver, the
// transmitter and the VESC motor configuration are known; set them on the
// bench before the first drive.
#pragma once

#include <stdint.h>

namespace arbiter {
namespace cfg {

// ---- Timing (ms) -----------------------------------------------------------

// Auto-mode switch debounce.
constexpr uint32_t kSwitchDebounceMs = 50;
// The heartbeat must toggle at least this often. The Orin toggles at 20 Hz,
// so this is three missed toggles.
constexpr uint32_t kHeartbeatTimeoutMs = 150;
// ARMING needs the heartbeat alive and both sticks centred for this long.
constexpr uint32_t kArmingHoldMs = 1000;
// ARMING without any heartbeat for this long means the Orin was never started.
constexpr uint32_t kArmingTimeoutMs = 5000;
// A pulse channel with no new pulse for this long is lost. The receiver frames
// at 50 Hz and the PCA9685 at 60 Hz, so this is five frames. It must exceed
// kSwitchDebounceMs: turning the switch OFF cuts the Orin pulses at once, and
// the board must see MANUAL before it sees the Orin pulses go missing.
constexpr uint32_t kPulseTimeoutMs = 100;
// Consecutive receiver frames with a stick past kTakeoverUs that count as a
// takeover.
constexpr uint8_t kTakeoverFrames = 2;

// ---- Pulse widths (us) -----------------------------------------------------

// Anything outside this range is noise or a wiring fault.
constexpr uint16_t kPulseMinUs = 700;
constexpr uint16_t kPulseMaxUs = 2500;

// Receiver centre, for both sticks. BENCH: read it off the receiver.
constexpr uint16_t kRxCentreUs = 1500;
// A stick within this of centre counts as centred for ARMING.
constexpr uint16_t kCentredUs = 50;
// A stick past this from centre counts as the driver moving it. Wider than
// kCentredUs so the two do not chatter against each other.
constexpr uint16_t kTakeoverUs = 100;

// Heartbeat: the Orin alternates 1200 us and 1800 us; this splits them.
constexpr uint16_t kHeartbeatSplitUs = 1500;

// ---- Throttle maps ---------------------------------------------------------

// Pulse width to a normalised throttle in [-1, 1]. Positive drives, negative
// brakes; there is no reverse yet.
struct ThrottleMap {
  uint16_t centre_us;
  uint16_t forward_full_us;
  uint16_t brake_full_us;
  uint16_t deadband_us;
};

// PCA9685 counts at 60 Hz to microseconds: counts * 1e6 / (60 * 4096).
constexpr uint16_t pca_us(uint32_t counts) {
  return static_cast<uint16_t>((counts * 1000000UL + 60UL * 4096UL / 2) / (60UL * 4096UL));
}

// From actuator.yaml: init_pwm 370, max_pwm 470, brake_pwm 340. The PCA9685
// oscillator is only accurate to a few percent, so BENCH: measure the real
// widths with the logic analyser and replace these.
constexpr ThrottleMap kOrinThrottle = {pca_us(370), pca_us(470), pca_us(340), 10};

// BENCH: the transmitter's throttle travel.
constexpr ThrottleMap kRxThrottle = {kRxCentreUs, 2000, 1000, 30};

// ---- VESC currents (A) -----------------------------------------------------

// BENCH: all four. The VESC motor configuration limits current as well, so
// these only need to be at or below it.
constexpr float kOrinMaxCurrentA = 15.0f;
constexpr float kRxMaxCurrentA = 30.0f;
constexpr float kMaxBrakeCurrentA = 30.0f;
// Held while the Orin is lost and the joystick throttle sits at neutral.
constexpr float kHoldBrakeCurrentA = 20.0f;

}  // namespace cfg
}  // namespace arbiter
