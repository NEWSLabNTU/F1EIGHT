// The arbiter against the mode table in docs/design/rc-override-arbiter.md.
//
// Sim plays the vehicle at 1 ms resolution: the receiver frames at 50 Hz, the
// PCA9685 at about 60 Hz, the heartbeat flips its width every 50 ms, and the
// Orin's throttle and steer pulses reach the board only while the switch is
// ON, as the 3PDT poles cut them in hardware.
#include <unity.h>

#include <initializer_list>

#include "arbiter.h"
#include "led_pattern.h"

using namespace arbiter;

namespace {

constexpr uint16_t kOrinSteerCentre = cfg::pca_us(489);  // actuator.yaml init_steer
constexpr uint16_t kOrinNeutral = cfg::pca_us(370);

struct Channel {
  Pulse p;
  uint32_t last_ms = 0;
  void emit(uint16_t w, uint32_t now) {
    p.width_us = w;
    p.seq++;
    last_ms = now;
  }
  void age(uint32_t now) { p.age_ms = p.seq == 0 ? UINT32_MAX : now - last_ms; }
};

struct Sim {
  Arbiter arb;
  uint32_t now = 0;
  Outputs out;

  bool switch_on = false;
  bool radio = true;        // receiver outputs pulses
  bool orin_running = true; // PCA9685 outputs pulses
  bool heartbeat = true;    // actuator node toggles ch2

  uint16_t rx_steer = cfg::kRxCentreUs;
  uint16_t rx_throttle = cfg::kRxCentreUs;
  uint16_t orin_steer = kOrinSteerCentre;
  uint16_t orin_throttle = kOrinNeutral;

  Channel ch_rx_steer, ch_rx_throttle, ch_orin_steer, ch_orin_throttle, ch_hb;
  uint16_t hb_width = 1200;

  void tick() {
    ++now;
    if (radio && now % 20 == 0) {
      ch_rx_steer.emit(rx_steer, now);
      ch_rx_throttle.emit(rx_throttle, now);
    }
    if (heartbeat && now % 50 == 0) hb_width = hb_width == 1200 ? 1800 : 1200;
    if (orin_running && now % 17 == 0) {
      if (switch_on) {
        ch_orin_steer.emit(orin_steer, now);
        ch_orin_throttle.emit(orin_throttle, now);
      }
      ch_hb.emit(hb_width, now);
    }
    Inputs in;
    in.switch_on = switch_on;
    for (Channel* c : {&ch_rx_steer, &ch_rx_throttle, &ch_orin_steer, &ch_orin_throttle, &ch_hb}) {
      c->age(now);
    }
    in.rx_steer = ch_rx_steer.p;
    in.rx_throttle = ch_rx_throttle.p;
    in.orin_steer = ch_orin_steer.p;
    in.orin_throttle = ch_orin_throttle.p;
    in.heartbeat = ch_hb.p;
    out = arb.step(in, now);
  }

  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) tick();
  }

  // Run until the mode changes or the time runs out; returns the elapsed ms.
  uint32_t run_until(Mode m, uint32_t max_ms) {
    for (uint32_t i = 1; i <= max_ms; ++i) {
      tick();
      if (out.mode == m) return i;
    }
    return UINT32_MAX;
  }

  // Power up with the switch OFF, let the inputs settle, turn it ON and arm.
  void to_auto() {
    run(200);
    switch_on = true;
    TEST_ASSERT_NOT_EQUAL(UINT32_MAX, run_until(Mode::Auto, 2000));
  }
};

#define ASSERT_MODE(expected, sim) \
  TEST_ASSERT_EQUAL_STRING(mode_name(expected), mode_name((sim).out.mode))

}  // namespace

void setUp() {}
void tearDown() {}

// ---- MANUAL -----------------------------------------------------------------

static void test_starts_manual_and_joystick_drives() {
  Sim s;
  s.rx_steer = 1600;
  s.rx_throttle = 1750;  // half forward
  s.run(100);
  ASSERT_MODE(Mode::Manual, s);
  TEST_ASSERT_EQUAL_UINT16(1600, s.out.servo_us);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Current);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.5f * cfg::kRxMaxCurrentA, s.out.vesc.amps);
  TEST_ASSERT_FALSE(s.out.auto_line);
}

static void test_joystick_brake_below_centre() {
  Sim s;
  s.rx_throttle = 1000;
  s.run(100);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Brake);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, cfg::kMaxBrakeCurrentA, s.out.vesc.amps);
}

static void test_radio_lost_in_manual_brakes() {
  Sim s;
  s.rx_throttle = 1750;
  s.run(100);
  s.radio = false;
  s.run(cfg::kPulseTimeoutMs + 30);
  ASSERT_MODE(Mode::Manual, s);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Brake);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, cfg::kHoldBrakeCurrentA, s.out.vesc.amps);
}

static void test_switch_on_at_power_up_stays_manual() {
  Sim s;
  s.switch_on = true;
  s.run(3000);
  ASSERT_MODE(Mode::Manual, s);
  // A real OFF to ON arms.
  s.switch_on = false;
  s.run(100);
  s.switch_on = true;
  TEST_ASSERT_NOT_EQUAL(UINT32_MAX, s.run_until(Mode::Arming, 100));
}

// ---- ARMING -----------------------------------------------------------------

static void test_arming_waits_one_second_then_auto() {
  Sim s;
  s.run(200);
  s.switch_on = true;
  uint32_t to_arming = s.run_until(Mode::Arming, 100);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(cfg::kSwitchDebounceMs + 1, to_arming);
  uint32_t to_auto = s.run_until(Mode::Auto, 3000);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(cfg::kArmingHoldMs, to_auto);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(cfg::kArmingHoldMs + 200, to_auto);
  TEST_ASSERT_TRUE(s.out.auto_line);
}

static void test_arming_needs_sticks_centred() {
  Sim s;
  s.run(200);
  s.rx_steer = 1600;  // off centre, inside the takeover band or not, it blocks
  s.switch_on = true;
  s.run(4000);
  ASSERT_MODE(Mode::Arming, s);
  s.rx_steer = cfg::kRxCentreUs;
  uint32_t t = s.run_until(Mode::Auto, 2000);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(cfg::kArmingHoldMs, t);
}

static void test_arming_releases_hold_when_stick_moves() {
  Sim s;
  s.run(200);
  s.switch_on = true;
  s.run(800);  // most of the hold
  ASSERT_MODE(Mode::Arming, s);
  s.rx_throttle = 1600;
  s.run(100);
  s.rx_throttle = cfg::kRxCentreUs;
  // The hold starts over, so AUTO is a full second away again.
  uint32_t t = s.run_until(Mode::Auto, 2000);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(cfg::kArmingHoldMs, t);
}

static void test_arming_without_heartbeat_times_out_to_orin_lost() {
  Sim s;
  s.heartbeat = false;
  s.run(200);
  s.switch_on = true;
  s.run_until(Mode::Arming, 100);
  uint32_t t = s.run_until(Mode::DisengagedOrinLost, 6000);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(cfg::kArmingTimeoutMs - 10, t);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(cfg::kArmingTimeoutMs + 10, t);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Brake);
}

static void test_joystick_drives_while_arming() {
  Sim s;
  s.run(200);
  s.switch_on = true;
  s.heartbeat = false;
  s.rx_steer = 1700;
  s.rx_throttle = 1800;
  s.run(500);
  ASSERT_MODE(Mode::Arming, s);
  TEST_ASSERT_EQUAL_UINT16(1700, s.out.servo_us);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Current);
}

// ---- AUTO -------------------------------------------------------------------

static void test_auto_follows_the_orin() {
  Sim s;
  s.to_auto();
  s.orin_steer = cfg::pca_us(520);
  s.orin_throttle = cfg::pca_us(470);  // max_pwm: full Orin current
  s.run(50);
  ASSERT_MODE(Mode::Auto, s);
  TEST_ASSERT_EQUAL_UINT16(cfg::pca_us(520), s.out.servo_us);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Current);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, cfg::kOrinMaxCurrentA, s.out.vesc.amps);

  s.orin_throttle = kOrinNeutral;
  s.run(50);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, s.out.vesc.amps);

  s.orin_throttle = cfg::pca_us(340);  // brake_pwm: full brake
  s.run(50);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Brake);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, cfg::kMaxBrakeCurrentA, s.out.vesc.amps);
}

static void test_takeover_latches() {
  Sim s;
  s.to_auto();
  s.rx_steer = 1700;
  uint32_t t = s.run_until(Mode::DisengagedTakeover, 200);
  // Two receiver frames, 20 ms apart, plus up to one frame of phase.
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(60, t);
  TEST_ASSERT_FALSE(s.out.auto_line);
  TEST_ASSERT_EQUAL_UINT16(1700, s.out.servo_us);

  // Letting go does not hand control back.
  s.rx_steer = cfg::kRxCentreUs;
  s.run(3000);
  ASSERT_MODE(Mode::DisengagedTakeover, s);
  TEST_ASSERT_EQUAL_UINT16(cfg::kRxCentreUs, s.out.servo_us);
}

static void test_throttle_takeover() {
  Sim s;
  s.to_auto();
  s.rx_throttle = 1300;  // driver brakes
  TEST_ASSERT_NOT_EQUAL(UINT32_MAX, s.run_until(Mode::DisengagedTakeover, 100));
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Brake);
}

static void test_one_frame_glitch_is_not_a_takeover() {
  Sim s;
  s.to_auto();
  // Line up with a frame boundary so exactly one frame carries the glitch.
  while (s.now % 20 != 19) s.tick();
  s.rx_steer = 1700;
  s.run(20);
  s.rx_steer = cfg::kRxCentreUs;
  s.run(200);
  ASSERT_MODE(Mode::Auto, s);
}

static void test_small_stick_drift_is_not_a_takeover() {
  Sim s;
  s.to_auto();
  s.rx_steer = cfg::kRxCentreUs + cfg::kTakeoverUs;  // at the edge, not past it
  s.run(1000);
  ASSERT_MODE(Mode::Auto, s);
}

static void test_heartbeat_lost_brakes() {
  Sim s;
  s.to_auto();
  s.orin_throttle = cfg::pca_us(470);
  s.run(100);
  s.heartbeat = false;
  uint32_t t = s.run_until(Mode::DisengagedOrinLost, 500);
  // 150 ms after the last toggle, which was up to 50 ms before it stopped,
  // seen at the next 17 ms pulse.
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(cfg::kHeartbeatTimeoutMs + 20, t);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Brake);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, cfg::kHoldBrakeCurrentA, s.out.vesc.amps);
}

static void test_orin_dead_brakes() {
  // PCA9685 or I2C gone: no pulses at all, heartbeat included.
  Sim s;
  s.to_auto();
  s.orin_running = false;
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(cfg::kHeartbeatTimeoutMs + 20,
                                   s.run_until(Mode::DisengagedOrinLost, 500));
}

static void test_radio_lost_in_auto_is_orin_lost() {
  Sim s;
  s.to_auto();
  s.radio = false;
  uint32_t t = s.run_until(Mode::DisengagedOrinLost, 500);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(cfg::kPulseTimeoutMs + 21, t);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Brake);
}

// ---- DISENGAGED: Orin lost --------------------------------------------------

static void test_orin_lost_holds_brake_until_throttle_moves() {
  Sim s;
  s.to_auto();
  s.heartbeat = false;
  s.run_until(Mode::DisengagedOrinLost, 500);

  // Steering passes through at once, throttle stays braked at neutral.
  s.rx_steer = 1650;
  s.run(100);
  TEST_ASSERT_EQUAL_UINT16(1650, s.out.servo_us);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Brake);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, cfg::kHoldBrakeCurrentA, s.out.vesc.amps);

  // The driver pushes the throttle: it passes through from then on.
  s.rx_throttle = 1750;
  s.run(50);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Current);
  s.rx_throttle = cfg::kRxCentreUs;
  s.run(50);
  TEST_ASSERT_TRUE(s.out.vesc.kind == VescCommand::Kind::Current);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, s.out.vesc.amps);

  // The heartbeat coming back does not re-enter AUTO.
  s.heartbeat = true;
  s.run(3000);
  ASSERT_MODE(Mode::DisengagedOrinLost, s);
}

// ---- The switch -------------------------------------------------------------

static void test_switch_off_from_auto_is_manual_not_orin_lost() {
  // Turning the switch OFF cuts the Orin pulses at once; the debounce must
  // finish before those pulses count as missing.
  Sim s;
  s.to_auto();
  s.switch_on = false;
  uint32_t t = s.run_until(Mode::Manual, 200);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(cfg::kSwitchDebounceMs + 1, t);
  s.run(500);
  ASSERT_MODE(Mode::Manual, s);
}

static void test_switch_bounce_does_not_leave_auto() {
  Sim s;
  s.to_auto();
  s.switch_on = false;
  s.run(20);
  s.switch_on = true;
  s.run(500);
  ASSERT_MODE(Mode::Auto, s);
}

static void test_off_on_re_arms_after_disengage() {
  Sim s;
  s.to_auto();
  s.rx_steer = 1700;
  s.run_until(Mode::DisengagedTakeover, 200);
  s.rx_steer = cfg::kRxCentreUs;
  s.switch_on = false;
  s.run(200);
  ASSERT_MODE(Mode::Manual, s);
  s.switch_on = true;
  TEST_ASSERT_NOT_EQUAL(UINT32_MAX, s.run_until(Mode::Auto, 2000));
}

// ---- LED --------------------------------------------------------------------

static void test_led_patterns() {
  TEST_ASSERT_TRUE(led_pattern(Mode::Manual, 0) == kOff);
  TEST_ASSERT_TRUE(led_pattern(Mode::Arming, 100) == kGreen);
  TEST_ASSERT_TRUE(led_pattern(Mode::Arming, 600) == kOff);
  TEST_ASSERT_TRUE(led_pattern(Mode::Auto, 600) == kGreen);
  TEST_ASSERT_TRUE(led_pattern(Mode::DisengagedTakeover, 0) == kAmber);
  TEST_ASSERT_TRUE(led_pattern(Mode::DisengagedTakeover, 130) == kOff);
  TEST_ASSERT_TRUE(led_pattern(Mode::DisengagedOrinLost, 0) == kRed);
  TEST_ASSERT_TRUE(led_pattern(Mode::DisengagedOrinLost, 130) == kDimRed);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_starts_manual_and_joystick_drives);
  RUN_TEST(test_joystick_brake_below_centre);
  RUN_TEST(test_radio_lost_in_manual_brakes);
  RUN_TEST(test_switch_on_at_power_up_stays_manual);
  RUN_TEST(test_arming_waits_one_second_then_auto);
  RUN_TEST(test_arming_needs_sticks_centred);
  RUN_TEST(test_arming_releases_hold_when_stick_moves);
  RUN_TEST(test_arming_without_heartbeat_times_out_to_orin_lost);
  RUN_TEST(test_joystick_drives_while_arming);
  RUN_TEST(test_auto_follows_the_orin);
  RUN_TEST(test_takeover_latches);
  RUN_TEST(test_throttle_takeover);
  RUN_TEST(test_one_frame_glitch_is_not_a_takeover);
  RUN_TEST(test_small_stick_drift_is_not_a_takeover);
  RUN_TEST(test_heartbeat_lost_brakes);
  RUN_TEST(test_orin_dead_brakes);
  RUN_TEST(test_radio_lost_in_auto_is_orin_lost);
  RUN_TEST(test_orin_lost_holds_brake_until_throttle_moves);
  RUN_TEST(test_switch_off_from_auto_is_manual_not_orin_lost);
  RUN_TEST(test_switch_bounce_does_not_leave_auto);
  RUN_TEST(test_off_on_re_arms_after_disengage);
  RUN_TEST(test_led_patterns);
  return UNITY_END();
}
