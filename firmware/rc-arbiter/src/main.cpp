// RC override arbiter on a Seeed XIAO RP2350.
//
// The hardware layer only: capture the inputs, run the arbiter, write the
// outputs. Every decision is in lib/arbiter, which the host tests cover.
// Design: docs/design/rc-override-arbiter.md.

// The NeoPixel header trips -Wextra on its own PIO table; keep it out of ours.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include <Adafruit_NeoPixel.h>
#pragma GCC diagnostic pop
#include <Arduino.h>
#include <Servo.h>

#include "arbiter.h"
#include "capture.h"
#include "led_pattern.h"
#include "pins.h"
#include "vesc_packet.h"

namespace {

constexpr uint32_t kLoopPeriodUs = 1000;  // arbiter at 1 kHz
constexpr uint32_t kVescPeriodMs = 10;    // 100 Hz, well inside the 50 ms VESC timeout
constexpr uint32_t kLedPeriodMs = 20;
constexpr uint32_t kWatchdogMs = 100;
constexpr uint32_t kVescBaud = 115200;

arbiter::Arbiter arb;
Servo servo;
Adafruit_NeoPixel led(1, pins::kLedData, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ext_led(1, pins::kExtLedData, NEO_GRB + NEO_KHZ800);

void send_vesc(const arbiter::VescCommand& c) {
  uint8_t buf[vesc::kMaxPacket];
  size_t n = c.kind == arbiter::VescCommand::Kind::Current ? vesc::set_current(c.amps, buf)
                                                           : vesc::set_brake_current(c.amps, buf);
  Serial1.write(buf, n);
}

void show_led(arbiter::Mode m, uint32_t now_ms) {
  arbiter::Rgb c = arbiter::led_pattern(m, now_ms);
  uint32_t colour = Adafruit_NeoPixel::Color(c.r, c.g, c.b);
  led.setPixelColor(0, colour);
  led.show();
  ext_led.setPixelColor(0, colour);
  ext_led.show();
}

}  // namespace

void setup() {
  Serial.begin(115200);

  pinMode(pins::kSwitchSense, INPUT_PULLUP);
  pinMode(pins::kAutoLine, OUTPUT);
  digitalWrite(pins::kAutoLine, LOW);

  pinMode(pins::kLedPower, OUTPUT);
  digitalWrite(pins::kLedPower, HIGH);
  led.begin();
  led.setBrightness(64);
  ext_led.begin();

  Serial1.setTX(pins::kVescTx);
  Serial1.setRX(pins::kVescRx);
  Serial1.begin(kVescBaud);

  capture::begin();

  // A hung loop stops the VESC commands, and the VESC brakes on its own
  // timeout; the watchdog then restarts the board in MANUAL.
  rp2040.wdt_begin(kWatchdogMs);
}

void loop() {
  static uint32_t next_us = micros();
  static uint32_t last_vesc_ms = 0;
  static uint32_t last_led_ms = 0;
  static arbiter::Mode last_mode = arbiter::Mode::Manual;

  while (static_cast<int32_t>(micros() - next_us) < 0) {
  }
  next_us += kLoopPeriodUs;

  uint32_t now = millis();

  arbiter::Inputs in;
  in.switch_on = digitalRead(pins::kSwitchSense) == LOW;
  in.rx_steer = capture::read(capture::RxSteer, now);
  in.rx_throttle = capture::read(capture::RxThrottle, now);
  in.orin_steer = capture::read(capture::OrinSteer, now);
  in.orin_throttle = capture::read(capture::OrinThrottle, now);
  in.heartbeat = capture::read(capture::Heartbeat, now);

  arbiter::Outputs out = arb.step(in, now);

  if (out.servo_us != 0) {
    if (!servo.attached()) servo.attach(pins::kServo, arbiter::cfg::kPulseMinUs,
                                        arbiter::cfg::kPulseMaxUs);
    servo.writeMicroseconds(out.servo_us);
  }
  digitalWrite(pins::kAutoLine, out.auto_line ? HIGH : LOW);

  if (now - last_vesc_ms >= kVescPeriodMs) {
    last_vesc_ms = now;
    send_vesc(out.vesc);
  }
  if (now - last_led_ms >= kLedPeriodMs) {
    last_led_ms = now;
    show_led(out.mode, now);
  }
  if (out.mode != last_mode) {
    if (Serial) {
      Serial.printf("%lu ms: %s -> %s\n", static_cast<unsigned long>(now),
                    arbiter::mode_name(last_mode), arbiter::mode_name(out.mode));
    }
    last_mode = out.mode;
  }

  rp2040.wdt_reset();
}
