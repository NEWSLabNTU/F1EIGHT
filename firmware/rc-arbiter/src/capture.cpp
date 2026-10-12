#include "capture.h"

#include <Arduino.h>

#include "pins.h"

namespace capture {

namespace {

struct State {
  uint8_t pin;
  volatile bool high;
  volatile uint32_t rise_us;
  volatile uint16_t width_us;
  volatile uint32_t fall_ms;
  volatile uint32_t seq;
};

State channels[kCount] = {
    {pins::kRxSteer, false, 0, 0, 0, 0},
    {pins::kRxThrottle, false, 0, 0, 0, 0},
    {pins::kOrinThrottle, false, 0, 0, 0, 0},
    {pins::kOrinSteer, false, 0, 0, 0, 0},
    {pins::kHeartbeat, false, 0, 0, 0, 0},
};

void on_edge(void* param) {
  State* s = static_cast<State*>(param);
  uint32_t t = micros();
  if (digitalRead(s->pin)) {
    s->high = true;
    s->rise_us = t;
  } else if (s->high) {
    // Only a falling edge after a seen rising edge is a pulse; the first fall
    // after boot has no start.
    s->high = false;
    uint32_t w = t - s->rise_us;
    s->width_us = w > 0xffff ? 0xffff : static_cast<uint16_t>(w);
    s->fall_ms = millis();
    s->seq = s->seq + 1;
  }
}

}  // namespace

void begin() {
  for (auto& s : channels) {
    pinMode(s.pin, INPUT);
    attachInterruptParam(digitalPinToInterrupt(s.pin), on_edge, CHANGE, &s);
  }
}

arbiter::Pulse read(Channel ch, uint32_t now_ms) {
  const State& s = channels[ch];
  arbiter::Pulse p;
  noInterrupts();
  p.width_us = s.width_us;
  uint32_t fall_ms = s.fall_ms;
  p.seq = s.seq;
  interrupts();
  p.age_ms = p.seq == 0 ? UINT32_MAX : now_ms - fall_ms;
  return p;
}

}  // namespace capture
