// Pulse capture on GPIO interrupts: each edge timestamps with micros(), each
// falling edge completes a pulse. Five channels at 50 to 60 Hz is a light
// interrupt load with a few us of jitter; the design note's PIO capture is the
// upgrade if the bench shows that jitter matters.
#pragma once

#include <stdint.h>

#include "pulse.h"

namespace capture {

enum Channel : uint8_t { RxSteer, RxThrottle, OrinThrottle, OrinSteer, Heartbeat, kCount };

void begin();

// A consistent copy of one channel, with its age relative to now_ms.
arbiter::Pulse read(Channel ch, uint32_t now_ms);

}  // namespace capture
