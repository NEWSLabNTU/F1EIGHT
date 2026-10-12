# RC override arbiter firmware

Firmware for the board that makes the remote joystick a kill switch: a Seeed
XIAO RP2350 between the receiver, the Orin's PCA9685, the steering servo and
the VESC. What it does and why is in
[`docs/design/rc-override-arbiter.md`](../../docs/design/rc-override-arbiter.md);
this file is about building it.

Status: builds and passes its host tests. It has not run on a board yet.

## Layout

```
platformio.ini        two environments: xiao_rp2350 (the board), native (host tests)
lib/arbiter/          the state machine, pure C++17 with no Arduino.h
  config.h            every timing, threshold and current, in one place
  arbiter.h/.cpp      step(Inputs, now_ms) -> Outputs
  pulse.h/.cpp        pulse validity, stick centred or moved, throttle maps
  heartbeat.h/.cpp    the Orin heartbeat on PCA9685 ch2
  debounce.h          the auto-mode switch
  led_pattern.h       mode to LED colour and blink
lib/vesc/             VESC UART packets as bytes, no Arduino.h
src/                  the hardware layer: capture, servo, UART, LED, watchdog
  pins.h              the pin map, matching the design note's wiring table
test/test_arbiter/    the mode table played through a simulated vehicle
test/test_vesc/       packets byte for byte, CRC against its check value
```

Everything that decides anything lives in `lib/`, because PlatformIO's host
test environment builds `lib/` and `test/` but never `src/`. `src/` only reads
pins, calls `Arbiter::step()` once a millisecond and writes the result.

## Prerequisites

PlatformIO Core, from the setup program or by hand:

```bash
./setup.sh --run --only platformio      # uv, pipx or pip, whichever is present
uv tool install platformio              # or: pipx install platformio
```

The first board build downloads the toolchain, the arduino-pico core and
picotool into `~/.platformio`, about 500 MB. The host tests need only the
system g++.

To flash, the machine with the board plugged in also needs the udev rules and
the `dialout` group (log out and back in afterwards):

```bash
./setup.sh --run --only platformio-udev
```

## Developer steps

```bash
just arbiter test       # host tests, no board
just arbiter build      # cross-compile; prints flash and RAM use
just arbiter flash      # upload over USB
just arbiter monitor    # one log line per mode change
```

The usual loop is to change `lib/arbiter`, add or adjust a test in
`test/test_arbiter`, run `just arbiter test`, then `just arbiter build` to check
the board build still compiles.

**First flash:** hold BOOT while plugging the board in; it mounts as a drive
named `RP2350`, and `just arbiter flash` copies the UF2 onto it. Later uploads
reset the board over USB serial and need no button.

The toolchain is pinned to a commit of maxgerhardt's `platform-raspberrypi`
fork in `platformio.ini`; the official `raspberrypi` platform ships the Mbed
core, which has no RP2350. Moving the pin is a deliberate edit.

## What the tests cannot cover

The host tests check the decisions; the bench has to check the hardware.

- Pulse capture accuracy and jitter. Capture uses GPIO interrupts and
  `micros()`; the PIO capture in the design note is the upgrade if the jitter
  turns out to matter.
- The real widths of the Orin's pulses. The PCA9685 oscillator is only
  accurate to a few percent, so `kOrinThrottle` in `config.h` has to be
  measured with a logic analyser.
- Every value marked `BENCH` in `config.h`: the receiver centre and travel and
  the four VESC currents.
- How the VESC answers: the 50 ms UART timeout, the timeout brake current and
  the braking itself.
- The LED, the 5 V supply from the receiver rail, and the takeover latency.

## Not built yet

- **The speed cap.** The design stores the Orin-mode `max-speed` in the VESC
  and raises it while the joystick drives. Changing it over UART needs the
  VESC's motor configuration commands; until then the cap stays whatever VESC
  Tool stored, in every mode.
- **Reverse.** A throttle below centre brakes; there is no reverse yet.
- **VESC telemetry for odometry.** The UART RX line is wired but unread.
