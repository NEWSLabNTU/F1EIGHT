# RC override arbiter firmware for the Seeed XIAO RP2350.
#
# Sources: firmware/rc-arbiter. Design: docs/design/rc-override-arbiter.md.
# Needs PlatformIO: ./setup.sh --run --only platformio

set working-directory := '..'

fw := 'firmware/rc-arbiter'

# `just <module>` runs the module's FIRST recipe, so that recipe has to be the
# harmless one. Private keeps it out of `just --list --list-submodules`.
[private]
default:
    @just --list arbiter

# Host unit tests of the arbiter and the VESC packets; no board needed
test:
    pio test -d {{fw}} -e native

# Cross-compile for the board; prints flash and RAM use
build:
    pio run -d {{fw}} -e xiao_rp2350

# Flash over USB. First time: hold BOOT while plugging the board in
flash:
    pio run -d {{fw}} -e xiao_rp2350 -t upload

# USB serial log: one line per mode change
monitor:
    pio device monitor -d {{fw}} -e xiao_rp2350

# Remove the build output (.pio/build); the downloaded toolchain stays
clean:
    pio run -d {{fw}} -e xiao_rp2350 -t clean
    rm -rf {{fw}}/.pio/build/native
