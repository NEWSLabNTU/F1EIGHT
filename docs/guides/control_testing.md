# Control System Testing Guide

This guide provides instructions for testing the AutoSDV motor control system, including manual and automatic control modes.

## Table of Contents
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [System Architecture](#system-architecture)
- [Quick Start](#quick-start)
- [Testing Procedures](#testing-procedures)
- [Debug Topics](#debug-topics)
- [Known Issues](#known-issues)
- [Troubleshooting](#troubleshooting)

## Overview

The AutoSDV control system consists of:
- **Actuator Node** (`/autosdv/actuator_node`): Implements cascaded PID control for motor and steering
- **Velocity Report Node**: Measures wheel speed via GPIO sensors
- **Manual Control Interface**: Keyboard-based control for manual testing
- **Autoware Control Stack**: Generates autonomous control commands

## Prerequisites

1. **Built System**: Ensure AutoSDV is built with `just build`
2. **Hardware**:
   - PWM driver (PCA9685) properly connected
   - Wheel speed sensor connected to GPIO pin
   - Motor and steering servo connected
3. **Permissions**: User in `dialout` and `gpio` groups
4. **tmux** (for `make test-control`): Install with `sudo apt install tmux`

## System Architecture

### Control Flow
```
Autoware Planning          Manual Controller
       ↓                           ↓
       └─────→ Control Command ←───┘
                     ↓
              Actuator Node
              (PID Controller)
                ↓         ↓
              Motor    Steering
                ↓
           Velocity Sensor
                ↓
          Velocity Report
                ↓
           (feedback loop)
```

### Key Topics
- **Input**: `/autosdv/actuator_node/input/control_cmd` (Control)
- **Input**: `/autosdv/actuator_node/input/velocity_status` (VelocityReport)
- **Debug**: `/autosdv/actuator_node/debug/control_values` (Float32MultiArrayStamped)
- **Debug**: `/autosdv/actuator_node/debug/pwm_values` (Float32MultiArrayStamped)
- **Debug**: `/autosdv/actuator_node/debug/pid_values` (Float32MultiArrayStamped)

## Quick Start

### Method 1: Test Control (Recommended)
Launch everything in a tmux session with three windows:

```bash
make test-control
```

This creates a tmux session with:
- **Window 0** (`launch`): AutoSDV system
- **Window 1** (`controller`): Keyboard controller
- **Window 2** (`monitor`): Control command monitor

**tmux Navigation**:
- `Ctrl+b` then `0/1/2`: Switch between windows
- `Ctrl+b` then `d`: Detach from session
- `tmux attach -t autosdv-control-test`: Re-attach to session
- `Ctrl+b` then `:kill-session`: Close entire session

### Method 2: Manual Start
Start system and controller separately:

```bash
# Terminal 1: Launch AutoSDV system
just launch

# Terminal 2: Launch keyboard controller
make controller
```

## Testing Procedures

### 1. Manual Control Testing

#### Step 1: Start the System
```bash
make test-control
```

#### Step 2: Switch to Controller Window
Press `Ctrl+b` then `1` to switch to the controller window.

#### Step 3: Set Local Mode in RViz
1. Open RViz (browser or RViz2 client)
2. Find the control mode panel
3. Set mode to **"Local"** (not "Remote")
4. This enables manual control commands to reach the actuator

#### Step 4: Drive the Vehicle
Use the keyboard controller interface:
- **W/S**: Forward/Backward throttle
- **A/D**: Steering left/right
- **Space**: Emergency stop
- **Q**: Quit controller

#### Step 5: Monitor Control Commands
Switch to monitor window (`Ctrl+b` then `2`) to see control commands:
```bash
# Shows real-time control_cmd messages
longitudinal:
  velocity: 1.5
  acceleration: 0.0
lateral:
  steering_tire_angle: 0.2
```

### 2. Automatic Control Testing

#### Step 1: Launch AutoSDV
```bash
just launch
```

#### Step 2: Set Remote Mode in RViz
1. Open RViz
2. Set control mode to **"Remote"**
3. This enables Autoware planning to control the vehicle

#### Step 3: Set Initial Pose
1. Use RViz's "2D Pose Estimate" tool
2. Click and drag on the map to set vehicle position and orientation
3. Wait for localization to converge

#### Step 4: Set Goal Pose
1. Use RViz's "2D Goal Pose" tool
2. Click and drag to set target destination
3. Autoware planning will generate trajectory

#### Step 5: Engage Autonomous Mode
1. In RViz control panel, click "Engage"
2. Vehicle should start following the planned trajectory
3. Monitor debug topics for control feedback

### 3. Debug Topic Monitoring

Monitor real-time control data in separate terminals:

#### Control Values (throttle, brake, reverse)
```bash
ros2 topic echo /autosdv/actuator_node/debug/control_values
```
Output format: `[throttle, brake, in_reverse]`
- `throttle`: 0.0 - 1.0 (forward throttle command)
- `brake`: 0.0 - 1.0 (brake command)
- `in_reverse`: 0.0 (forward) or 1.0 (reverse)

#### PWM Values (motor, steering)
```bash
ros2 topic echo /autosdv/actuator_node/debug/pwm_values
```
Output format: `[motor_pwm, steer_pwm]`
- `motor_pwm`: 340-400 (typical range, 370 = stop)
- `steer_pwm`: 350-450 (400 = center)

#### PID Controller Values
```bash
ros2 topic echo /autosdv/actuator_node/debug/pid_values
```
Output format (11 values):
```
[target_speed, current_speed, target_tire_angle, current_tire_angle,
 speed_p, speed_i, speed_d, accel_p, accel_i, accel_d, accel_target]
```

#### Velocity Status
```bash
ros2 topic echo /vehicle/status/velocity_status
```

#### Combined Monitoring with rqt_multiplot
For real-time plotting:
```bash
ros2 run rqt_multiplot rqt_multiplot
```

## Debug Topics

### `/autosdv/actuator_node/debug/control_values`
**Type**: `autoware_internal_debug_msgs/msg/Float32MultiArrayStamped`
**Rate**: ~20 Hz (matches controller rate)
**Data**: `[throttle, brake, in_reverse]`

### `/autosdv/actuator_node/debug/pwm_values`
**Type**: `autoware_internal_debug_msgs/msg/Float32MultiArrayStamped`
**Rate**: ~20 Hz
**Data**: `[motor_pwm, steer_pwm]`

Expected PWM ranges:
- **Motor**: 340 (min/brake) - 370 (stop) - 400 (max forward)
- **Steering**: 350 (left) - 400 (center) - 450 (right)

### `/autosdv/actuator_node/debug/pid_values`
**Type**: `autoware_internal_debug_msgs/msg/Float32MultiArrayStamped`
**Rate**: ~20 Hz
**Data**: 11 values including setpoints, measurements, and PID terms

This topic is crucial for tuning PID parameters.

## Known Issues

### 1. Steering Direction Reversed
**Symptom**: Steering input is inverted (left command turns right, right command turns left)

**Status**: Known issue, under investigation

**Workaround**: Temporarily invert steering commands in controller, or adjust `tire_angle_to_steer_ratio` parameter sign

**Location**: `src/vehicle/autosdv_vehicle_launch/config/actuator.param.yaml`

### 2. PID Control Not Working
**Symptom**: Setting positive speed in `control_cmd` doesn't actuate motor

**Status**: Under investigation - possible causes:
- PID gains too low (no output)
- Velocity feedback not arriving (check `/vehicle/status/velocity_status`)
- Control mode not set correctly (must be "Local" for manual, "Remote" for autonomous)
- Dead zone in throttle mapping

**Debug Steps**:
1. Check velocity feedback:
   ```bash
   ros2 topic hz /vehicle/status/velocity_status
   ```
   Should show ~20 Hz updates

2. Check control commands arriving:
   ```bash
   ros2 topic echo /autosdv/actuator_node/input/control_cmd
   ```

3. Monitor PID values:
   ```bash
   ros2 topic echo /autosdv/actuator_node/debug/pid_values
   ```
   - If all zeros → velocity feedback missing or PID gains too low
   - If non-zero P/I/D terms → check throttle/brake conversion

4. Check PWM output:
   ```bash
   ros2 topic echo /autosdv/actuator_node/debug/pwm_values
   ```
   - Should show non-370 values when throttle commanded
   - If stuck at 370 → throttle-to-PWM conversion issue

**Temporary Manual Control**:
Use the interactive PWM control script for direct hardware testing:
```bash
python3 src/vehicle/autosdv_vehicle_launch/scripts/motor_pwm_interactive.py --car-profile car_b   # car_a|car_b|car_c
```

## Troubleshooting

### Actuator Node Not Starting
**Check**: Node is running
```bash
ros2 node list | grep actuator
```

**Check**: I2C permissions
```bash
ls -l /dev/i2c-*
```

**Fix**: Add user to i2c group
```bash
sudo usermod -a -G i2c $USER
# Log out and back in
```

### No Velocity Feedback
**Check**: Velocity report node is running
```bash
ros2 node list | grep velocity
```

**Check**: GPIO pin configuration
```bash
cat src/vehicle/autosdv_vehicle_launch/config/velocity_report.param.yaml
```

**Test**: Run GPIO monitor script
```bash
./scripts/control/gpio_read.py
```
Should show events when wheel rotates

### Controller Not Responding
**Check**: Control mode in RViz
- Must be set to "Local" for manual control
- Set to "Remote" for autonomous control

**Check**: Controller is publishing
```bash
ros2 topic hz /control/command/control_cmd
```

**Check**: Actuator is subscribed
```bash
ros2 topic info /autosdv/actuator_node/input/control_cmd
```

### Steering Not Centering
**Check**: Init steering PWM value
```bash
ros2 param get /autosdv/actuator_node init_steer
```
Should be 400 (center position)

**Manual Reset**:
```bash
./scripts/control/test_steering_pwm.py
```

### Emergency Stop
**Immediate Stop**:
```bash
python3 src/vehicle/autosdv_vehicle_launch/scripts/stop_motor.py --car-profile car_b   # car_a|car_b|car_c
```
Sets motor PWM to the car's neutral (`init_pwm` in `params/cars/<car>.param.yaml`: 370 on car A, 395 on car B) and centres the steering.

**System Stop**:
```bash
make stop
```

## Parameter Tuning

### PID Gains
Located in: `src/vehicle/autosdv_vehicle_launch/config/actuator.param.yaml`

**Speed Controller** (outer loop):
```yaml
kp_speed: 1.0    # Proportional gain
ki_speed: 0.1    # Integral gain
kd_speed: 0.0    # Derivative gain
```

**Acceleration Controller** (inner loop):
```yaml
kp_accel: 0.5    # Proportional gain
ki_accel: 0.1    # Integral gain
kd_accel: 0.0    # Derivative gain
```

**Tuning Guidelines**:
1. Start with low gains and gradually increase
2. Tune outer loop (speed) first, then inner loop (accel)
3. Monitor oscillations in debug topics
4. Use Ziegler-Nichols method for initial values
5. Rebuild after changes: `just build`

### Steering Calibration
```yaml
init_steer: 400              # Center position
min_steer: 350               # Full left
max_steer: 450               # Full right
tire_angle_to_steer_ratio: 50.0  # PWM per radian
```

### Motor Calibration
```yaml
init_pwm: 370    # Stop position
min_pwm: 340     # Full reverse / brake
max_pwm: 400     # Full forward
```

## Additional Testing Tools

### GPIO Speed Measurement (Non-ROS)
```bash
./scripts/control/gpio_speed.py
```
Displays speed in m/s and km/h without ROS dependencies.

### GPIO Pin Monitor
```bash
./scripts/control/gpio_read.py
```
Shows GPIO state changes and event counts for debugging wheel sensor.

### Interactive PWM Control
```bash
python3 src/vehicle/autosdv_vehicle_launch/scripts/motor_pwm_interactive.py --car-profile car_b   # car_a|car_b|car_c
```
Direct PWM control for hardware testing and calibration.

## References

- **Actuator Node**: `src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/autosdv_vehicle_interface/actuator.py`
- **Configuration**: `src/vehicle/autosdv_vehicle_launch/config/actuator.param.yaml`
- **Velocity Report**: `src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/autosdv_vehicle_interface/velocity_report.py`
- **PWM Calibration**: `CLAUDE.md` - Vehicle Interface Calibration section
