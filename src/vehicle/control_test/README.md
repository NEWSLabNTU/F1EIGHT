# Control Test Package

This package provides testing utilities for the AutoSDV vehicle control system.

## Overview

The `control_test` package contains nodes and launch files for testing various aspects of vehicle control:

- **Static PWM Test**: Direct hardware PWM control for basic testing
- **PID Speed Control**: PID-based speed controller with keyboard input
- **Control Command Service**: Service-based control command publisher
- **Speedometer Test**: Velocity measurement testing

## Nodes

### 1. `static_pwm_test`
Sets constant PWM values for motor and steering hardware testing.

**Usage:**
```bash
ros2 launch control_test static_pwm_test.launch.xml
ros2 launch control_test static_pwm_test.launch.xml motor_pwm:=380 steering_pwm:=410
```

**Parameters:**
- `motor_pwm`: Motor PWM value (370=stop, >370=forward, <370=reverse)
- `steering_pwm`: Steering PWM value (400=center, 350=left, 450=right)
- `update_rate`: Update rate in Hz (default: 10.0)

### 2. `pid_speed_control`
PID-based speed controller with keyboard input for target speed adjustment.

**⚠️ Requires TTY**: Must be run in a terminal, not through systemd or background process.

**Usage:**
```bash
ros2 launch control_test pid_tuning.launch.xml
ros2 launch control_test pid_tuning.launch.xml kp:=2.0 ki:=0.2 kd:=0.3
```

**Keyboard Controls:**
- `w/s`: Increase/Decrease target speed
- `x`: Emergency stop (target = 0)
- `a/d`: Left/Right steering
- `c`: Center steering
- `q`: Quit
- `h`: Show help

**Parameters:**
- `kp`: Proportional gain
- `ki`: Integral gain
- `kd`: Derivative gain
- `speed_step`: Speed increment per keypress in m/s
- `steering_pwm_step`: Steering PWM increment per keypress

### 3. `control_command_service`
Service-based control command publisher. Accepts target speed/steering via parameters and publishes to `/control/command/control_cmd` when enabled.

**Usage:**
```bash
# Launch with default values (speed=0, steering=0)
ros2 launch control_test control_command_service.launch.xml

# Launch with specific target values
ros2 launch control_test control_command_service.launch.xml target_speed:=1.5 target_steering:=0.1

# Enable publishing
ros2 service call /control_command_service_node/enable example_interfaces/srv/SetBool "{data: true}"

# Disable publishing
ros2 service call /control_command_service_node/enable example_interfaces/srv/SetBool "{data: false}"
```

**Parameters:**
- `target_speed`: Target longitudinal speed in m/s
- `target_steering`: Target steering angle in rad
- `target_acceleration`: Target acceleration in m/s²
- `publish_rate`: Control command publish rate in Hz

**Services:**
- `~/enable` (example_interfaces/srv/SetBool): Enable/disable command publishing

### 4. `keyboard_control` (Autoware Manual Control with GUI)
Keyboard-based manual control for Autoware using arrow keys with a GUI.

This is a Python re-implementation of the `autoware_manual_control` package with:
- Arrow key support for intuitive control
- **Tkinter GUI** for status display and keyboard input
- **Launch-friendly** - works without TTY

**⚠️ Requires**: X11 display (DISPLAY environment variable)

**Usage:**
```bash
# Run standalone
ros2 run control_test keyboard_control

# Launch with GUI (uses config file)
ros2 launch control_test keyboard_control.launch.xml
```

**Configuration:**

Edit `config/keyboard_control.yaml` to customize:
```yaml
speed_step_ms: 0.5       # Speed increment (m/s)
steering_step_deg: 1.0   # Steering increment (degrees)
max_speed_ms: 10.0       # Maximum speed (m/s)
max_steer_deg: 22.5      # Maximum steering angle (degrees)
publish_rate: 30.0       # Command publishing rate (Hz)
```

**Output Topic Selection (GUI Feature):**

The GUI includes a dropdown to select output topic presets or define custom topics:

**Presets:**

1. **External (Standard)** - Default
   - Control: `/external/selected/control_cmd`
   - Gear: `/external/selected/gear_cmd`
   - Standard Autoware external control workflow
   - Requires toggling to EXTERNAL gate mode with 'z' key

2. **Direct (Bypass)**
   - Control: `/control/command/control_cmd`
   - Gear: `/control/command/gear_cmd`
   - Direct vehicle control, bypasses external control selector
   - Useful for testing and debugging

3. **Custom**
   - Allows you to specify custom topic names
   - Text entry fields appear when selected
   - Enter your desired topics and click "Apply Custom Topics"
   - Useful for integration with custom control pipelines

Simply select from the dropdown in the GUI and topics will be applied instantly!

**Controls:**
- **Mode Control:**
  - `z`: Toggle AUTO/EXTERNAL mode
  - `x`: Set gear to DRIVE
  - `c`: Set gear to REVERSE
  - `v`: Set gear to PARK
- **Speed Control:**
  - `↑`: Increase speed (1 m/s steps, configurable)
  - `↓`: Decrease speed (1 m/s steps, configurable)
  - `Space`: Stop (set speed to 0)
  - Range: -10.0 to +10.0 m/s (configurable)
- **Steering Control:**
  - `←`: Turn left (1° steps, configurable)
  - `→`: Turn right (1° steps, configurable)
  - `Enter`: Center steering (angle = 0)
  - Range: ±22.5° (configurable)
- **Other:**
  - `s`: Show current status
  - `h`: Show help
  - `q`: Quit

**Topics Published:**
- `/control/gate_mode_cmd` - Switch between AUTO/EXTERNAL modes
- `/external/selected/control_cmd` - Control commands (velocity, acceleration, steering)
- `/external/selected/gear_cmd` - Gear commands (PARK/REVERSE/DRIVE)

**Topics Subscribed:**
- `/control/current_gate_mode` - Current gate mode
- `/api/autoware/get/engage` - Engage status
- `/vehicle/status/velocity_status` - Current velocity
- `/vehicle/status/gear_status` - Current gear

**Workflow:**
1. Launch AutoSDV system: `make launch`
2. Launch keyboard control: `ros2 launch control_test keyboard_control.launch.xml`
3. Click on the GUI window to focus it
4. Press `z` to toggle to EXTERNAL mode
5. Press `x` to set gear to DRIVE
6. Use arrow keys (↑/↓/←/→) to control speed and steering

**GUI Features:**
- Real-time status display (engage, mode, gear, speed, angle)
- **Output topic presets** - Choose from External, Direct, or Custom topics
- **Custom topic entry** - Define your own control/gear topics
- Color-coded interface for easy reading
- Built-in help text and controls reference
- No TTY required - works with launch files

### 5. `keyboard_pwm_control` (Non-ROS Script)
Direct keyboard control of motor and steering PWM values.

**⚠️ Moved to scripts/**: This tool has been converted to a standalone non-ROS script.

**Location:** `/scripts/control/keyboard_pwm_control.py`

**Usage:**
```bash
# Run from AutoSDV root directory; the car is required (its stop PWM differs)
./scripts/control/keyboard_pwm_control.py --vehicle-id car_c

# Custom step sizes, or try a value before writing it into the profile
./scripts/control/keyboard_pwm_control.py --vehicle-id car_c --motor-step 2 --stop 385
```

Stop, brake, the motor and steering clamps and the steering centre come from
the car's profile (`individual_params/config/<vehicle_id>/autosdv_vehicle/
actuator.param.yaml` over `actuator.yaml`); `--help` lists the overrides.

**Keyboard Controls:**
- `w/s`: Increase/Decrease motor PWM
- `q`: Emergency stop (motor PWM = the car's `init_pwm`)
- `a/d`: Left/Right steering
- `c`: Center steering
- `q`: Quit
- `h`: Show help

## Launch Files

### `control_test.launch.xml`
Full vehicle interface stack for control testing, including:
- ZED IMU sensor
- Velocity report (speedometer)
- Actuator node
- Gear manager
- Control mode manager
- Steering status
- Signal manager
- Vehicle velocity converter

**Usage:**
```bash
ros2 launch control_test control_test.launch.xml
```

### `speedometer.launch.xml`
Standalone velocity report node for testing wheel speed sensor.

**Usage:**
```bash
ros2 launch control_test speedometer.launch.xml
```

## Workflow Examples

### Testing Hardware PWM
```bash
# Set motor to forward at PWM 380, steering centered
ros2 launch control_test static_pwm_test.launch.xml motor_pwm:=380 steering_pwm:=400
```

### PID Tuning
```bash
# Launch full vehicle stack
ros2 launch control_test control_test.launch.xml

# In another terminal, launch PID controller with keyboard input
ros2 launch control_test pid_tuning.launch.xml kp:=1.5 ki:=0.1 kd:=0.2

# Use keyboard to adjust target speed and observe response
```

### Autonomous Control Testing
```bash
# Launch full vehicle stack
ros2 launch control_test control_test.launch.xml

# Launch control command service with target speed
ros2 launch control_test control_command_service.launch.xml target_speed:=1.0

# Enable publishing
ros2 service call /control_command_service_node/enable example_interfaces/srv/SetBool "{data: true}"

# Monitor velocity
ros2 topic echo /vehicle/status/velocity_status
```

## Notes

- **Keyboard controllers** (pid_speed_control) require TTY access and cannot be run in background or systemd services.
- **keyboard_pwm_control** is now a standalone script in `scripts/control/` (non-ROS).
- **Service-based control** (control_command_service) is suitable for automated testing and integration with higher-level control systems.
- **PWM values** are calibrated for AutoSDV hardware:
  - Motor: 370=stop, 390-395=forward start, 350=reverse start
  - Steering: 400=center, 350=left limit, 450=right limit

## Dependencies

- `autosdv_vehicle_interface`: Vehicle interface nodes
- `autoware_control_msgs`: Control message definitions
- `simple-pid`: Python PID controller library
- `Adafruit_PCA9685`: PWM driver library

## See Also

- [Vehicle Interface Documentation](../autosdv_vehicle_launch/autosdv_vehicle_interface/README.md)
- [Control System Testing Guide](../../../docs/control_system_testing.md)

