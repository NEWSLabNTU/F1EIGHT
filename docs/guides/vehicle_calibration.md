# Vehicle Interface Calibration

This guide covers motor and steering PWM calibration for the AutoSDV vehicle.

## Motor PWM Control

| State | PWM Value | Description |
|-------|-----------|-------------|
| Stop | 370 | Neutral position |
| Forward | > 370 | e.g., 380, 390, 400+ |
| Reverse | < 370 | e.g., 360, 350, 340- |
| Brake | 340 | Used for forward→reverse transition |

### Brake Sequence for Reverse

When transitioning from forward to reverse:
1. If moving forward (PWM > 370), set to 340 to engage brake
2. Return to 370 (stop position)
3. Then decrease below 370 for reverse motion

## Steering PWM Control

| Position | PWM Value |
|----------|-----------|
| Center | 400 |
| Left limit | 350 |
| Right limit | 450 |

- **Range**: Symmetrical ±50 PWM units from center

**Known Issue**: Steering direction is reversed (left/right inverted)

## Testing Tools

| Tool | Purpose |
|------|---------|
| `motor_pwm_interactive.py --vehicle-id <car>` | Interactive PWM control |
| `stop_motor.py --vehicle-id <car>` | Emergency stop (PWM = the car's neutral: 370 car A, 395 car B) |
| `test_steering_pwm.py --vehicle-id <car>` | Steering calibration |
| `scripts/control/gpio_speed.py` | Non-ROS GPIO speed measurement |
| `scripts/control/gpio_read.py` | GPIO pin state monitor |

## Control System Testing

See `docs/guides/control_testing.md` for comprehensive procedures.

### Quick Start

```bash
make test-control   # PID controller + speedometer + monitor (tmux)
make plot-test      # Launch PlotJuggler for visualization
make controller     # Keyboard manual control (requires RViz "Local" mode)
```

### Debug Topics

- `/autosdv/actuator_node/debug/{control_values,pwm_values,pid_values}`
- `/pid_speed_control_node/debug/{control_values,pwm_values,pid_values}`

### PlotJuggler Workflow

1. Terminal 1: `make test-control`
2. Terminal 2: `make plot-test`
3. In PlotJuggler: Click "Start" to stream topics
4. Use `w/s` keys to change target speed
5. Tune PID: `ros2 launch control_test pid_tuning.launch.xml kp:=10.0 ki:=0.2 kd:=0.5`

### PID Tuning Guidelines

| Symptom | Action |
|---------|--------|
| Slow response | Increase Kp |
| Oscillation | Decrease Kp, increase Kd |
| Steady-state error | Increase Ki |
| Noisy control | Decrease all gains, increase speed filter |
| Derivative noise | Enable differential_on_measurement, increase speed_filter_size |

## Manual vs Autonomous Control

- **Manual control**: Set "Local" mode in RViz, use keyboard controller
- **Autonomous**: Set "Remote" mode in RViz, set pose and goal in RViz
