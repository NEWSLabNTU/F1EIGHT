#!/usr/bin/env bash
# One live speed-controller step test on a car, with the motor capped low.
#
#   scripts/testing/speed_control/run_trial.sh VEHICLE_ID NAME [actuator -p overrides...] [-- step_test options...]
#   scripts/testing/speed_control/run_trial.sh car_c base
#   MAX_PWM=405 scripts/testing/speed_control/run_trial.sh car_c kp8 -p kp_speed:=8.0 -- --profile 0:1,0.3:2,0:2
#
# Runs only velocity_report and the actuator (no Autoware: nothing else may
# publish /control/command/control_cmd), with the car's individual_params profile.
#
# Safety:
#   - max_pwm is forced LAST on the actuator's command line, so no override can
#     raise it. Default: the car's forward_start_pwm + 10. Refused above
#     forward_start_pwm + 25 or below init_pwm.
#   - step_test.py aborts on stall, overspeed, distance, lost speed reports and
#     run time (see its docstring); its stall threshold is forward_start_pwm + 3.
#   - the whole step test is killed after 30 s; the actuator's own watchdog
#     brakes and goes to neutral 0.3 s after the last command.
# Stop the full stack first (`just launch` publishes control_cmd too).
set -u
if [ $# -lt 2 ]; then sed -n '2,6p' "$0"; exit 2; fi
VEHICLE_ID=$1; NAME=$2; shift 2
ACT_ARGS=(); while [ $# -gt 0 ] && [ "$1" != "--" ]; do ACT_ARGS+=("$1"); shift; done
[ "${1:-}" = "--" ] && shift

ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
HERE=$ROOT/scripts/testing/speed_control
cd "$ROOT"
OUT=tmp/speed_control/$VEHICLE_ID; mkdir -p "$OUT"
set +u; source /opt/autoware/1.5.0/setup.bash; source install/setup.bash; set -u

L=install/autosdv_vehicle_interface/lib/autosdv_vehicle_interface
VI=install/autosdv_vehicle_interface/share/autosdv_vehicle_interface/params
IP=install/individual_params/share/individual_params/config/$VEHICLE_ID/autosdv_vehicle/actuator.param.yaml
VINFO=install/autosdv_vehicle_description/share/autosdv_vehicle_description/config/vehicle_info.param.yaml
if [ ! -f "$IP" ]; then echo "no profile for vehicle_id '$VEHICLE_ID': $IP"; exit 2; fi

read -r INIT_PWM FWD_START < <(python3 - "$IP" <<'EOF'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
print(p['init_pwm'], p.get('forward_start_pwm', 0))
EOF
)
if [ -z "${FWD_START:-}" ] || [ "$FWD_START" -le 0 ]; then
  echo "$VEHICLE_ID has no measured forward_start_pwm; measure the ESC first (docs/design/esc-pwm-model.md)"; exit 2
fi
MAX_PWM=${MAX_PWM:-$((FWD_START + 10))}
if [ "$MAX_PWM" -gt $((FWD_START + 25)) ] || [ "$MAX_PWM" -lt "$INIT_PWM" ]; then
  echo "refusing MAX_PWM=$MAX_PWM for $VEHICLE_ID (allowed $INIT_PWM..$((FWD_START + 25)))"; exit 2
fi
STALL_PWM=$((FWD_START + 3))
echo "$VEHICLE_ID: init_pwm $INIT_PWM, forward_start_pwm $FWD_START -> max_pwm $MAX_PWM, stall at $STALL_PWM"

if pgrep -f "lib/autosdv_vehicle_interface/actuator" >/dev/null; then
  echo "an actuator is already running; stop it (or the stack) first"; exit 2
fi

$L/velocity_report --ros-args -r __ns:=/autosdv -r __node:=velocity_report_node \
  --params-file $VI/velocity_report.yaml -r '~/output/velocity_status:=/vehicle/status/velocity_status' \
  > "$OUT/$NAME.velocity.log" 2>&1 &
VPID=$!
$L/actuator --ros-args -r __ns:=/autosdv -r __node:=actuator_node \
  --params-file $VI/actuator.yaml --params-file $VINFO --params-file "$IP" -p vehicle_id:=$VEHICLE_ID "${ACT_ARGS[@]}" \
  -p max_pwm:=$MAX_PWM \
  -r '~/input/control_cmd:=/control/command/control_cmd' -r '~/input/velocity_status:=/vehicle/status/velocity_status' \
  > "$OUT/$NAME.actuator.log" 2>&1 &
APID=$!
cleanup() {
  kill -INT $APID $VPID 2>/dev/null
  for _ in 1 2 3 4 5 6; do kill -0 $APID 2>/dev/null || kill -0 $VPID 2>/dev/null || break; sleep 0.5; done
  kill -9 $APID $VPID 2>/dev/null; true
}
trap cleanup EXIT
sleep 4
if ! kill -0 $APID 2>/dev/null; then echo "actuator died:"; tail -5 "$OUT/$NAME.actuator.log"; exit 1; fi
grep -E "PCA9685|DRY-RUN|refus|vehicle_id" "$OUT/$NAME.actuator.log" | head -3
GOT=$(ros2 param get /autosdv/actuator_node max_pwm 2>&1 | grep -oE '[0-9]+$')
echo "max_pwm read back: $GOT"
if [ "$GOT" != "$MAX_PWM" ]; then echo "max_pwm is not the forced cap; not driving"; exit 1; fi
timeout --signal=INT --kill-after=3 30 python3 "$HERE/step_test.py" "$OUT/$NAME.csv" --stall-pwm "$STALL_PWM" "$@" 2>&1 | grep -v "^\[INFO\]"
