#!/usr/bin/env python3
"""Show the vehicle gate failing: deliberately broken team models, each must FAIL.

    python3 scripts/testing/lab2/vehicle_gate_selftest.py [--cars car_a] [--keep]

Generates team models with `new_team.py` into tmp/lab2_gate_selftest/ws/src, gives each the
reference AckermannSpeedPID (a fresh template's SpeedPID is the student stub, which does not
drive, so every stop check would be "not tested"), breaks each the way a team plausibly would, builds the workspace over this repository's install, runs
the safety gate (vehicle_gate.py) on them, and checks each fails the check it should:

    team901_vehicle  wrong neutral   its tuning file sets init_pwm: 375, five counts off car A's
                                     380 and inside its ESC band, so the actuator's own
                                     profile check lets it through                -> neutral
    team902_vehicle  no watchdog     control_cmd_timeout: 1000 (the last command is held)
                                                                                 -> watchdog
    team903_vehicle  reverse arming  brake_pwm: 330, below every car's reverse_arm_pwm, with
                                     the actuator's own refusal of such a profile removed
                                                                                 -> no reverse

Exit status 0 when every broken model failed as expected (and nothing else went wrong).
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import new_team as nt  # noqa: E402

REPO = HERE.parents[2]
WORK = REPO / 'tmp/lab2_gate_selftest'

BROKEN = {
    901: ('wrong neutral', 'neutral', {'init_pwm': 375}, None),
    902: ('no watchdog', 'watchdog', {'control_cmd_timeout': 1000.0}, None),
    903: ('reverse arming', 'no reverse', {'brake_pwm': 330},
          ('longitudinal.py', '    if problems:\n        raise ValueError("car profile (ESC thresholds): "',
           '    if problems and False:   # BROKEN on purpose (vehicle_gate_selftest.py)\n'
           '        raise ValueError("car profile (ESC thresholds): "')),
}


WORKING = 'speed_controller:AckermannSpeedPID'


def break_model(root: Path, model: str, params: dict, patch) -> None:
    launch = root / f'{model}_launch/launch/vehicle_interface.launch.xml'
    text = launch.read_text()
    old = f'default="{model}_interface.speed_pid:SpeedPID"'
    if text.count(old) != 1:
        raise SystemExit(f'selftest: {launch} has no {old}; update break_model')
    launch.write_text(text.replace(old, f'default="{model}_interface.{WORKING}"'))
    for tuning in (root / f'{model}_launch/config').glob('*/speed_controller.param.yaml'):
        tuning.write_text(tuning.read_text()
                          + ''.join(f'    {k}: {v}\n' for k, v in params.items()))
    if patch:
        name, old, new = patch
        f = root / f'{model}_interface/{model}_interface/{name}'
        text = f.read_text()
        if text.count(old) != 1:
            raise SystemExit(f'selftest: {f} no longer has the line to break; update BROKEN')
        f.write_text(text.replace(old, new))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--cars', default='car_a', help='the cars the gate runs (default car_a)')
    ap.add_argument('--keep', action='store_true', help='keep the generated workspace')
    ap.add_argument('--from-worktree', action='store_true',
                    help='generate from the working trees, not HEAD (testing uncommitted edits)')
    args = ap.parse_args(argv)
    shutil.rmtree(WORK, ignore_errors=True)
    src = WORK / 'ws/src'
    src.mkdir(parents=True)
    files, source = nt.read_reference(worktree=args.from_worktree)
    lab = nt.read_lab_files(worktree=args.from_worktree)
    ids = nt.vehicle_ids(worktree=args.from_worktree)
    for n, (what, _, params, patch) in BROKEN.items():
        root = nt.generate(n, src, files, source, lab, ids)
        break_model(root, nt.model_name(n), params, patch)
        print(f'{nt.model_name(n)}: {what}', flush=True)
    ws = WORK / 'ws'
    build = subprocess.run(
        ['bash', '-c', f'source {REPO}/install/setup.bash && colcon build --base-paths src '
         '--symlink-install --event-handlers console_direct- 2>&1 | tail -3'], cwd=ws)
    if build.returncode:
        print('selftest: the broken models did not build')
        return 2
    bad = 0
    for n, (what, check, _, _) in BROKEN.items():
        model = nt.model_name(n)
        out = WORK / 'gate'
        subprocess.run(['bash', '-c', f'source {REPO}/scripts/env.sh >/dev/null 2>&1; '
                        f'set +u; source {ws}/install/setup.bash && python3 '
                        f'{HERE}/vehicle_gate.py vehicle_model:={model} --cars {args.cars} '
                        f'--workdir {out}'], cwd=REPO, env=dict(os.environ))
        for car in args.cars.split(','):
            f = out / model / f'{car}.json'
            res = json.loads(f.read_text()) if f.is_file() else {'ok': False, 'checks': []}
            failed = [c['check'] for c in res.get('checks', []) if not c['ok']]
            if res.get('error'):
                failed.append('start: ' + res['error'][:120])
            expected = check in failed
            print(f"{'OK  ' if expected and not res.get('ok') else 'BAD '} {model} ({what}) on "
                  f"{car}: failed {', '.join(failed) or 'nothing'}"
                  + ('' if expected else f'; expected {check} to fail'), flush=True)
            bad += 0 if expected and not res.get('ok') else 1
    if not args.keep:
        shutil.rmtree(ws, ignore_errors=True)
    print('\nselftest: every broken model was refused' if not bad
          else f'\nselftest: {bad} broken model(s) NOT refused as expected')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
