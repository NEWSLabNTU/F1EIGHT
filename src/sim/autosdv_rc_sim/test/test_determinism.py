"""Grading is bit-identical: under CPU load, across runs, and (golden file) across machines."""
import ast
import json
import multiprocessing
import os
from pathlib import Path

from conftest import PKG, VEHICLE_INTERFACE

from autosdv_rc_sim import grade

REFERENCE = PKG / 'config' / 'reference_speed_controller.yaml'
GOLDEN = Path(__file__).resolve().parent / 'golden' / 'nominal_digests.json'

LOCKSTEP_MODULES = [PKG / 'autosdv_rc_sim' / f for f in (
    'params.py', 'esc.py', 'drivetrain.py', 'tires.py', 'wheels.py', 'servo.py', 'hall.py',
    'chassis.py', 'plant.py', 'track.py', 'lockstep.py', 'grade.py')] + [
    VEHICLE_INTERFACE / 'autosdv_vehicle_interface' / f for f in (
        'actuator_core.py', 'longitudinal.py', 'speed_controller.py', 'safety.py',
        'velocity_estimator.py')]
FORBIDDEN_MATH = {'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'expm1', 'log',
                  'log1p', 'log2', 'log10', 'pow', 'sinh', 'cosh', 'tanh', 'hypot', 'erf', 'gamma'}
# The one documented exception: ActuatorCore.run_steering_control is
# production code, moved verbatim from the actuator. It calls math.tan/atan on the steering command, and
# its result is truncated to an integer PWM; the straight track commands about zero steering, and the
# golden digests and the CPU-stress test below catch a last-bit difference. Nothing else is allowed.
ALLOWED_MATH = {'actuator_core.py': {'tan', 'atan'}}


def offences(path):
    tree = ast.parse(path.read_text())
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Pow):
            ints = all(isinstance(n, ast.Constant) and type(n.value) is int
                       for n in (node.left, node.right))
            if not ints:
                found.append(f'{path.name}:{node.lineno}: ** (libm pow)')
        if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                and node.value.id == 'math' and node.attr in FORBIDDEN_MATH
                and node.attr not in ALLOWED_MATH.get(path.name, ())):
            found.append(f'{path.name}:{node.lineno}: math.{node.attr}')
        if isinstance(node, ast.ImportFrom) and node.module == 'math':
            bad = [a.name for a in node.names
                   if a.name in FORBIDDEN_MATH and a.name not in ALLOWED_MATH.get(path.name, ())]
            if bad:
                found.append(f'{path.name}:{node.lineno}: from math import {bad}')
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [a.name for a in node.names]
            if isinstance(node, ast.ImportFrom) and node.module:
                names.append(node.module)
            if any(n.split('.')[0] in ('numpy', 'random') for n in names):
                found.append(f'{path.name}:{node.lineno}: imports numpy/random')
    return found


def test_lockstep_path_is_free_of_libm():
    found = [o for path in LOCKSTEP_MODULES for o in offences(path)]
    assert not found, 'non-deterministic calls on the lockstep path:\n' + '\n'.join(found)


def _busy(stop):
    x = 0
    while not stop.is_set():
        x += 1


def test_bit_identical_under_cpu_load(tmp_path):
    params = grade.load_submission(REFERENCE)
    quiet = grade.evaluate(params, variants=['nominal'], out_dir=tmp_path / 'quiet')
    ctx = multiprocessing.get_context('spawn')
    stop = ctx.Event()
    hogs = [ctx.Process(target=_busy, args=(stop,)) for _ in range(min(4, os.cpu_count() or 2))]  # modest: shared host
    for h in hogs:
        h.start()
    try:
        loaded = grade.evaluate(params, variants=['nominal'], jobs=2, out_dir=tmp_path / 'loaded')
    finally:
        stop.set()
        for h in hogs:
            h.join(timeout=10)
            if h.is_alive():
                h.kill()
                h.join()
    assert loaded == quiet
    for name in ('nominal_profile.csv', 'nominal_estop.csv'):
        assert (tmp_path / 'quiet/traces' / name).read_bytes() == (tmp_path / 'loaded/traces' / name).read_bytes()


def test_golden_digests():
    """Any machine, any CPU: the reference run's traces hash to the committed digests.

    After an intended model change: RC_SIM_UPDATE_GOLDEN=1 python3 -m pytest test/test_determinism.py
    and commit the new file with the change that caused it.
    """
    digests = grade.evaluate(grade.load_submission(REFERENCE), variants=['nominal'])['traces']
    if os.environ.get('RC_SIM_UPDATE_GOLDEN') == '1':
        GOLDEN.parent.mkdir(exist_ok=True)
        GOLDEN.write_text(json.dumps(digests, indent=2, sort_keys=True) + '\n')
    assert digests == json.loads(GOLDEN.read_text())


def test_the_allow_list_is_the_steering_code_and_nothing_wider():
    """The exception covers math.tan/atan in actuator_core.py, and only there."""
    assert ALLOWED_MATH == {'actuator_core.py': {'tan', 'atan'}}
    source = (VEHICLE_INTERFACE / 'autosdv_vehicle_interface' / 'actuator_core.py').read_text()
    used = {node.attr for node in ast.walk(ast.parse(source))
            if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
            and node.value.id == 'math' and node.attr in FORBIDDEN_MATH}
    assert used == {'tan', 'atan'}                      # the allow-list is exactly what the file uses
