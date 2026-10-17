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

PKG_DIR = PKG / 'autosdv_rc_sim'
IFACE_DIR = VEHICLE_INTERFACE / 'autosdv_vehicle_interface'
# imu.py imports random (seeded noise for the ROS-side IMU node); the lockstep harness never imports it.
EXEMPT = {'imu.py', '__init__.py'}
HARNESS_IFACE = ('actuator_core.py', 'longitudinal.py', 'safety.py', 'speed_controller.py',
                 'velocity_estimator.py')
LOCKSTEP_MODULES = sorted(p for p in PKG_DIR.glob('*.py') if p.name not in EXEMPT) + [
    IFACE_DIR / f for f in HARNESS_IFACE]
FORBIDDEN_MATH = {'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'exp2', 'expm1',
                  'log', 'log1p', 'log2', 'log10', 'pow', 'sinh', 'cosh', 'tanh', 'asinh',
                  'acosh', 'atanh', 'hypot', 'erf', 'erfc', 'gamma', 'lgamma', 'cbrt'}
# The one documented exception: ActuatorCore.run_steering_control is production code, moved verbatim
# from the actuator. It calls math.tan/atan on the steering command; its result is truncated to an
# integer PWM, so a last-bit libm difference almost never changes the output. That is NOT proof:
# the single-CPU stress test below cannot see a cross-CPU libm difference. Only the golden digests,
# run on a second machine, can. Nothing else is allowed.
ALLOWED_MATH = {'actuator_core.py': {'tan', 'atan'}}
# detmath.py is the deterministic replacement for libm, so it gets the narrowest list: it may touch
# only correctly rounded or exact operations of math.
DETMATH_ONLY = {'sqrt', 'ldexp', 'pi', 'inf'}


def offences(path):
    tree = ast.parse(path.read_text())
    allowed = ALLOWED_MATH.get(path.name, ())
    found = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.BinOp, ast.AugAssign)) and isinstance(node.op, ast.Pow):
            operands = ((node.left, node.right) if isinstance(node, ast.BinOp)
                        else (node.target, node.value))
            ints = all(isinstance(n, ast.Constant) and type(n.value) is int for n in operands)
            if not ints:
                found.append(f'{path.name}:{node.lineno}: ** (libm pow)')
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'pow':
            found.append(f'{path.name}:{node.lineno}: builtin pow')
        if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                and node.value.id == 'math' and node.attr not in allowed):
            if node.attr in FORBIDDEN_MATH or (
                    path.name == 'detmath.py' and node.attr not in DETMATH_ONLY):
                found.append(f'{path.name}:{node.lineno}: math.{node.attr}')
        if isinstance(node, ast.Import):
            for a in node.names:
                if a.name == 'math' and a.asname not in (None, 'math'):
                    found.append(f'{path.name}:{node.lineno}: aliased math import')
        if isinstance(node, ast.ImportFrom) and node.module == 'math':
            bad = [a.name for a in node.names
                   if a.name == '*' or (a.name in FORBIDDEN_MATH and a.name not in allowed)
                   or (path.name == 'detmath.py' and a.name not in DETMATH_ONLY)]
            if bad:
                found.append(f'{path.name}:{node.lineno}: from math import {bad}')
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [a.name for a in node.names]
            if isinstance(node, ast.ImportFrom) and node.module:
                names.append(node.module)
            if any(n.split('.')[0] in ('numpy', 'random') for n in names):
                found.append(f'{path.name}:{node.lineno}: imports numpy/random')
    return found


def test_every_module_is_linted_or_exempt():
    on_disk = {p.name for p in PKG_DIR.glob('*.py')}
    linted = {p.name for p in LOCKSTEP_MODULES}
    assert on_disk == (linted & on_disk) | EXEMPT
    assert 'detmath.py' in linted
    assert set(HARNESS_IFACE) <= linted
    assert all(p.exists() for p in LOCKSTEP_MODULES)


def test_the_lint_catches_what_it_claims(tmp_path):
    bad = tmp_path / 'detmath.py'
    bad.write_text('import math as m\nfrom math import *\nfrom math import sin\n'
                   'import math\nx = math.exp(1.0)\ny = math.sqrt(2.0) ** 0.5\ny **= z\n'
                   'w = pow(2.0, 0.5)\nv = math.asinh(1.0)\nu = math.floor(1.5)\n')
    found = '\n'.join(offences(bad))
    for needle in ('aliased', "['*']", "['sin']", 'math.exp', '** (libm', 'builtin pow',
                   'math.asinh', 'math.floor'):
        assert needle in found, needle


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
        assert all(h.is_alive() for h in hogs)
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
