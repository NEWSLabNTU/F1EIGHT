# Lab 2 One Stack, One Speed Controller — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Part 1 and Part 2 of Lab 2 run the same stack software (only localization differs) and the same student `SpeedPID` class with a per-car parameter file; `lab2_waypoint` is retired.

**Architecture:** The Part 1 `SpeedPID` (feed-forward, PID, braking mode) gains an `integral_band` clause, which makes it a superset of Part 2's controller. `lab2_speed` absorbs Part 2's calibration, car recordings and speed bench, and gains `config/speed_pid_car.yaml`. The car's `waypoint_follow.launch.xml` gets `localization:=aruco|ground_truth` and `hardware:=car|sim` switches, and Part 1's launch becomes that file plus a test harness, with the e-stop injected through the command gate's own external-emergency service.

**Tech Stack:** ROS 2 Humble, Autoware 1.5.0, Python 3 (rclpy, pytest, PyYAML), play_launch XML launch files, LaTeX (the course spec).

**Spec:** `~/repos/ntu-sdv-course/2026/labs/Lab2/lab2-one-controller.md` (decisions D1–D12). Read it first.

## Global Constraints

- Two repositories: AutoSDV `develop` (`~/repos/AutoSDV`) and ntu-sdv-course `main` (`~/repos/ntu-sdv-course`). Commit and push each task in both when it touches both; push is authorized. Never force-push; `develop` stays linear (`git pull --rebase`).
- Submodule lockstep: a change inside a submodule is pushed before the superproject pin. (No task here should need a submodule change; if one does, follow CLAUDE.md "Lockstep".)
- Temp files in `~/repos/AutoSDV/tmp/`, never `/tmp`.
- ROS commands run under **bash**: `bash -c 'source ~/repos/AutoSDV/install/setup.bash; ...'`. The interactive shell is zsh/fish, where sourcing silently fails ("Package 'autosdv_rc_sim' not found").
- Launches through `play_launch` only; kill by process group, never `pkill -f` patterns that can match your own shell.
- One sim run at a time on the host (`flock tmp/bench.lock` for graded or calibration runs); each run is ~50 s.
- With `--symlink-install`, `.py`/`.yaml`/`.xml` edits apply without a rebuild; a **new** file in a ROS package needs `just build` (or `colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <pkg>`).
- The handout's `lab2_speed/lab2_speed/speed_pid.py` must equal AutoSDV's `src/lab/autosdv_waypoint_lab/autosdv_waypoint_lab/stub/speed_pid_part1.py` byte for byte. The handout's `selfcheck.py` `KEYS` must equal `submission.py`'s `part1_keys()`.
- The spec voice for `lab2.tex` and student READMEs: academic register, second person, no TA-only content outside `tanote` boxes; `make check` in `spec/` must report `"the student": 0`.
- Code style: match the surrounding file (comment density, naming, quote style).

## Review Focus

1. **Determinism under load after the e-stop moves to the gate (D11).** Graded runs must stay within the measured load tolerance (`part1_tolerance.py`). A service call lands at an arbitrary instant relative to the actuator tick, which the scenario's lockstep emergency timer existed to prevent. Task 6 measures this before anything depends on it.
2. **The car's launch with default arguments must be unchanged.** A node-list diff before and after the switches (Task 5), plus the gate passing.
3. **A Part 1 submission without `integral_band` in its yaml** must still be accepted (it is optional, default off): `check_part1_yaml` warns about a missing key and leaves it out, and the class default 20.0 applies.
4. **The car file has no `ff_*` keys**, and the calibration must fill them. A missing or uncalibrated calibration gives ff 0 with a warning, never a crash or a silent default from the sim's 5.27/0.1433.
5. **Old `lab2_waypoint` submission directories** are given to `just lab2 gate`: the error must name the new layout (`lab2_speed/` with `config/speed_pid_car.yaml`), not a stack trace.

---

### Task 1: `integral_band` in the Part 1 SpeedPID

**Files:**
- Modify: `src/lab/autosdv_waypoint_lab/autosdv_waypoint_lab/stub/speed_pid_part1.py` (param + TODO 5 text)
- Modify: `src/lab/autosdv_waypoint_lab/autosdv_waypoint_lab/reference/speed_pid_part1.py` (update())
- Modify: `src/lab/autosdv_waypoint_lab/config/speed_pid_part1.yaml`, `config/speed_pid_part1_reference.yaml`
- Modify: `scripts/testing/lab2/submission.py` (`part1_keys`)
- Test: `src/lab/autosdv_waypoint_lab/test/test_speed_pid_part1_reference.py`, `scripts/testing/lab2/test_submission.py`
- Course: `handout/lab2_speed/lab2_speed/speed_pid.py` (copy of the stub), `handout/lab2_speed/config/speed_pid.yaml`, `handout/lab2_speed/selfcheck.py` (`KEYS`, a band check)

**Interfaces:**
- Produces: param key `integral_band` (float, m/s, range (0, 20], default 20.0); attribute `self.integral_band` on the skeleton.

- [ ] **Step 1: Write the failing reference tests** (append to `test_speed_pid_part1_reference.py`, which has a `pid(**over)` helper with all gains 0 and `error_limit=100`):

```python
def test_integral_band_stops_integration_far_from_the_target():
    c = pid(ki=10.0, integral_limit=100.0, integral_band=0.25)
    c.update(1.0, 0.0, 0.1)            # |e| = 1.0 > 0.25: no integration
    assert c.integral == 0.0
    c.update(1.0, 0.9, 0.1)            # |e| = 0.1 <= 0.25: integrates 10 * 0.1 * 0.1
    assert c.integral == pytest.approx(0.1)


def test_integral_band_default_is_off():
    c = pid(ki=10.0, integral_limit=100.0)
    c.update(5.0, 0.0, 0.1)            # |e| = 5 m/s: still inside the default 20 m/s band
    assert c.integral == pytest.approx(5.0)
```

- [ ] **Step 2: Run, expect FAIL** (the first test: integral 1.0, not 0.0)

Run: `bash -c 'source install/setup.bash; cd src/lab/autosdv_waypoint_lab && python3 -m pytest -q test/test_speed_pid_part1_reference.py -k integral_band'`

- [ ] **Step 3: Implement.** In the stub's `__init__`, after `self.error_limit = ...`:

```python
        self.integral_band = float(get('integral_band', 20.0))  # integrate only when |e| <= this [m/s]
```

In the stub's TODO 5 text, the first anti-windup bullet becomes:

```python
        #   - integrate only when dt > 0, when the car is near the target
        #     (|e| <= integral_band: a start or a step is the feed-forward's job), and,
        #     if enable_conditional_integration is set, only when the PREVIOUS output
        #     was not saturated (self.saturated):
        #         integral += ki * e_d * dt
```

Add `integral_band` to the docstring's TODO summary line ("I with the integral clamp, the integration band and conditional integration"). In the reference's `update()`:

```python
        if (dt > 0.0 and abs(error) <= self.integral_band
                and not (self.enable_conditional_integration and self.saturated)):
            self.integral += self.ki * e_d * dt
```

Both yaml files: add after `error_limit`:

```yaml
integral_band: 20.0                    # integrate only when |error| <= this [m/s], (0, 20]; 20 = off on this course
```

`submission.py` `part1_keys()`: add after `'error_limit'`:

```python
        'integral_band': (float, 0.0, 20.0, False),
```

- [ ] **Step 4: Run the lab tests and the submission tests, expect PASS**

Run: `bash -c 'source install/setup.bash; cd src/lab/autosdv_waypoint_lab && python3 -m pytest -q test/ && cd ~/repos/AutoSDV && python3 -m pytest -q scripts/testing/lab2/test_submission.py'`
If `test_part1_defaults_resolve_to_every_key` counts keys, it now expects 16.

- [ ] **Step 5: Handout.** `cp` the stub to `handout/lab2_speed/lab2_speed/speed_pid.py`; add the same yaml line to `handout/lab2_speed/config/speed_pid.yaml`; add `'integral_band': (float, 0.0, 20.0, False),` to `selfcheck.py` `KEYS` after `error_limit`; add a check and register it in `CHECKS` after `('TODO 5: the integral', check_integral)`:

```python
def check_integral_band(cls):
    c = make(cls, ki=10.0, integral_limit=100.0, integral_band=0.25)
    c.update(1.0, 0.0, 0.1)
    expect(close(c.integral, 0.0),
           f'the integral grew to {c.integral:g} with |e| = 1.0 > integral_band 0.25 (TODO 5)')
    c.update(1.0, 0.9, 0.1)
    expect(close(c.integral, 0.1),
           f'the integral is {c.integral:g} after |e| = 0.1 for 0.1 s with ki 10; expected 0.1')
```

(`make`, `expect`, `close` are the file's own helpers; check their signatures and adapt the call if `make` takes the class differently.)
Run `python3 selfcheck.py` in the handout dir with the stub (TODO checks fail, rules pass) and with the reference copied in temporarily (all pass; restore the stub after).

- [ ] **Step 6: Commit both repos**

```bash
git -C ~/repos/AutoSDV add -A src/lab/autosdv_waypoint_lab scripts/testing/lab2/submission.py scripts/testing/lab2/test_submission.py
git -C ~/repos/AutoSDV commit -m "Lab 2 Part 1: integral_band, integrate only near the target (default off)"
git -C ~/repos/ntu-sdv-course add 2026/labs/Lab2/handout/lab2_speed
git -C ~/repos/ntu-sdv-course commit -m "Lab 2 handout: integral_band in lab2_speed"
```

(End each message with the session's Co-Authored-By / Claude-Session lines.)

---

### Task 2: `lab2_speed` absorbs the car kit; `lab2_waypoint` retired

**Files (all in `~/repos/ntu-sdv-course/2026/labs/Lab2/handout/`):**
- Move: `lab2_waypoint/lab2_waypoint/{calibration.py,bench/}` → `lab2_speed/lab2_speed/`
- Move: `lab2_waypoint/cars/`, `lab2_waypoint/exercise_tests/`, `lab2_waypoint/runs/` → `lab2_speed/`
- Move: `lab2_waypoint/config/calibration.yaml` → `lab2_speed/config/`
- Create: `lab2_speed/config/speed_pid_car.yaml`
- Modify: every `lab2_waypoint` import/path inside the moved files → `lab2_speed`; `bench/params.py`; `exercise_tests/conftest.py`; `lab2_speed/selfcheck.py` (`--car`); `lab2_speed/setup.py` (package_data for `bench/models/*.yaml`, if `lab2_waypoint/setup.py` had it)
- Delete: `handout/lab2_waypoint/`

**Interfaces:**
- Consumes: Task 1's `integral_band`.
- Produces: `lab2_speed.bench.params.speed_pid_params(path=None, calibration=None)` reading `config/speed_pid_car.yaml` by default; `lab2_speed.calibration` (same API as before: `load_calibration`, `is_calibrated`, `ff_params`, `CARS`); `python3 selfcheck.py --car` runs the contract tests + the bench and prints the bench score, as `lab2_waypoint/selfcheck.py` did.

- [ ] **Step 1: `git mv` the files** (keeps history):

```bash
cd ~/repos/ntu-sdv-course/2026/labs/Lab2/handout
git mv lab2_waypoint/lab2_waypoint/calibration.py lab2_speed/lab2_speed/calibration.py
git mv lab2_waypoint/lab2_waypoint/bench lab2_speed/lab2_speed/bench
git mv lab2_waypoint/cars lab2_speed/cars
git mv lab2_waypoint/exercise_tests lab2_speed/exercise_tests
git mv lab2_waypoint/runs lab2_speed/runs
git mv lab2_waypoint/config/calibration.yaml lab2_speed/config/calibration.yaml
```

- [ ] **Step 2: Create `config/speed_pid_car.yaml`** (D5):

```yaml
# The REAL car's SpeedPID parameters (Part 2): the same class as config/speed_pid.yaml, the
# simulated car's. Given and tested on the car bench (python3 selfcheck.py --car); a team may
# adjust them on the car. The car's motor gives about 0.03 m/s per count where the simulated
# car gives 0.14, so gains in counts per m/s are about five times the simulated car's.
# ff_deadband and ff_gain are NOT set here: they come from config/calibration.yaml.
kp: 20.0
ki: 25.0
kd: 0.0
integral_limit: 10.0
error_limit: 0.25
integral_band: 0.25
enable_conditional_integration: true
derivative_filter_alpha: 0.15
max_pwm_offset: 100
max_brake_offset: 10
kp_brake: 20.0
brake_enter: 0.2
brake_exit: 0.05
brake_deadband: 5
```

- [ ] **Step 3: Rename references.** `grep -rn lab2_waypoint lab2_speed/` and change each to `lab2_speed`. `bench/params.py`: default file `config_dir() / 'speed_pid_car.yaml'`, docstring says so, `get_package_share_directory('lab2_speed')`. `speed_bench.py`/`speed_eval.py`: `DEFAULT_CONTROLLER = 'lab2_speed.speed_pid:SpeedPID'`; help texts and examples `{i_band: 100.0}` → `{integral_band: 100.0}`, `i_max` → `integral_limit`. `exercise_tests/conftest.py`: `SPEED_CLASS = 'lab2_speed.speed_pid:SpeedPID'`, the own-module import rule names `lab2_speed`, and the parameter source is `speed_pid_car.yaml` + calibration. Any test using `i_band`/`i_max` uses `integral_band`/`integral_limit`.

- [ ] **Step 4: Merge `selfcheck.py`.** Keep `lab2_speed/selfcheck.py` as the base (Part 1 rules, KEYS, TODO checks). Port from the old `lab2_waypoint/selfcheck.py` (read it from git: `git show HEAD:2026/labs/Lab2/handout/lab2_waypoint/selfcheck.py`) the contract-test run, the bench score and the vendor-provenance check, behind `--car`. `check_yaml` also validates `speed_pid_car.yaml` against `KEYS` with `--car` (ff keys absent is fine there). `--submission` checks the layout `lab2_speed/{lab2_speed/speed_pid.py,config/speed_pid.yaml,config/speed_pid_car.yaml,config/calibration.yaml}` and the report.

- [ ] **Step 5: Verify the bench with the reference.** Copy the AutoSDV reference in place temporarily and calibrate to the public model:

```bash
cd ~/repos/ntu-sdv-course/2026/labs/Lab2/handout/lab2_speed
bash -c 'source ~/repos/AutoSDV/install/setup.bash; python3 -m lab2_speed.bench.speed_eval --controller autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID --params "{ff_deadband: 5.0, ff_gain: 0.03}"' | tee ~/repos/AutoSDV/tmp/lab2/bench_ref_car.txt
grep -E "E\.|\.R|\.S|O\.|  \.$" ~/repos/AutoSDV/tmp/lab2/bench_ref_car.txt   # expect no output: every cell passes
```

Expected: every hold row `EORS`, steps `UD`, ramp `R`, load `RW`, stall `S` (nominal stall ≈ 7.7 %). Then `python3 selfcheck.py` (stub) and `python3 -m pytest -q exercise_tests` with the reference copied in and `calibration.yaml` set to `{car: A, deadband: 5.0, gain: 0.03}`; restore the stub and the uncalibrated file afterwards.

- [ ] **Step 6: Delete the old package and commit**

```bash
git rm -r -q lab2_waypoint
git commit -m "Lab 2 handout: lab2_speed takes the calibration, the car bench and speed_pid_car.yaml; lab2_waypoint retired"
```

---

### Task 3: Part 2 loading, the gate tooling and the TA grader

**Files:**
- Modify: `scripts/testing/lab2/submission.py` (default mode → `lab2_speed` + car file + calibration; docstring)
- Modify: `scripts/testing/lab2/test_submission.py`
- Modify: `just/lab2.just` (comments at lines ~104–109), `scripts/testing/lab2/bench_gate.py` (docstring example), `scripts/testing/lab2/fuzz_bags.py` (`rglob('lab2_speed/speed_pid.py')`, mutants of the new class: check each mutant's source pattern still exists in the Part 1 stub-based reference), `scripts/testing/lab2/strip_student_branch.py` (if it lists `lab2_waypoint` paths)
- Course: `ta/grade.py` (`HANDOUT = .../lab2_speed`, `STUDENT_FILES`, `find_package`, `overlay_reference`), `ta/reference/` (delete Part 2 `speed_pid.py`/`speed_pid.yaml`; the overlay uses the AutoSDV reference via its import path, or a copy named `speed_pid.py` generated from `autosdv_waypoint_lab/reference/speed_pid_part1.py` + the stub; choose the copy, since `grade.py` assembles a pristine package without ROS)

**Interfaces:**
- Consumes: Task 2's layout and `speed_pid_car.yaml`.
- Produces: `submission.resolve(path)` → `(pkg_dir, ['speed_controller_class:=lab2_speed.speed_pid:SpeedPID', 'speed_controller_params:={...}'], warnings)`, params = checked car file + `ff_deadband`/`ff_gain` from calibration (car-file keys win). `CLASS = PART1_CLASS`.

- [ ] **Step 1: Update the tests first.** In `test_submission.py`, the `make()` helper builds `lab2_speed/{lab2_speed/speed_pid.py, config/speed_pid_car.yaml, config/calibration.yaml}`; expected class `lab2_speed.speed_pid:SpeedPID`. Add:

```python
def test_old_lab2_waypoint_layout_is_refused_with_the_new_name(tmp_path):
    (tmp_path / 'lab2_waypoint' / 'lab2_waypoint').mkdir(parents=True)
    (tmp_path / 'lab2_waypoint' / 'lab2_waypoint' / 'speed_pid.py').write_text('')
    with pytest.raises(submission.SubmissionError, match='lab2_speed'):
        submission.resolve(tmp_path)


def test_car_file_is_checked_like_part1(tmp_path):
    pkg = make(tmp_path)
    (pkg / 'config' / 'speed_pid_car.yaml').write_text('kp: -1.0\n')
    with pytest.raises(submission.SubmissionError, match='kp'):
        submission.resolve(tmp_path)


def test_car_file_without_ff_takes_the_calibration(tmp_path):
    make(tmp_path)          # calibration deadband 5.0, gain 0.03
    _, args, _ = submission.resolve(tmp_path)
    assert 'ff_deadband: 5.0' in args[1] and 'ff_gain: 0.03' in args[1]
```

(Adapt to the file's existing helper names and fixtures.)

- [ ] **Step 2: Run, expect FAIL.** `python3 -m pytest -q scripts/testing/lab2/test_submission.py`

- [ ] **Step 3: Implement** `package_dir` = `part1_package_dir`, refusing with a message naming `lab2_speed/` and `config/speed_pid_car.yaml` (and saying `lab2_waypoint` is retired) when only the old layout is present; `params(pkg)` = `check_part1_yaml(pkg / 'config' / 'speed_pid_car.yaml')` with the `ff_*` "not set" warnings dropped, then the calibration merged with `setdefault` as today; `CLASS = PART1_CLASS`; `team_controller` error text says `lab2_speed/`. Update the module docstring's first section.

- [ ] **Step 4: Run, expect PASS**, plus `python3 scripts/testing/lab2/submission.py <handout>/lab2_speed` prints the merged mapping.

- [ ] **Step 5: Rename in the tooling files listed above**; `grep -rn lab2_waypoint scripts/testing/lab2 just docs/guides` must return only historical mentions in prose you deliberately keep (none in code).

- [ ] **Step 6: TA grader.** Update `ta/grade.py` and `ta/reference/`; run `python3 ta/grade.py --selftest` (or its equivalent flag; read `main()`): the reference scores full bench marks on every car, the stub 0, and the calibration check unchanged.

- [ ] **Step 7: Gate with the reference submission.** Build a reference submission dir in `tmp/lab2/ref_sub/` (copy the handout `lab2_speed`, put the reference class in `speed_pid.py`, calibration from `ta/cars/car_A/calibration_ref.yaml`), then:

```bash
bash -c 'source install/setup.bash; just lab2 gate submission:=tmp/lab2/ref_sub'
```

Expected: `PASS` on `lab2_uturn`.

- [ ] **Step 8: Commit both repos** ("Lab 2 Part 2: the gate drives lab2_speed with the car file and the calibration").

---

### Task 4: Docs, spec and TA guide for the controller change

**Files:**
- Course: `spec/lab2.tex` (Task 1 table and algorithm: the band clause and key; Task 4 §`sec:carctl`: rewritten as "Configure Your SpeedPID for the Car": the package tree (lab2_speed only), calibration as before, the car file and why its gains are ~5× the sim's, the bench via `selfcheck.py --car`, the gate; delete the "Pending decision" tanote and the old feed-forward+PI algorithm; troubleshooting rows naming `lab2_waypoint`), rebuild `lab2.pdf`/`lab2-ta.pdf` (`make all check`)
- Course: `handout/lab2_speed/README.md` (the band in "What you write" and the parameter table; new sections "The car file", "Calibration", "The car bench", "The gate and the car", taken from the old `lab2_waypoint/README.md` in git history and renamed), `ta/lab2-ta-grading.md` §3 and §8, `lab2-part1-sim.md` (P2 row mentions the band; Part 2 sentence already updated)
- AutoSDV: `docs/guides/lab2-waypoint-stack.md`, `docs/guides/lab2-part1-sim.md` (the band key in the parameter list)

- [ ] **Step 1:** Edit the files. Keep the spec's voice (second person, academic, no "the student"); TA-only facts in `tanote`.
- [ ] **Step 2:** `cd spec && make all check` → `"the student": 0`; `pdftotext lab2.pdf - | grep -c lab2_waypoint` → 0.
- [ ] **Step 3:** Commit both repos ("Lab 2 docs: one SpeedPID for both parts").

---

### Task 5: `waypoint_follow.launch.xml` switches (D9)

**Files:**
- Modify: `src/launcher/autosdv_launch/launch/waypoint_follow.launch.xml`
- Modify: `src/lab/autosdv_waypoint_lab/launch/waypoint_nodes.launch.xml` (`launch_localization_monitor` arg)
- Test: node-list diff (below)

**Interfaces:**
- Produces: args `localization` (`aruco` | `ground_truth`, default `aruco`), `hardware` (`car` | `sim`, default `car`), `path_param_file`, `vehicle_cmd_gate_param_path`, `use_sim_time` (default false), `tracker_control_topic` (default `/control/trajectory_follower/control_cmd`), `tracker_lockstep` (default false), `output_emergency_cmd` (default the control component's own). Defaults reproduce today's car exactly.

- [ ] **Step 1: Record today's node set** (the car launch in dry-run, no hardware):

```bash
bash -c 'source install/setup.bash; play_launch dump autosdv_launch waypoint_follow.launch.xml is_simulation:=true launch_camera:=false > tmp/lab2/nodes_before.txt' || \
bash -c 'source install/setup.bash; ros2 launch --print autosdv_launch waypoint_follow.launch.xml is_simulation:=true launch_camera:=false > tmp/lab2/nodes_before.txt'
```

(`play_launch --help` shows the dump/parse subcommand that prints the record without running; use it. Save the node list with their parameters.)

- [ ] **Step 2: Add the args and conditions.** `ground_truth`: `zed_aruco`, `waypoint_localization` and `localization_monitor` are not launched (use `<let>` + `if`/`unless` on `$(eval "'$(var localization)' == 'aruco'")`), and the graph path defaults to `autosdv_waypoint_lab/config/part1/part1-diagnostics.yaml`'s generalisation: rename it `config/waypoint-ground-truth-diagnostics.yaml` only if Part 1 is its sole user (else keep the name). `sim`: `vehicle_interface.launch.xml` gets `hardware:=sim`, `is_simulation:=false`, and the `autosdv.launch.yaml` include is replaced by `tier4_vehicle_launch/vehicle.launch.xml` with `launch_vehicle_interface:=false` (as Part 1 does today; the sensor kit drivers are off anyway). Pass the new parameter-file args through to `waypoint_nodes.launch.xml` and `autosdv_pursuit_control_component.launch.xml`; `waypoint_nodes.launch.xml` gains `control_topic` and `lockstep` params for `path_tracker`, defaulting to today's values.

- [ ] **Step 3: Diff.** Repeat Step 1 into `nodes_after.txt`; `diff` must show no change in nodes or parameter values (ordering differences only).

- [ ] **Step 4: Gate still passes:** `just lab2 gate submission:=tmp/lab2/ref_sub` → PASS.

- [ ] **Step 5: Commit** ("waypoint_follow: localization and hardware switches; defaults unchanged").

---

### Task 6: Part 1 on the shared launch, e-stop through the gate (D10, D11)

**Files:**
- Modify: `src/launcher/autosdv_launch/launch/lab2_part1_sim.launch.xml`
- Modify: `src/lab/autosdv_waypoint_lab/autosdv_waypoint_lab/part1/scenario_node.py`
- Modify: `scripts/testing/lab2/part1_calibrate.py` outputs → `part1_profile`/limits files it writes; re-run
- Test: `part1_tolerance.py`, `part1_run.py --final` with the reference

**Interfaces:**
- Consumes: Task 5's switches.
- Produces: the same `/lab2/part1/event` events (`engaged`, `estop_on`, `estop_off`, `aborted`) with their `t`, so the grader is untouched.

- [ ] **Step 1: Spike D11 before rewriting.** Keep the current launch; in a scratch copy of the scenario node (`tmp/lab2/proto/`), replace the emergency publishing with calls to `/control/vehicle_cmd_gate/external_emergency_stop` and `.../clear_external_emergency_stop` (`std_srvs/srv/Trigger`; confirm the type with `ros2 service type` on a running stack) and drop the command pass-through (path_tracker publishing straight to `/control/trajectory_follower/control_cmd`). Run the reference `--final` three times and `part1_tolerance.py`. Record: does the gate's emergency reach the actuator (actuator mode `emergency`/brake), the e-stop's actuator-tick spread across runs, `estop_resume_*` values, and every other check's spread against the tolerance in `docs/design/lab2-part1-sim-design.md`.

- [ ] **Step 2: Decide from the measurement** and write the ruling into the plan's ledger:
  - spread within tolerance and the resume passes a limit ≤ 1.5× the reference's worst case → proceed with D11 as written;
  - resume fails but spread is fine → the fallback in D11 (the tracker's acceleration limit, a parameter);
  - spread out of tolerance (the e-stop no longer lands on a fixed tick) → **stop and report to the user**: D11 conflicts with the load-independent grading decided earlier, and the choice between them is theirs.

- [ ] **Step 3: Rewrite `lab2_part1_sim.launch.xml`** as: args as today; `<include waypoint_follow.launch.xml>` with `localization:=ground_truth hardware:=sim use_sim_time:=true course_dir:=... path_param_file:=$(var cfg)/waypoint_path_publisher.param.yaml vehicle_cmd_gate_param_path:=$(var cfg)/vehicle_cmd_gate_part1.param.yaml actuator_param_overrides:=<rc_sim's> speed_controller_*:=... tracker_lockstep:=$(var lockstep)`; then the harness only: `rc_sim_plant`, `lab2_part1_scenario` (no command or emergency remaps), the grader, the dashboard, RViz and the RViz-only lane map. Update the header comment (what differs from the car: localization, the car, parameter files, the harness).

- [ ] **Step 4: Rewrite the scenario's e-stop** per Step 2's ruling: keep the engage, the trigger and rest detection, and the events; remove `on_cmd`, `on_gate`, the emergency publisher, and the lockstep emergency participant if no longer needed (the plant's participant list in `rc_sim` then drops "emergency"; update `lockstep` participants accordingly). Keep the start alignment only if it works without touching the command (e.g. by timing the engage request); otherwise note in the ledger what the start spread becomes.

- [ ] **Step 5: Recalibrate and verify.** `flock tmp/bench.lock python3 scripts/testing/lab2/part1_calibrate.py ...` (read its docstring), `part1_tolerance.py`, then:

```bash
bash -c 'source install/setup.bash; python3 scripts/testing/lab2/part1_run.py --class autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID --params "$(python3 -c "import yaml;print(yaml.safe_dump(yaml.safe_load(open(\"src/lab/autosdv_waypoint_lab/config/speed_pid_part1_reference.yaml\")),default_flow_style=True,width=10000).strip())")" --final --no-rviz --no-gui --no-hold --name ref_after'
```

Expected: 48/48 (25.00). Then the stub (valid, every check fails) and the detuned defaults (about 35/48; update the number in the spec's tanote and `speed_pid_part1_reference.yaml` comment if it moved). Run the lab tests (`test_part1_*`), `scripts/testing/lab2/test_part1_run.py`, and `just sim grade` if it uses the Part 1 launch.

- [ ] **Step 6:** Update `docs/design/lab2-part1-sim-design.md` (the stack section, the e-stop, the new limits and tolerance), `docs/guides/lab2-part1-sim.md`, and the spec's Part 1 limits table if the numbers changed; `make all check`.

- [ ] **Step 7: Commit both repos** ("Lab 2 Part 1: the car's launch with ground-truth localization; e-stop through the gate").

---

### Task 7: Final verification

- [ ] **Step 1:** Spec verification list (lab2-one-controller.md §Verification) 1–6, each with its output saved under `tmp/lab2/verify/`.
- [ ] **Step 2:** `python3 scripts/testing/lab2/strip_student_branch.py` (dry run): lists the reference files and no `lab2_waypoint` leftovers.
- [ ] **Step 3:** `git -C ~/repos/AutoSDV status`, `git -C ~/repos/ntu-sdv-course status` clean except the untouched `src/localization/cuda_ndt_matcher`; push both.
- [ ] **Step 4:** Update `lab2-one-controller.md` with the measured outcomes (D11's ruling, the new limits) and commit.
