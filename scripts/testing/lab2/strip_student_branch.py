#!/usr/bin/env python3
"""Strip the TA-only Lab 2 Part 1 material for the student branch (`Lab2`).

    python3 scripts/testing/lab2/strip_student_branch.py            # dry run: print the plan
    python3 scripts/testing/lab2/strip_student_branch.py --apply    # delete and edit the files

Run it on a checkout of the release commit, before committing the student branch. It
deletes the TA reference controller, its tuned parameters and its tests, and the
calibration and tolerance scripts. It then cuts the sections that name or quote the
reference out of the docs, the justfile, the runner's help, the limits table and the
metrics test. It never touches git: review `git status` / `git diff` afterwards, then
commit. Every rule must match exactly what it expects: a file or a section that moved is
reported as MISSING and nothing is half-applied. In that case, update this script.

After applying, `python3 -m pytest src/lab/autosdv_waypoint_lab/test` must still pass:
`test_speed_pid_part1.py` covers the stub, the given parts, the detuned defaults and the
handout without the reference.
"""
import argparse
from pathlib import Path
import re
import sys

REPO = Path(__file__).resolve().parents[3]
LAB = 'src/lab/autosdv_waypoint_lab'

# Files removed outright (TA-only).
DELETE = [
    f'{LAB}/autosdv_waypoint_lab/reference/speed_pid_part1.py',   # the TA reference controller
    f'{LAB}/config/speed_pid_part1_reference.yaml',               # its tuned parameters
    f'{LAB}/test/test_speed_pid_part1_reference.py',              # its tests
    'scripts/testing/lab2/part1_calibrate.py',                    # limits from the reference
    'scripts/testing/lab2/part1_tolerance.py',                    # load tolerance of the reference
    'docs/superpowers/plans/2026-10-18-lab2-part1-sim.md',        # the TA implementation plan
    'scripts/testing/lab2/strip_student_branch.py',               # this script (names the above)
]


def _cut_from(marker):
    """Delete from the line starting with `marker` to the end of the file."""
    def edit(text):
        i = text.find('\n' + marker)
        if i < 0:
            return None
        return text[:i + 1]
    return edit


def _cut_between(start, end):
    """Delete from the line starting with `start` up to (not including) the line `end`."""
    def edit(text):
        i = text.find(start)
        j = text.find(end, i + 1) if i >= 0 else -1
        if i < 0 or j < 0:
            return None
        return text[:i] + text[j:]
    return edit


def _replace(old, new):
    def edit(text):
        if text.count(old) != 1:
            return None
        return text.replace(old, new)
    return edit


def _limits_comments(text):
    """In metrics.py: the calibration note and each limit's 'worst ... / reference ...' tail."""
    start = '# Limits. Calibrated'
    end = 'LIMITS_CALIBRATED = True'
    i, j = text.find(start), text.find(end)
    if i < 0 or j < 0:
        return None
    text = (text[:i] + '# Limits: the physical ones and the stop error are fixed; the others are '
            'calibrated.\n' + text[j:])
    pattern = re.compile(r"^(\s+'[a-z_0-9]+': -?[0-9.]+,\s+#[^;(\n]*?)\s*[;(][^\n]*"
                         r"(?:worst|reference)[^\n]*$", re.M)
    text, n = pattern.subn(r'\1', text)
    return text if n else None


# (path, description, edit function returning the new text or None when it does not match)
EDITS = [
    ('docs/guides/lab2-part1-sim.md', 'the "TA notes" section (reference, calibrate, tolerance)',
     _cut_from('## TA notes')),
    ('docs/guides/lab2-part1-sim.md', "the calibration paragraph and the reference's worst cases",
     _cut_between('*calibrated*: 1.5', 'Each run prints the limit it was graded against.')),
    ('docs/guides/lab2-part1-sim.md', 'the calibration paragraph, its replacement text',
     _replace('The others are\nEach run prints', 'The others are\n*calibrated* by the course '
              'staff on a reference run of the course. Each run prints')),
    ('just/lab2.just', 'the `just lab2 sim --class <reference>` example',
     _cut_between('#   just lab2 sim --class autosdv_waypoint_lab.reference',
                  '# No submission anywhere')),
    ('scripts/testing/lab2/part1_run.py', 'the help text naming the TA reference',
     _replace('''    --class/--params   a controller by import path, e.g. the TA reference
                       autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID with the
                       parameters of config/speed_pid_part1.yaml
''', '''    --class/--params   a controller by import path, with its parameters
''')),
    (f'{LAB}/autosdv_waypoint_lab/part1/metrics.py',
     "the calibration note and the reference's worst cases beside the limits", _limits_comments),
    (f'{LAB}/test/test_part1_metrics.py',
     "the reference's worst cases and the test that re-derives the limits (uses part1_calibrate)",
     _cut_between("# the reference's worst case per calibrated check",
                  'def test_fixed_limits_unchanged')),
]

# Left as they are, on purpose (no reference code or tuned values): the design doc names
# where the reference lives; the launch file defaults to the stub; config/speed_pid_part1.yaml
# is the students' detuned starting point (= the handout's).
REVIEW = [
    'docs/design/lab2-part1-sim-design.md  (names the reference and quotes calibration/'
    'tolerance numbers; decide whether the student branch keeps the design doc)',
]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--apply', action='store_true', help='delete and edit (default: dry run)')
    args = ap.parse_args(argv)
    problems, plan = [], []
    for rel in DELETE:
        p = REPO / rel
        (plan if p.exists() else problems).append(
            f'delete  {rel}' if p.exists() else f'MISSING {rel} (already removed?)')
    new_texts = {}
    for rel, what, edit in EDITS:
        p = REPO / rel
        if not p.is_file():
            problems.append(f'MISSING {rel}')
            continue
        old = new_texts.get(p, p.read_text())     # edits of one file apply in order
        new = edit(old)
        if new is None or new == old:
            problems.append(f'MISSING {rel}: {what} (the section moved; update this script)')
            continue
        removed = len(old.splitlines()) - len(new.splitlines())
        plan.append(f'edit    {rel}: {what} ({removed} lines removed)')
        new_texts[p] = new
    print('Student-branch strip, ' + ('APPLY' if args.apply else 'dry run'))
    for line in plan:
        print('  ' + line)
    for line in problems:
        print('  ' + line)
    print('  review (not changed):')
    for line in REVIEW:
        print('    ' + line)
    if problems:
        print('not applying: fix the MISSING entries first' if args.apply else '')
        return 1
    if args.apply:
        for rel in DELETE:
            (REPO / rel).unlink()
        for p, text in new_texts.items():
            p.write_text(text)
        print('done; review `git status` and `git diff`, then run the tests')
    return 0


if __name__ == '__main__':
    sys.exit(main())
