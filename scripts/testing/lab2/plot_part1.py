#!/usr/bin/env python3
"""Draw a Lab 2 Part 1 run's curves to a PNG, with no display.

    just lab2 plot-sim tmp/lab2/part1/B12345678_lab_2_1/nominal
    python3 scripts/testing/lab2/plot_part1.py RUN_DIR [-o FILE] [--t0 S] [--t1 S]

RUN_DIR is one run's output directory, the one holding trace.csv (and marks.json).
A directory with a single variant below it (nominal/, low_mu/, ...) is also accepted.
Writes RUN_DIR/curves.png (or -o FILE) with matplotlib's Agg backend, so it works
over SSH and after a run with rviz:=false gui:=false, which saves no dashboard.png:

    speed      target (dashed), measured (what the controller receives), true (the plant)
    motor PWM  with neutral, the brake zone and the dead band shaded
    grey       the emergency stop
    red        each failed check's window, from marks.json, labelled with its name

--t0/--t1 zoom into a time window [s of simulated time], e.g. the 11 -> 6 m/s step down.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

PWM_DEFAULTS = {'init_pwm': 370, 'min_pwm': 360, 'max_pwm': 470, 'brake_pwm': 340}
BRAKE_DEADBAND = 5          # the ESC's dead band below neutral [counts]
SURFACE, INK_2, GRID = '#fcfcfb', '#52514e', '#e4e3df'
C_TRUE, C_MEASURED, C_TARGET = '#2a78d6', '#eb6834', '#52514e'
C_FAIL, C_ESTOP, C_BRAKE = '#e34948', '#b9b8b2', '#2a78d6'


def find_run(path: Path) -> Path:
    if (path / 'trace.csv').is_file():
        return path
    runs = sorted(p.parent for p in path.glob('*/trace.csv'))
    if len(runs) == 1:
        return runs[0]
    if runs:
        names = ', '.join(r.name for r in runs)
        raise SystemExit(f'{path} holds several runs ({names}); name one, e.g. {runs[0]}')
    raise SystemExit(f'no trace.csv in {path} (give the run directory, '
                     'tmp/lab2/part1/<name>/<variant>)')


def load(run: Path):
    cols = {k: [] for k in ('t', 'target', 'measured', 'speed', 'pwm', 'emergency')}
    with open(run / 'trace.csv', newline='') as f:
        for row in csv.DictReader(f):
            for k in cols:
                try:
                    cols[k].append(float(row[k]))
                except (KeyError, ValueError):
                    cols[k].append(float('nan'))
    marks = {}
    if (run / 'marks.json').is_file():
        try:
            marks = json.loads((run / 'marks.json').read_text())
        except ValueError:
            marks = {}
    return cols, marks


def spans(t, flags):
    out, start = [], None
    for ti, f in zip(t, flags):
        if f and start is None:
            start = ti
        elif not f and start is not None:
            out.append((start, ti))
            start = None
    if start is not None and t:
        out.append((start, t[-1]))
    return out


def plot(run: Path, out: Path, t0=None, t1=None) -> Path:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    d, marks = load(run)
    if not d['t']:
        raise SystemExit(f'{run / "trace.csv"} is empty')
    lv = dict(PWM_DEFAULTS, **(marks.get('pwm_levels') or {}))
    fig, (ax_v, ax_p) = plt.subplots(2, 1, sharex=True, figsize=(12, 7), facecolor=SURFACE,
                                     gridspec_kw={'height_ratios': [3, 2]},
                                     layout='constrained')
    for ax in (ax_v, ax_p):
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.8)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
    t = d['t']
    ax_v.plot(t, d['speed'], color=C_TRUE, lw=2.0, label='true', zorder=3)
    ax_v.plot(t, d['measured'], color=C_MEASURED, lw=1.0, label='measured', zorder=4)
    ax_v.plot(t, d['target'], '--', color=C_TARGET, lw=1.2, label='target', zorder=5)
    ax_v.set_ylabel('speed [m/s]', color=INK_2)
    ax_v.legend(loc='upper left', frameon=False, ncol=3, fontsize=9)

    ax_p.axhspan(lv['min_pwm'], lv['init_pwm'] - BRAKE_DEADBAND, color=C_BRAKE, alpha=0.15,
                 lw=0, label='brake')
    ax_p.axhspan(lv['init_pwm'] - BRAKE_DEADBAND, lv['init_pwm'], color=C_ESTOP, alpha=0.25,
                 lw=0, label='dead band')
    ax_p.axhline(lv['init_pwm'], color=INK_2, lw=1.0, label=f"neutral {lv['init_pwm']}")
    ax_p.plot(t, d['pwm'], color=C_TRUE, lw=1.3, label='motor PWM', zorder=3)
    ax_p.set_ylim(lv['brake_pwm'] - 8, lv['max_pwm'] + 8)
    ax_p.set_ylabel('motor PWM [counts]', color=INK_2)
    ax_p.set_xlabel('simulated time [s]', color=INK_2)
    ax_p.legend(loc='upper left', frameon=False, ncol=4, fontsize=9)

    for a, b in spans(t, [e > 0.5 for e in d['emergency']]):
        for ax in (ax_v, ax_p):
            ax.axvspan(a, b, color=C_ESTOP, alpha=0.3, lw=0)
        ax_v.text(a, 1.0, ' e-stop', transform=ax_v.get_xaxis_transform(), va='top',
                  fontsize=8, color=INK_2, clip_on=True)
    for i, v in enumerate(marks.get('violations') or []):
        a, b = v.get('t0'), v.get('t1')
        if a is None:
            continue
        b = a + 0.2 if b is None or b <= a else b
        for ax in (ax_v, ax_p):
            ax.axvspan(a, b, color=C_FAIL, alpha=0.12, lw=0)
            if v.get('t') is not None:
                ax.axvline(v['t'], color=C_FAIL, lw=1.0)
        ax_v.text(a, 0.92 - 0.07 * (i % 6), f" {v.get('name', '?')}",
                  transform=ax_v.get_xaxis_transform(), va='top', fontsize=8, color=C_FAIL, clip_on=True)

    title = f"{run.parent.name} / {run.name}"
    if marks:
        if marks.get('valid', True):
            title += f"   {marks.get('passed', '?')}/{marks.get('total', '?')} checks passed"
        else:
            title += f"   INVALID: {marks.get('invalid_reason')}"
    ax_v.set_title(title, loc='left', fontsize=11)
    if t0 is not None or t1 is not None:
        ax_v.set_xlim(t0 if t0 is not None else t[0], t1 if t1 is not None else t[-1])
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Draw a Part 1 run's trace.csv to a PNG "
                                             '(no display needed).')
    ap.add_argument('run_dir', type=Path, help='tmp/lab2/part1/<name>/<variant>')
    ap.add_argument('-o', '--out', type=Path, default=None,
                    help='the PNG to write (default RUN_DIR/curves.png)')
    ap.add_argument('--t0', type=float, default=None, help='zoom: from this time [s]')
    ap.add_argument('--t1', type=float, default=None, help='zoom: up to this time [s]')
    args = ap.parse_args(argv)
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        print('matplotlib is not installed: sudo apt install python3-matplotlib', file=sys.stderr)
        return 2
    run = find_run(args.run_dir.expanduser())
    out = args.out.expanduser() if args.out else run / 'curves.png'
    print(plot(run, out, args.t0, args.t1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
