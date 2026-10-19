"""plot_part1.py: python3 -m pytest scripts/testing/lab2/test_plot_part1.py"""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_part1 as P  # noqa: E402

pytest.importorskip('matplotlib')


def write_run(d: Path, violations=()):
    d.mkdir(parents=True)
    rows = ['t,s,y,target,measured,speed,pwm,mode,slip_r,kappa_r,emergency,segment']
    for i in range(300):
        t = i * 0.01
        rows.append(f'{t:.2f},{t:.3f},0,{min(t, 2.0)},{min(t, 1.9)},{min(t, 1.95)},'
                    f'{390 if i < 200 else 362},active,0,0,{int(150 <= i < 180)},0:accel')
    (d / 'trace.csv').write_text('\n'.join(rows) + '\n')
    (d / 'marks.json').write_text(json.dumps({'valid': True, 'passed': 15, 'total': 16,
                                              'violations': list(violations)}))


def test_writes_curves_png_into_the_run(tmp_path):
    run = tmp_path / 'B1_lab_2_1' / 'nominal'
    write_run(run, [{'name': 'overshoot', 't': 1.0, 't0': 0.5, 't1': 1.5}])
    assert P.main([str(run)]) == 0
    png = run / 'curves.png'
    assert png.read_bytes()[:8] == b'\x89PNG\r\n\x1a\n'


def test_a_single_variant_below_the_name_is_found(tmp_path):
    write_run(tmp_path / 'x' / 'low_mu')
    assert P.find_run(tmp_path / 'x') == tmp_path / 'x' / 'low_mu'
    out = tmp_path / 'zoom.png'
    assert P.main([str(tmp_path / 'x'), '-o', str(out), '--t0', '1', '--t1', '2']) == 0
    assert out.is_file()


def test_no_trace_is_a_clear_error(tmp_path):
    with pytest.raises(SystemExit, match='no trace.csv'):
        P.find_run(tmp_path)


def test_a_run_directory_draws_every_variant(tmp_path):
    run = tmp_path / 'B1_lab_2_1' / '2026-10-19_14-03-07'
    for v in ('nominal', 'low_batt', 'low_mu'):
        write_run(run / v)
    assert P.find_runs(run) == [run / v for v in ('low_batt', 'low_mu', 'nominal')]
    assert P.main([str(run)]) == 0
    for v in ('nominal', 'low_batt', 'low_mu'):
        assert (run / v / 'curves.png').is_file()
    with pytest.raises(SystemExit, match='several'):
        P.main([str(run), '-o', str(tmp_path / 'one.png')])     # one PNG, three runs


def test_no_argument_is_the_latest_run(tmp_path):
    import os
    import time
    old = tmp_path / 'A1_lab_2_1' / '2026-10-18_09-00-00'
    new = tmp_path / 'B1_lab_2_1' / '2026-10-19_14-03-07'
    write_run(old / 'nominal')
    write_run(new / 'nominal')
    os.symlink(old.name, old.parent / 'latest')
    time.sleep(0.02)
    os.symlink(new.name, new.parent / 'latest')
    assert P.latest_run(tmp_path) == new.parent / 'latest'
    with pytest.raises(SystemExit, match='just lab2 sim'):
        P.latest_run(tmp_path / 'empty')


def test_title_names_the_submission_through_latest(tmp_path):
    import os
    run = tmp_path / 'B1_lab_2_1' / '2026-10-19_14-03-07'
    write_run(run / 'low_mu')
    os.symlink(run.name, run.parent / 'latest')
    assert P.title_for(run.parent / 'latest' / 'low_mu') == 'B1_lab_2_1 / 2026-10-19_14-03-07 / low_mu'
    assert P.title_for(tmp_path / 'o' / 'nominal') == 'o / nominal'
