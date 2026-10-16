"""fuzz_bags' verdict logic, without ROS: python3 -m pytest scripts/testing/lab2/test_fuzz_bags.py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fuzz_bags import Expect, catalogue, classify, combine, monotonic  # noqa: E402


def got(check_ok, grade_valid, audit=None, metric=None, errors=()):
    return {'check_ok': check_ok, 'grade_valid': grade_valid, 'audit': audit, 'metric': metric,
            'errors': list(errors)}


def test_caught_and_missed():
    e = Expect('REJECT', 'INVALID')
    assert classify(e, got(False, False, 'INVALID'))[0] == 'CAUGHT'
    assert classify(e, got(True, False, 'INVALID'))[0] == 'MISSED'
    assert classify(e, got(False, True, 'INVALID'))[0] == 'MISSED'
    assert classify(e, got(False, False, 'VALID'))[0] == 'MISSED'


def test_ungradable_counts_as_invalid():
    assert classify(Expect(None, 'INVALID'), got(True, None))[0] == 'CAUGHT'


def test_false_alarm_and_metric():
    e = Expect('ACCEPT', 'VALID')
    assert classify(e, got(True, True, 'VALID'))[0] == 'OK'
    assert classify(e, got(False, True, 'INVALID'))[0] == 'FALSE'
    assert classify(e, got(True, True, 'VALID', (False, 'flat')))[0] == 'METRIC'


def test_crash_is_reported():
    s, d = classify(Expect('REJECT'), got(False, None, errors=['check_bags raised TypeError']))
    assert s == 'CRASH' and 'check_bags raised TypeError' in d


def test_audit_follows_the_gates():
    assert Expect('REJECT').audit == 'INVALID'
    assert Expect(None, 'INVALID').audit == 'INVALID'
    assert Expect('ACCEPT', 'VALID').audit == 'VALID'
    assert Expect('ACCEPT').audit is None
    assert Expect('REJECT', audit_want='VALID').audit == 'VALID'


def test_combine_takes_the_worst():
    ok = Expect('ACCEPT', 'VALID')
    assert combine([ok, ok]).label() == 'ACCEPT/VALID'
    assert combine([ok, Expect('REJECT')]).label() == 'REJECT/-'
    assert combine([Expect(None, 'INVALID'), ok]).label() == '-/INVALID'
    assert combine([Expect('REJECT'), Expect(None, 'INVALID')]).label() == 'REJECT/INVALID'


def test_monotonic():
    assert monotonic([0.1, 0.2, 0.3])
    assert not monotonic([0.1, 0.1, 0.3])


def test_catalogue_names_are_unique():
    names = [c.name for c in catalogue(Path('/nonexistent'))]
    assert len(names) == len(set(names))
    assert any(n.startswith('drop:') for n in names)
