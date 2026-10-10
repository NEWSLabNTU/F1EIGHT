"""The speed step schedule (speed_step_planner's script)."""
import pytest

from autosdv_coach_lab.calibration.speed_steps import StepSchedule


def test_targets_and_end_stopped():
    s = StepSchedule.build([0.5, 1.0, 1.5], 2.0)
    assert s.duration == 6.0
    assert s.target(-0.1) == (0.0, -1)
    assert s.target(0.0) == (0.5, 0)
    assert s.target(3.9) == (1.0, 1)
    assert s.target(5.99) == (1.5, 2)
    # Past the end the schedule always ends stopped, whatever its last step.
    assert s.target(6.0) == (0.0, 3)
    assert s.distance() == pytest.approx(6.0)


def test_per_step_holds():
    s = StepSchedule.build([0.0, 1.0], [1.0, 3.0])
    assert s.target(3.5) == (1.0, 1)
    assert StepSchedule.build([0.2, 0.4], [2.5]).holds == [2.5, 2.5]


@pytest.mark.parametrize('speeds,hold', [([], 1.0), ([-0.5], 1.0), ([0.5], 0.0),
                                         ([0.5, 1.0], [1.0, 2.0, 3.0])])
def test_rejects_bad_schedules(speeds, hold):
    with pytest.raises(ValueError):
        StepSchedule.build(speeds, hold)
