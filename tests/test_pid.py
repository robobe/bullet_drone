import pytest

from bullet_drone.commands import InvalidCommandError
from bullet_drone.config import ConfigurationError
from bullet_drone.controllers import PID, PidSettings


def test_proportional_integral_derivative_and_reset():
    pid = PID(PidSettings(2, 3, 4, 100, 20))
    assert pid.update(2, 1, 0.5) == pytest.approx(3.5)  # P=2 I=1.5 D=0
    assert pid.update(2, 1.25, 0.25) == pytest.approx(-0.4375)  # P=1.5 I=2.0625 D=-4
    pid.reset()
    assert pid.integral == 0
    assert pid.update(2, 1.25, 0.25) == pytest.approx(2.0625)


def test_derivative_on_measurement_has_no_setpoint_kick():
    pid = PID(PidSettings(0, 0, 2, 100, 0))
    assert pid.update(0, 2, 0.1) == 0
    assert pid.update(10, 2, 0.1) == 0
    assert pid.update(10, 3, 0.5) == -4


def test_integral_is_bounded_and_rejects_windup_but_allows_unwinding():
    pid = PID(PidSettings(0, 1, 0, 2, 1))
    for _ in range(100):
        pid.update(1, 0, 0.1)
    assert pid.integral == 1
    assert pid.update(-1, 0, 0.1) == pytest.approx(0.9)
    pid = PID(PidSettings(1, 1, 0, 2, 2))
    for _ in range(100):
        assert pid.update(5, 0, 0.1) == 2
    assert pid.integral == 0
    assert pid.update(0.5, 0, 1) == 1
    assert pid.update(-0.1, 0, 1) == pytest.approx(0.3)


def test_feedforward_participates_in_saturation():
    pid = PID(PidSettings(0, 1, 0, 1, 1))
    assert pid.update(1, 0, 1, feedforward=2) == 1
    assert pid.integral == 0


@pytest.mark.parametrize('kwargs', [dict(kp=-1), dict(ki=True), dict(kd=float('nan')),
                                    dict(output_limit=0), dict(integral_limit=-1), dict(integral_limit=11)])
def test_settings_reject_invalid_values(kwargs):
    values = dict(kp=1, ki=1, kd=1, output_limit=10, integral_limit=2)
    values.update(kwargs)
    with pytest.raises(ConfigurationError):
        PidSettings(**values)


@pytest.mark.parametrize('args', [(1, 0, 0), (1, 0, -1), (True, 0, 1), (1, float('inf'), 1),
                                  (1, 0, float('nan')), (1, 0, 1, float('inf'))])
def test_rejected_update_keeps_history(args):
    pid = PID(PidSettings(1, 1, 1, 10, 2))
    pid.update(1, 0, 0.1)
    previous = (pid.integral, pid._previous_measurement)
    with pytest.raises(InvalidCommandError):
        pid.update(*args)
    assert (pid.integral, pid._previous_measurement) == previous
