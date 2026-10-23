import pytest

from bullet_drone.commands import InvalidCommandError
from bullet_drone.config import ConfigurationError
from bullet_drone.controllers import AltitudeController, PidSettings


def controller(mass=1, capacity=30):
    return AltitudeController(mass, 9.81, capacity,
                              PidSettings(1.5, 0, 0, 1, 0), PidSettings(4, 1, 0, 3, 1))


def test_altitude_direction_gravity_compensation_and_mass_scaling():
    for mass in (0.5, 1, 2):
        c = controller(mass)
        assert c.thrust(3, 3, 0, 0.1) == pytest.approx(mass * 9.81)
        c.reset()
        assert c.thrust(3, 0, 0, 0.1) == pytest.approx(mass * 12.81)
        c.reset()
        assert c.thrust(0, 3, 0, 0.1) == pytest.approx(mass * 6.81)
        c.reset()
        assert c.thrust(3, 3, 0.5, 0.1) < mass * 9.81


def test_descent_limit_and_reset():
    c = controller()
    assert c.thrust(0, 3, 0, 0.1, velocity_limit_m_s=0.5) == pytest.approx(9.81 - 2 - 0.05)
    assert c.velocity.integral == pytest.approx(-0.05)
    c.reset()
    assert c.velocity.integral == 0
    assert c.thrust(3, 3, 0, 0.1) == pytest.approx(9.81)


def test_collective_clamps_to_capacity_and_nonnegative_thrust():
    c = controller(capacity=10)
    assert c.thrust(3, 0, 0, 0.1) == 10
    c = AltitudeController(1, 9.81, 30, PidSettings(1, 0, 0, 10, 0), PidSettings(5, 0, 0, 20, 0))
    assert c.thrust(0, 10, 0, 0.1) == 0


@pytest.mark.parametrize('kwargs', [dict(mass=0), dict(mass=True), dict(capacity=9.81), dict(capacity=float('nan'))])
def test_unusable_parameters_rejected(kwargs):
    with pytest.raises(ConfigurationError):
        controller(**kwargs)


@pytest.mark.parametrize('args', [(3, 3, 0, 0), (3, float('nan'), 0, 0.1), (3, 3, True, 0.1), (3, 3, 0, 0.1, -1)])
def test_invalid_input_preserves_pid_history(args):
    c = controller()
    c.thrust(3, 2.9, 0, 0.1)
    old = (c.position.integral, c.position._previous_measurement, c.velocity.integral, c.velocity._previous_measurement)
    with pytest.raises(InvalidCommandError):
        c.thrust(*args)
    assert (c.position.integral, c.position._previous_measurement, c.velocity.integral, c.velocity._previous_measurement) == old
