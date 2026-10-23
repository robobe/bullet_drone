from dataclasses import FrozenInstanceError
from math import acos, cos, hypot, pi, radians

import pytest

from bullet_drone.commands import InvalidCommandError
from bullet_drone.config import ConfigurationError
from bullet_drone.controllers import PidSettings, PositionController, VelocityController
from bullet_drone.geometry import conjugate, from_rpy, multiply


def position_controller():
    return PositionController(PidSettings(.8, .1, .05, 1, .2))


def velocity_controller(tilt=15, capacity=30):
    return VelocityController(1, capacity, PidSettings(2, .1, .05, 1.5, .2), radians(tilt))


def test_position_direction_vector_limit_and_reset():
    c = position_controller()
    assert c.velocity((1, -1), (0, 0), .01)[0] > 0
    c.reset()
    diagonal = c.velocity((100, 100), (0, 0), .01)
    assert hypot(*diagonal) == pytest.approx(1)
    assert diagonal[0] == diagonal[1]
    c.reset()
    assert c.last_output is None
    assert c.velocity((1, -1), (0, 0), .01) == position_controller().velocity((1, -1), (0, 0), .01)


@pytest.mark.parametrize('target,yaw,axis', [((1,0),0,0), ((0,1),0,1),
                                           ((1,0),pi/2,0), ((0,1),pi/2,1)])
def test_velocity_world_force_directions_and_yaw_independence(target, yaw, axis):
    c = velocity_controller()
    output = c.update(target, (0, 0), 9.81, yaw, .01)
    q = from_rpy((output.roll_rad, output.pitch_rad, yaw))
    force = multiply(multiply(q, (0,0,output.thrust_n,0)), conjugate(q))[:3]
    assert force[axis] > 0 and abs(force[1-axis]) < 1e-8
    assert force[2] == pytest.approx(9.81, abs=1e-10)


def test_velocity_tilt_collective_limits_zero_thrust_and_reset():
    c = velocity_controller(tilt=5)
    output = c.update((100,100), (-100,-100), 9.81, 0, .01)
    assert acos(cos(output.roll_rad) * cos(output.pitch_rad)) <= radians(5) + 1e-12
    assert 9.81 < output.thrust_n <= 30
    assert hypot(*output.acceleration_xy_m_s2) > 0
    with pytest.raises(FrozenInstanceError):
        output.thrust_n = 0
    c.reset()
    assert c.last_output is None
    level = c.update((0,0), (0,0), 9.81, 0, .01)
    assert (level.roll_rad, level.pitch_rad, level.thrust_n) == (0,0,9.81)
    capped = velocity_controller(capacity=10).update((100,100), (0,0), 20, 0, .01)
    assert capped.thrust_n == 10
    no_thrust = c.update((100,100), (0,0), 0, 0, .01)
    assert (no_thrust.roll_rad, no_thrust.pitch_rad, no_thrust.thrust_n) == (0,0,0)


@pytest.mark.parametrize('kwargs', [dict(target_position_xy_m=(float('nan'),0)),
                                    dict(position_xy_m=(0,float('inf'))),
                                    dict(target_position_xy_m=(1,)), dict(dt_s=0), dict(dt_s=True)])
def test_invalid_position_requests_preserve_history(kwargs):
    c, reference = position_controller(), position_controller()
    args = dict(target_position_xy_m=(1,1), position_xy_m=(0,0), dt_s=.01)
    previous = c.velocity(**args)
    reference.velocity(**args)
    with pytest.raises(InvalidCommandError):
        c.velocity(**(args | kwargs))
    assert c.last_output is previous
    next_args = args | dict(position_xy_m=(.1,.2))
    assert c.velocity(**next_args) == reference.velocity(**next_args)


@pytest.mark.parametrize('kwargs', [dict(target_velocity_xy_m_s=(float('nan'),0)),
                                    dict(velocity_xy_m_s=(0,float('inf'))),
                                    dict(velocity_xy_m_s=(0,)), dict(vertical_thrust_n=-1),
                                    dict(vertical_thrust_n=True), dict(measured_yaw_rad=float('nan')),
                                    dict(dt_s=0)])
def test_invalid_velocity_requests_preserve_history(kwargs):
    c, reference = velocity_controller(), velocity_controller()
    args = dict(target_velocity_xy_m_s=(1,1), velocity_xy_m_s=(0,0),
                vertical_thrust_n=9.81, measured_yaw_rad=0, dt_s=.01)
    previous = c.update(**args)
    reference.update(**args)
    with pytest.raises(InvalidCommandError):
        c.update(**(args | kwargs))
    assert c.last_output is previous
    next_args = args | dict(velocity_xy_m_s=(.1,.2))
    assert c.update(**next_args) == reference.update(**next_args)
    c.reset()
    reference.reset()
    assert c.update(**args) == reference.update(**args)


@pytest.mark.parametrize('mass,capacity,tilt', [(0,30,.2), (True,30,.2), (1,0,.2),
                                             (1,30,0), (1,30,pi/3), (1,30,float('nan'))])
def test_invalid_velocity_configuration(mass, capacity, tilt):
    with pytest.raises(ConfigurationError):
        VelocityController(mass, capacity, PidSettings(2,0,0,1.5,0), tilt)
