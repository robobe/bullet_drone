from dataclasses import replace
from math import pi
import subprocess
import sys

import pytest

from bullet_drone.commands import HighLevelCommand, InvalidCommandError
from bullet_drone.controllers.internal import AttitudeController, IdealStateFeedback, InternalFlightController
from bullet_drone.controllers import InternalControlSettings, PidSettings
from bullet_drone.contracts import LifecycleError
from bullet_drone.environment import DroneEnvironment
from bullet_drone.geometry import (GeometryError, attitude_error, conjugate, from_rpy,
                                   multiply, normalize, world_to_body, yaw_from_quaternion)
from bullet_drone.models import ControlFeedback, DroneObservation, DroneState
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.sensors import ImuSensor


IDENTITY = (0.0, 0.0, 0.0, 1.0)
ZERO_VECTOR = (0.0, 0.0, 0.0)
GRAVITY_M_S2 = (0.0, 0.0, -9.81)
QUARTER_TURN_RAD = pi / 2
HALF_TURN_QUATERNION = (1.0, 0.0, 0.0, 0.0)
TEST_VECTOR = (1.0, 2.0, 3.0)
TEST_RPY_RAD = (0.1, -0.2, 0.3)
DT_S = 1 / 240
ATTITUDE_GAINS = (4.0, 4.0, 4.0)
BODY_RATE_LIMITS_RAD_S = (2.0, 2.0, 2.0)
RATE_PID = PidSettings(0.05, 0.1, 0.01, 0.1, 0.1)
NUMERIC_TOLERANCE = 1e-12


def source_state():
    return DroneState('episode', 0, 0.0, ZERO_VECTOR, IDENTITY, ZERO_VECTOR, ZERO_VECTOR)


def test_quaternion_rotation_normalization_and_raw_product():
    q = from_rpy((0.0, 0.0, QUARTER_TURN_RAD))
    assert multiply(q, conjugate(q)) == pytest.approx(IDENTITY, abs=NUMERIC_TOLERANCE)
    assert normalize(tuple(3 * value for value in q)) == pytest.approx(q)
    assert world_to_body(TEST_VECTOR, q) == pytest.approx((2.0, -1.0, 3.0))
    assert world_to_body(TEST_VECTOR, tuple(-value for value in q)) == pytest.approx((2.0, -1.0, 3.0))
    assert yaw_from_quaternion(q) == pytest.approx(QUARTER_TURN_RAD)
    assert multiply(IDENTITY, (*TEST_VECTOR, 0.0)) == (*TEST_VECTOR, 0.0)


def test_rpy_identity_and_noncommuting_rotations():
    assert from_rpy(ZERO_VECTOR) == IDENTITY
    roll = from_rpy((QUARTER_TURN_RAD, 0.0, 0.0))
    yaw = from_rpy((0.0, 0.0, QUARTER_TURN_RAD))
    assert multiply(yaw, roll) != pytest.approx(multiply(roll, yaw))
    assert multiply(yaw, roll) == pytest.approx(from_rpy((QUARTER_TURN_RAD, 0.0, QUARTER_TURN_RAD)))


def test_shortest_rotation_and_sign_equivalence():
    current = from_rpy((0.0, 0.0, pi - 0.01))
    target = from_rpy((0.0, 0.0, -pi + 0.01))
    expected = (0.0, 0.0, 0.02)
    for first, second in ((current, target), (tuple(-v for v in current), target),
                          (current, tuple(-v for v in target))):
        assert attitude_error(first, second) == pytest.approx(expected, abs=NUMERIC_TOLERANCE)
    assert attitude_error(IDENTITY, IDENTITY) == ZERO_VECTOR
    assert attitude_error(IDENTITY, HALF_TURN_QUATERNION) == attitude_error(
        IDENTITY, tuple(-v for v in HALF_TURN_QUATERNION))


@pytest.mark.parametrize('bad', [None, (), (0.0, 0.0, 1.0), (0.0,) * 4,
                                (False, 0.0, 0.0, 1.0), (float('nan'), 0.0, 0.0, 1.0),
                                (float('inf'), 0.0, 0.0, 1.0), (10 ** 1000, 0, 0, 1)])
def test_invalid_orientation_is_a_neutral_geometry_error(bad):
    for operation in (normalize, yaw_from_quaternion, lambda q: world_to_body(TEST_VECTOR, q),
                      lambda q: attitude_error(IDENTITY, q)):
        with pytest.raises(GeometryError) as failure:
            operation(bad)
        assert not isinstance(failure.value, InvalidCommandError)


@pytest.mark.parametrize('bad', [None, (1.0, 2.0), (True, 0.0, 0.0),
                                (float('nan'), 0.0, 0.0), ('bad', 0.0, 0.0)])
def test_vector_diagnostics_do_not_assume_angular_velocity(bad):
    with pytest.raises(GeometryError, match='vector') as failure:
        world_to_body(bad, IDENTITY)
    assert 'angular_velocity' not in str(failure.value)
    with pytest.raises(GeometryError, match='rpy_rad'):
        from_rpy(bad)


def test_algebra_rejects_malformed_quaternions():
    with pytest.raises(GeometryError):
        conjugate((0.0, 1.0))
    with pytest.raises(GeometryError, match='left_quaternion'):
        multiply((float('nan'), 0.0, 0.0, 1.0), IDENTITY)
    with pytest.raises(GeometryError, match='right_quaternion'):
        multiply(IDENTITY, (True, 0.0, 0.0, 1.0))


def test_invalid_source_state_errors_stay_neutral():
    invalid = replace(source_state(), orientation_xyzw=(0.0,) * 4)
    for operation in (ImuSensor(GRAVITY_M_S2).reset,
                      lambda state: IdealStateFeedback().read(DroneObservation(state))):
        with pytest.raises(GeometryError) as failure:
            operation(invalid)
        assert not isinstance(failure.value, InvalidCommandError)


def test_controller_errors_preserve_command_semantics_and_pid_history():
    control = InternalFlightController(InternalControlSettings(
        ATTITUDE_GAINS, BODY_RATE_LIMITS_RAD_S, (RATE_PID,) * 3))
    reference = InternalFlightController(control.settings)
    feedback = ControlFeedback(IDENTITY, ZERO_VECTOR)
    command = HighLevelCommand('angle', TEST_RPY_RAD, 9.81, 'episode', 1)
    assert control.update(command, feedback, DT_S) == reference.update(command, feedback, DT_S)
    for mode in ('angle', 'acro'):
        with pytest.raises(InvalidCommandError) as failure:
            control.update(replace(command, mode=mode),
                           replace(feedback, orientation_xyzw=(0.0,) * 4), DT_S)
        assert isinstance(failure.value.__cause__, GeometryError)
    assert control.update(command, feedback, DT_S) == reference.update(command, feedback, DT_S)
    attitude = AttitudeController(ATTITUDE_GAINS, BODY_RATE_LIMITS_RAD_S)
    with pytest.raises(InvalidCommandError) as failure:
        attitude.rates(TEST_RPY_RAD, replace(feedback, orientation_xyzw=(0.0,) * 4))
    assert isinstance(failure.value.__cause__, GeometryError)


def test_source_geometry_failure_invalidates_environment(config, monkeypatch):
    physics = PyBulletPhysics(config)
    env = DroneEnvironment(physics, imu=ImuSensor(config.simulation.gravity_m_s2))
    real_step = physics.step
    try:
        env.reset()
        monkeypatch.setattr(physics, 'step', lambda: replace(real_step(), orientation_xyzw=(0.0,) * 4))
        with pytest.raises(GeometryError):
            env.step()
        with pytest.raises(LifecycleError):
            env.step()
        monkeypatch.setattr(physics, 'step', real_step)
        assert env.reset().sensors.imu is not None
        assert env.step().ground_truth.step_index == 1
    finally:
        env.close()


def test_importing_sensors_does_not_import_controllers():
    subprocess.run([sys.executable, '-c', '''
import sys
import bullet_drone.sensors
assert not any(name == "bullet_drone.controllers" or name.startswith("bullet_drone.controllers.")
               for name in sys.modules)
'''], check=True)


def test_attitude_error_is_in_current_body_frame():
    current = from_rpy((0.0, 0.0, QUARTER_TURN_RAD))
    target = from_rpy((TEST_RPY_RAD[0], 0.0, QUARTER_TURN_RAD))
    assert attitude_error(current, target) == pytest.approx(
        (TEST_RPY_RAD[0], 0.0, 0.0), abs=NUMERIC_TOLERANCE)
