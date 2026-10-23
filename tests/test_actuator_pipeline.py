from unittest.mock import Mock

import pybullet as p
import pytest

from bullet_drone.actuators import IdealMotorDynamics
from bullet_drone.commands import InvalidCommandError, RotorCommand
from bullet_drone.contracts import BackendError, LifecycleError
from bullet_drone.models import PropellerOutput
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.propellers import IdealPropellerModel
from bullet_drone.vehicle import load_racer_parameters


def test_ideal_motor_passthrough_and_reset():
    motors = IdealMotorDynamics(100)
    assert motors.update((10, 20, 30, 40), 0.1) == (10, 20, 30, 40)
    assert motors.reset() == (0, 0, 0, 0)
    for targets, dt in [((101, 0, 0, 0), 1), ((-1, 0, 0, 0), 1), ((0,)*3, 1), ((True,)*4, 1), ((0,)*4, 0)]:
        with pytest.raises(InvalidCommandError):
            motors.update(targets, dt)


def test_propeller_equations_and_signs_are_independent_of_pybullet():
    vehicle = load_racer_parameters()
    speeds = (10, 20, 30, 40)
    output = IdealPropellerModel(vehicle).calculate(speeds)
    assert [f[2] for f in output.forces_n] == pytest.approx([vehicle.thrust_coefficient * v*v for v in speeds])
    assert all(f[:2] == (0, 0) for f in output.forces_n)
    assert output.reaction_torque_nm == pytest.approx((0, 0, vehicle.torque_coefficient * (100-400+900-1600)))


def test_injected_motor_actual_speeds_drive_propellers_and_state(config):
    vehicle = load_racer_parameters()
    motors = Mock()
    motors.reset.return_value = (0,)*4
    motors.update.return_value = (500,)*4
    propellers = Mock(wraps=IdealPropellerModel(vehicle))
    physics = PyBulletPhysics(config, vehicle, motor_dynamics=motors, propeller_model=propellers)
    try:
        state = physics.reset()
        output = physics.step(RotorCommand((1000,)*4, state.episode_id, 1))
        motors.update.assert_called_once_with((1000,)*4, config.simulation.timestep_s)
        propellers.calculate.assert_called_once_with((500,)*4)
        assert output.rotor_speeds_rad_s == (500,)*4
        acceleration = 4 * vehicle.thrust_coefficient * 500**2 / config.drone.mass_kg - 9.81
        assert output.linear_velocity_m_s[2] == pytest.approx(acceleration * config.simulation.timestep_s)
        assert physics.reset().rotor_speeds_rad_s == (0,)*4
        assert motors.reset.call_count == 2
    finally:
        physics.close()


@pytest.mark.parametrize('component', ['motor', 'propeller'])
def test_model_failure_aborts_physics(config, component):
    motors = Mock()
    motors.reset.return_value = (0,)*4
    motors.update.return_value = (0,)*4
    propellers = Mock(wraps=IdealPropellerModel(load_racer_parameters()))
    (motors.update if component == 'motor' else propellers.calculate).side_effect = RuntimeError('model failed')
    physics = PyBulletPhysics(config, motor_dynamics=motors, propeller_model=propellers)
    try:
        physics.reset()
        with pytest.raises(BackendError, match='model failed'):
            physics.step()
        with pytest.raises(LifecycleError):
            physics.step()
    finally:
        physics.close()
    assert physics._client is None
    with pytest.raises(LifecycleError):
        physics.step()


def test_invalid_model_output_applies_no_force(config, monkeypatch):
    propellers = Mock()
    propellers.calculate.return_value = PropellerOutput(((0, 0, 1),)*3 + ((0, 0, float('nan')),), (0, 0, 0))
    physics = PyBulletPhysics(config, propeller_model=propellers)
    try:
        physics.reset()
        force = Mock()
        monkeypatch.setattr(p, 'applyExternalForce', force)
        with pytest.raises(BackendError):
            physics.step()
        force.assert_not_called()
        assert physics._step_index == 0
    finally:
        physics.close()


def test_motor_reset_requires_zero_and_cleans_up(config):
    motors = Mock()
    motors.reset.return_value = (1,)*4
    physics = PyBulletPhysics(config, motor_dynamics=motors)
    with pytest.raises(BackendError, match='stopped'):
        physics.reset()
    assert physics._client is None
