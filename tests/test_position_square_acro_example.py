from math import dist, hypot
import subprocess
import sys
from unittest.mock import Mock

import pytest

from bullet_drone.config import ConfigurationError
from bullet_drone.controllers import IdealStateFeedback
from bullet_drone.geometry import attitude_error, from_rpy, yaw_from_quaternion
from bullet_drone.controllers.pid import clamp
from examples.position_square.mission import Phase, SquareMission
from examples.position_square_acro import demo


def test_full_acro_square_converts_upstream_and_bypasses_internal_attitude(monkeypatch):
    missions, simulations, samples = [], [], []
    make_mission = demo.SquareMission
    original_simulation = demo.RcSimulation

    def mission_factory(*args, **kwargs):
        mission = make_mission(*args, **kwargs)
        mission.attitude_controller.rates = Mock(wraps=mission.attitude_controller.rates)
        missions.append(mission)
        return mission

    class CheckedSimulation(original_simulation):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            simulations.append(self)
            self.control = self._provider._controller
            self.control.attitude.rates = Mock(side_effect=AssertionError('Acro must bypass internal attitude P'))
            self.control.rate.torque = Mock(wraps=self.control.rate.torque)

        def step(self, rc):
            before = self._observation
            mission = missions[0]
            result = super().step(rc)
            state, command = result.ground_truth, self.last_command
            assert command.mode == 'acro' and command.yaw_rate_rad_s is None
            assert command == mission.codec.decode(rc, state.episode_id, state.step_index)
            assert state.step_index == before.ground_truth.step_index + 1
            if command.armed:
                output = mission.velocity.last_output
                angles, feedback = mission.attitude_controller.rates.call_args.args
                measured_yaw = yaw_from_quaternion(before.ground_truth.orientation_xyzw)
                assert angles == (output.roll_rad, output.pitch_rad, measured_yaw)
                assert feedback == IdealStateFeedback().read(before)
                error = attitude_error(feedback.orientation_xyzw, from_rpy(angles))
                expected_rp = tuple(clamp(4 * e, 2) for e in error[:2])
                assert max(abs(a-b) for a, b in zip(command.rpy[:2], expected_rp)) <= .002 + 1e-12
                assert abs(command.thrust_n-output.thrust_n) <= mission.velocity.max_thrust_n / 2000 + 1e-12
                rates, measured_rates, dt = self.control.rate.torque.call_args.args
                assert rates == command.rpy and measured_rates == feedback.body_rates_rad_s
                assert dt == self.timestep_s
                assert all(abs(rate) <= 2 for rate in rates)
                allocation = self.allocation
                assert allocation.requested.thrust_n == command.thrust_n
                assert allocation.requested.torque_nm == pytest.approx(
                    tuple(clamp(.05 * (target-measured), .1) for target, measured in zip(rates, measured_rates)))
                assert state.rotor_speeds_rad_s == allocation.speeds_rad_s
                assert any(speed > 0 for speed in state.rotor_speeds_rad_s)
            else:
                assert self.allocation is None and state.rotor_speeds_rad_s == (0,0,0,0)
            samples.append(state)
            return result

    monkeypatch.setattr(demo, 'SquareMission', mission_factory)
    monkeypatch.setattr(demo, 'RcSimulation', CheckedSimulation)
    result = demo.run_demo(gui=False)
    simulations[0].control.attitude.rates.assert_not_called()
    assert missions[0].attitude_controller.rates.call_count == simulations[0].control.rate.torque.call_count > 0
    assert [event.phase for event in result.transitions] == list(Phase)
    assert [arrival.index for arrival in result.arrivals] == list(range(5))
    assert [arrival.target_position_m for arrival in result.arrivals] == [(0,0,3),(3,0,3),(3,3,3),(0,3,3),(0,0,3)]
    for arrival in result.arrivals:
        assert dist(arrival.state.position_m, arrival.target_position_m) <= .1
        assert hypot(*arrival.state.linear_velocity_m_s) <= .1
        assert abs(yaw_from_quaternion(arrival.state.orientation_xyzw)) < .05
    times = {event.phase: event.simulation_time_s for event in result.transitions}
    assert 10 <= times[Phase.LAND]-times[Phase.HOLD] <= 10 + 1/240
    hover = [s for s in samples if times[Phase.HOLD] <= s.simulation_time_s < times[Phase.LAND]]
    assert hover and max(dist(s.position_m, (0,0,3)) for s in hover) < .05
    final = result.observation.ground_truth
    assert hypot(*final.position_m[:2]) < .1 and final.position_m[2] < .02
    assert final.rotor_speeds_rad_s == (0,0,0,0)
    assert final.simulation_time_s-times[Phase.DISARMED] >= 1
    assert_closed(simulations[0])


def assert_closed(simulation):
    assert simulation._environment._physics._client is None and simulation._rc._closed


def test_acro_deadline_boundary():
    reference = demo.run_demo(gui=False)
    deadline = reference.observation.ground_truth.simulation_time_s
    assert demo.run_demo(gui=False, max_duration_s=deadline).transitions[-1].phase == Phase.COMPLETE
    with pytest.raises(demo.DemoTimeoutError, match='disarmed'):
        demo.run_demo(gui=False, max_duration_s=deadline-1/240)


@pytest.mark.parametrize('stage', ['reset','step','conversion'])
@pytest.mark.parametrize('failure', [RuntimeError('injected'), KeyboardInterrupt()])
def test_acro_failure_cleanup(monkeypatch, stage, failure):
    simulations = []
    original = demo.RcSimulation

    class Simulation(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            simulations.append(self)

    monkeypatch.setattr(demo, 'RcSimulation', Simulation)
    if stage == 'conversion':
        monkeypatch.setattr(demo.AttitudeController, 'rates', Mock(side_effect=failure))
    else:
        monkeypatch.setattr(Simulation, stage, Mock(side_effect=failure))
    with pytest.raises(type(failure)):
        demo.run_demo(gui=False)
    assert_closed(simulations[0])


def test_acro_timeout_cleanup(monkeypatch):
    simulation = demo.RcSimulation
    instances = []
    def create(*args, **kwargs):
        instance = simulation(*args, **kwargs)
        instances.append(instance)
        return instance
    monkeypatch.setattr(demo, 'RcSimulation', create)
    with pytest.raises(demo.DemoTimeoutError, match='settle'):
        demo.run_demo(gui=False, max_duration_s=.1)
    assert_closed(instances[0])


def test_acro_requires_conversion_and_rejects_invalid_mode():
    with pytest.raises(ConfigurationError, match='upstream attitude'):
        SquareMission(Mock(),Mock(),Mock(),Mock(),flight_mode='acro')
    with pytest.raises(ConfigurationError, match='flight_mode'):
        SquareMission(Mock(),Mock(),Mock(),Mock(),flight_mode='unknown')


@pytest.mark.parametrize('args,status', [(['--help'],0), (['--unknown'],2)])
def test_acro_cli_help_and_usage(args, status):
    result = subprocess.run([sys.executable, '-m', 'examples.position_square_acro.main', *args],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == status, result.stderr
    assert 'pybullet build time' not in result.stderr


def test_acro_import_safe():
    code = ("from loguru import logger; import sys; "
            "logger.add = logger.remove = lambda *a,**kw: (_ for _ in ()).throw(AssertionError('logging')); "
            "import examples.position_square_acro.main; assert 'pybullet' not in sys.modules")
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert result.stdout == result.stderr == ''
