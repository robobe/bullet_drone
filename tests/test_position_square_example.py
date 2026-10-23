from math import isfinite, dist, hypot
import subprocess
import sys
from unittest.mock import Mock

import pytest

from bullet_drone.controllers import IdealStateFeedback
from bullet_drone.geometry import attitude_error, from_rpy, yaw_from_quaternion
from bullet_drone.controllers.pid import clamp
from bullet_drone.models import DroneObservation, DroneState
from examples.position_square import demo
from examples.position_square.demo_cli import CliParser, DemoOptions
from examples.position_square.mission import Phase, SquareMission


def observation(t=0, position=(0,0,.015), velocity=(0,0,0), yaw=0):
    return DroneObservation(DroneState('episode', round(t * 240), t, position, from_rpy((0,0,yaw)),
                                      velocity, (0,0,0)))


def test_mission_dwell_interruptions_heading_hold_and_landing_home():
    position, velocity, altitude, codec = Mock(), Mock(), Mock(), Mock()
    velocity.update.return_value = Mock(roll_rad=.1, pitch_rad=.2, thrust_n=10)
    codec.max_rates_rad_s = (2,2,2)
    m = SquareMission(position, velocity, altitude, codec)
    m.reset(observation(position=(1,2,.015), yaw=.4))
    assert m.waypoints == ((1,2,3), (4,2,3), (4,5,3), (1,5,3), (1,2,3))
    assert m.heading_rad == pytest.approx(.4)
    m.command(observation(), 1/240)
    codec.encode.assert_called_with(mode='angle', armed=False)
    m.command(observation(.5, (1,2,.015), yaw=.4), 1/240)
    assert m.phase == Phase.TAKEOFF
    t = 1.0
    for index, target in enumerate(m.waypoints):
        m.command(observation(t, target, yaw=.5), 1/240)
        m.command(observation(t+.5, target, velocity=(.2,0,0)), 1/240)
        m.command(observation(t+1, target, yaw=.5), 1/240)
        m.command(observation(t+1.9, target, yaw=.5), 1/240)
        assert len(m.arrivals) == index
        m.command(observation(t+2, target, yaw=.5), 1/240)
        assert len(m.arrivals) == index + 1
        t += 3
    assert m.phase == Phase.HOLD
    hold_started = m.transitions[-1].simulation_time_s
    m.command(observation(hold_started+9.9, (1,2,3), yaw=.5), 1/240)
    assert m.phase == Phase.HOLD
    assert codec.encode.call_args.kwargs['rpy'][2] == pytest.approx(-.4)
    m.command(observation(hold_started+10, (1,2,3)), 1/240)
    assert m.phase == Phase.LAND
    position.velocity.assert_called_with((1,2), (1,2), 1/240)
    assert altitude.thrust.call_args.args[0] == 0
    assert altitude.thrust.call_args.kwargs['velocity_limit_m_s'] == .5
    t = hold_started + 11
    m.command(observation(t, (4,5,.015)), 1/240)  # Wrong pad cannot finish landing.
    m.command(observation(t+1, (4,5,.015)), 1/240)
    assert m.phase == Phase.LAND
    m.command(observation(t+2, (1,2,.015)), 1/240)
    m.command(observation(t+2.4, (1,2,.015), (0,0,-.1)), 1/240)
    m.command(observation(t+3, (1,2,.015)), 1/240)
    m.command(observation(t+3.5, (1,2,.015)), 1/240)
    assert m.phase == Phase.DISARMED
    codec.encode.assert_called_with(mode='angle', armed=False)
    m.command(observation(t+4.5, (1,2,.015)), 1/240)
    assert m.phase == Phase.COMPLETE
    assert [e.phase for e in m.transitions] == list(Phase)
    m.reset(observation())
    assert not m.arrivals and m.index == 0 and m.phase == Phase.SETTLE


def assert_closed(simulation):
    assert simulation._environment._physics._client is None
    assert simulation._rc._closed


def test_complete_mission_checks_every_control_link(monkeypatch):
    original = demo.RcSimulation
    instances, samples = [], []
    missions = []
    original_mission = demo.SquareMission

    def make_mission(*args):
        mission = original_mission(*args)
        missions.append(mission)
        return mission

    class CheckedSimulation(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            instances.append(self)
            self.stabilization = self._provider._controller
            self.stabilization.attitude.rates = Mock(wraps=self.stabilization.attitude.rates)
            self.stabilization.rate.torque = Mock(wraps=self.stabilization.rate.torque)
            self._provider._mixer.mix = Mock(wraps=self._provider._mixer.mix)
            physics = self._environment._physics
            physics._apply_rotor_thrust = Mock(wraps=physics._apply_rotor_thrust)
            physics._apply_body_torque = Mock(wraps=physics._apply_body_torque)

        def step(self, rc):
            before = self._observation
            result = super().step(rc)
            state, command = result.ground_truth, self.last_command
            mission = missions[0]
            assert command == mission.codec.decode(rc, state.episode_id, state.step_index)
            assert state.step_index == before.ground_truth.step_index + 1
            assert command.mode == 'angle'
            if command.armed:
                output = mission.velocity.last_output
                desired = mission.position.last_output
                expected_acceleration = tuple(clamp(2 * (v - measured), 1.5)
                                              for v, measured in zip(desired, before.ground_truth.linear_velocity_m_s[:2]))
                assert output.acceleration_xy_m_s2 == pytest.approx(expected_acceleration)
                vertical = mission.altitude.last_output
                assert vertical.thrust_n == pytest.approx(max(0, min(mission.altitude.max_thrust_n,
                                                                    9.81 + vertical.acceleration_m_s2)))
                assert hypot(*mission.position.last_output) <= 1 + 1e-12
                assert all(abs(a) <= 1.5 for a in output.acceleration_xy_m_s2)
                max_thrust = mission.velocity.max_thrust_n
                assert abs(command.thrust_n - output.thrust_n) <= max_thrust / 2000 + 1e-12
                assert abs(command.rpy[0] - output.roll_rad) <= mission.codec.settings.max_angle_rad / 1000 + 1e-12
                assert abs(command.rpy[1] - output.pitch_rad) <= mission.codec.settings.max_angle_rad / 1000 + 1e-12
                rpy, feedback = self.stabilization.attitude.rates.call_args.args
                assert feedback == IdealStateFeedback().read(before)
                assert rpy == (*command.rpy[:2], yaw_from_quaternion(before.ground_truth.orientation_xyzw))
                rates, measured_rates, dt = self.stabilization.rate.torque.call_args.args
                assert measured_rates == feedback.body_rates_rad_s and dt == self.timestep_s
                error = attitude_error(feedback.orientation_xyzw, from_rpy(rpy))
                assert rates == pytest.approx((*tuple(clamp(4 * e, 2) for e in error[:2]), command.yaw_rate_rad_s))
                assert all(abs(rate) <= 2 for rate in rates)
                allocation = self.allocation
                assert allocation.requested.torque_nm == pytest.approx(tuple(clamp(.05 * (r - actual), .1)
                                                                              for r, actual in zip(rates, measured_rates)))
                assert self._provider._mixer.mix.call_args.args[0] == allocation.requested
                assert allocation.requested.thrust_n == command.thrust_n
                assert state.rotor_speeds_rad_s == allocation.speeds_rad_s
                physics = self._environment._physics
                output_forces = physics._apply_rotor_thrust.call_args.args[0]
                assert physics._apply_body_torque.call_args.args[0] == output_forces
                # The mixer predicts ideal allocation; downstream aerodynamics changes delivery.
                assert len(output_forces.rotor_forces_n) == 4
                assert all(isfinite(value) for force in (*output_forces.rotor_forces_n,
                           output_forces.body_force_n, output_forces.body_torque_nm) for value in force)
            else:
                assert self.allocation is None and state.rotor_speeds_rad_s == (0,0,0,0)
            samples.append((state, self.allocation))
            return result

    monkeypatch.setattr(demo, 'SquareMission', make_mission)
    monkeypatch.setattr(demo, 'RcSimulation', CheckedSimulation)
    result = demo.run_demo(gui=False)
    assert [event.phase for event in result.transitions] == list(Phase)
    assert [arrival.index for arrival in result.arrivals] == list(range(5))
    for arrival in result.arrivals:
        assert dist(arrival.state.position_m, arrival.target_position_m) <= .1
        assert hypot(*arrival.state.linear_velocity_m_s) <= .1
        assert abs(yaw_from_quaternion(arrival.state.orientation_xyzw)) < .05
    times = {event.phase: event.simulation_time_s for event in result.transitions}
    assert 10 <= times[Phase.LAND] - times[Phase.HOLD] <= 10 + 1/240
    hover = [s for s, _ in samples if times[Phase.HOLD] <= s.simulation_time_s < times[Phase.LAND]]
    assert hover and max(dist(s.position_m, (0,0,3)) for s in hover) < .05
    assert max(abs(s.position_m[2] - 3) for s, _ in samples
               if times[Phase.SQUARE] <= s.simulation_time_s < times[Phase.LAND]) < .05
    assert any(a and abs(a.requested.torque_nm[0]) > 1e-4 for _, a in samples)
    assert any(a and abs(a.requested.torque_nm[1]) > 1e-4 for _, a in samples)
    assert max(s.position_m[0] for s, _ in samples) >= 2.9
    assert max(s.position_m[1] for s, _ in samples) >= 2.9
    final = result.observation.ground_truth
    assert hypot(*final.position_m[:2]) < .1 and final.position_m[2] < .02
    assert final.rotor_speeds_rad_s == (0,0,0,0)
    assert final.simulation_time_s - times[Phase.DISARMED] >= 1
    assert final.simulation_time_s < 90
    assert_closed(instances[0])


def test_deadline_completion_and_timeout():
    reference = demo.run_demo(gui=False)
    deadline = reference.observation.ground_truth.simulation_time_s
    assert demo.run_demo(gui=False, max_duration_s=deadline).transitions[-1].phase == Phase.COMPLETE
    with pytest.raises(demo.DemoTimeoutError, match='disarmed'):
        demo.run_demo(gui=False, max_duration_s=deadline - 1/240)


@pytest.mark.parametrize('stage', ['reset', 'step', 'position', 'velocity', 'altitude', 'timeout'])
@pytest.mark.parametrize('failure', [RuntimeError('injected'), KeyboardInterrupt()])
def test_failure_and_timeout_cleanup(monkeypatch, stage, failure):
    instances = []
    original = demo.RcSimulation

    class Simulation(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            instances.append(self)

    monkeypatch.setattr(demo, 'RcSimulation', Simulation)
    if stage in ('reset', 'step'):
        monkeypatch.setattr(Simulation, stage, Mock(side_effect=failure))
    elif stage != 'timeout':
        cls, method = {'position': (demo.PositionController, 'velocity'),
                       'velocity': (demo.VelocityController, 'update'),
                       'altitude': (demo.AltitudeController, 'thrust')}[stage]
        monkeypatch.setattr(cls, method, Mock(side_effect=failure))
    with pytest.raises(demo.DemoTimeoutError if stage == 'timeout' else type(failure)):
        demo.run_demo(gui=False, max_duration_s=.1 if stage == 'timeout' else 90)
    assert_closed(instances[0])


def test_parser_and_import_safety():
    assert CliParser().parse([]) == DemoOptions(False)
    assert CliParser().parse(['--headless']) == DemoOptions(True)
    code = ("from loguru import logger; import sys; "
            "logger.add = logger.remove = lambda *a, **k: (_ for _ in ()).throw(AssertionError('logging')); "
            "import examples.position_square.main; assert 'pybullet' not in sys.modules")
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert result.stdout == result.stderr == ''


@pytest.mark.parametrize('args,status', [(['--help'], 0), (['--unknown'], 2)])
def test_cli_help_and_usage_do_not_start_physics(args, status):
    result = subprocess.run([sys.executable, '-m', 'examples.position_square.main', *args],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == status, result.stderr
    assert 'pybullet build time' not in result.stderr and 'Position square:' not in result.stderr
