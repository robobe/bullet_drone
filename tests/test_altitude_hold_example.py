import subprocess
import sys
from unittest.mock import Mock

import pytest

from bullet_drone.controllers import AltitudeController, PidSettings
from bullet_drone.models import DroneObservation, DroneState
from bullet_drone.rc import RcCodec
from examples.altitude_hold import demo
from examples.altitude_hold.demo_cli import CliParser, DemoOptions
from examples.altitude_hold.mission import AltitudeHoldMission, Phase


def observation(t=0.0, z=0.015, speed=0.0):
    return DroneObservation(DroneState('episode', round(t * 240), t, (0, 0, z),
                                      (0, 0, 0, 1), (0, 0, speed), (0, 0, 0)))


def test_mission_dwell_hold_timing_and_angle_mode():
    codec = RcCodec(30)
    altitude = AltitudeController(1, 9.81, 30, PidSettings(1.5, 0, 0, 1, 0),
                                  PidSettings(4, 1, 0, 3, 1))
    mission = AltitudeHoldMission(altitude, codec)
    mission.reset(observation())

    def command(t, z=3, speed=0):
        obs = observation(t, z, speed)
        result = codec.decode(mission.command(obs, 1 / 240), 'episode', obs.ground_truth.step_index + 1)
        assert result.mode == 'angle' and result.rpy == (0, 0, 0)
        assert result.yaw_rate_rad_s == 0
        return result

    assert not command(0, 0.015).armed
    assert command(0.5, 0.015).armed
    command(1)
    command(1.5, speed=0.2)  # Interrupt takeoff dwell.
    command(2)
    command(2.9)
    assert mission.phase == Phase.TAKEOFF
    command(3)
    assert mission.phase == Phase.HOLD
    command(12.9)
    assert mission.phase == Phase.HOLD
    command(13)
    assert mission.phase == Phase.LAND
    assert altitude.last_output.velocity_target_m_s == -0.5
    command(14, 0.015)
    command(14.4, 0.015, -0.1)  # Interrupt touchdown dwell.
    command(15, 0.015)
    assert command(15.4, 0.015).armed
    off = command(15.5, 0.015)
    assert mission.phase == Phase.DISARMED and not off.armed and off.thrust_n == 0
    command(16.4, 0.015)
    assert mission.phase == Phase.DISARMED
    command(16.5, 0.015)
    assert mission.phase == Phase.COMPLETE
    assert [event.phase for event in mission.transitions] == list(Phase)
    mission.reset(observation())
    assert mission.phase == Phase.SETTLE and len(mission.transitions) == 1
    assert altitude.last_output is None


def record_simulations(monkeypatch, samples):
    instances = []
    original = demo.RcSimulation

    class RecordingSimulation(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            instances.append(self)

        def step(self, command):
            obs = super().step(command)
            assert self.last_command.mode == 'angle'
            samples.append((obs.ground_truth, self.last_command))
            return obs

    monkeypatch.setattr(demo, 'RcSimulation', RecordingSimulation)
    return instances


def assert_closed(simulation):
    assert simulation._environment._physics._client is None
    assert simulation._rc._closed


def test_complete_headless_flight(monkeypatch):
    samples = []
    simulations = record_simulations(monkeypatch, samples)
    result = demo.run_demo(gui=False)
    transitions = {event.phase: event.simulation_time_s for event in result.transitions}
    assert list(transitions) == list(Phase)
    assert 10 <= transitions[Phase.LAND] - transitions[Phase.HOLD] <= 10 + 1 / 240
    hovering = [s for s, _ in samples if transitions[Phase.HOLD] <= s.simulation_time_s < transitions[Phase.LAND]]
    assert hovering and max(abs(s.position_m[2] - 3) for s in hovering) <= 0.05
    assert max(abs(s.linear_velocity_m_s[2]) for s in hovering) <= 0.1
    off = [(s, c) for s, c in samples if s.simulation_time_s > transitions[Phase.DISARMED]]
    assert off and all(not c.armed and s.rotor_speeds_rad_s == (0, 0, 0, 0) for s, c in off)
    final = result.observation.ground_truth
    assert final.position_m[2] <= 0.02 and abs(final.linear_velocity_m_s[2]) <= 0.05
    assert final.simulation_time_s - transitions[Phase.DISARMED] >= 1
    assert final.simulation_time_s < 45
    assert_closed(simulations[0])


def test_timeout_closes_resources(monkeypatch):
    simulations = record_simulations(monkeypatch, [])
    with pytest.raises(demo.DemoTimeoutError, match='settle'):
        demo.run_demo(gui=False, max_duration_s=0.1)
    assert_closed(simulations[0])


def test_completion_at_exact_deadline():
    reference = demo.run_demo(gui=False)
    deadline = reference.observation.ground_truth.simulation_time_s
    completed = demo.run_demo(gui=False, max_duration_s=deadline)
    assert completed.transitions[-1].phase == Phase.COMPLETE
    assert completed.observation.ground_truth.simulation_time_s == deadline
    with pytest.raises(demo.DemoTimeoutError, match='disarmed'):
        demo.run_demo(gui=False, max_duration_s=deadline - 1 / 240)


@pytest.mark.parametrize('stage', ['reset', 'step', 'pid'])
@pytest.mark.parametrize('failure', [RuntimeError('injected failure'), KeyboardInterrupt()])
def test_failures_close_resources(monkeypatch, stage, failure):
    simulations = record_simulations(monkeypatch, [])
    if stage == 'pid':
        monkeypatch.setattr(demo.AltitudeController, 'thrust', Mock(side_effect=failure))
    else:
        monkeypatch.setattr(demo.RcSimulation, stage, Mock(side_effect=failure))
    with pytest.raises(type(failure)):
        demo.run_demo(gui=False)
    assert_closed(simulations[0])


def test_parser():
    assert CliParser().parse([]) == DemoOptions(False)
    assert CliParser().parse(['--headless']) == DemoOptions(True)


@pytest.mark.parametrize('args,status', [(['--help'], 0), (['--unknown'], 2)])
def test_cli_does_not_start_physics(args, status):
    result = subprocess.run([sys.executable, '-m', 'examples.altitude_hold.main', *args],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == status, result.stderr
    assert 'pybullet build time' not in result.stderr
    assert 'Altitude hold:' not in result.stderr


def test_import_has_no_startup_or_logging_side_effects():
    code = (
        "from loguru import logger; import sys; "
        "logger.add = logger.remove = lambda *a, **k: (_ for _ in ()).throw(AssertionError('logging')); "
        "import examples.altitude_hold.main; assert 'pybullet' not in sys.modules"
    )
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert result.stdout == result.stderr == ''
