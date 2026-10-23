from importlib import import_module
from types import SimpleNamespace

from loguru import logger
import pytest

from bullet_drone.commands import RcInput
from examples.timing import RealtimeFactorLogger, REPORT_INTERVAL_S


CLOCK_ORIGIN_S = 100.0
SIMULATION_ORIGIN_S = 10.0
SIMULATED_INTERVAL_S = 2.0
FAST_STEP_COST_S = 0.001
MISSION_STEPS = 2
EXAMPLE_NAMES = ('altitude_hold', 'position_square', 'position_square_acro')


@pytest.fixture
def messages():
    captured = []
    sink = logger.add(lambda message: captured.append(message.record['message']))
    try:
        yield captured
    finally:
        logger.remove(sink)


@pytest.mark.parametrize('factor', [1.0, 2.0, 0.5])
def test_realtime_factor_uses_elapsed_simulation_and_wall_time(factor, messages):
    wall_interval_s = SIMULATED_INTERVAL_S / factor
    timing = RealtimeFactorLogger(SIMULATION_ORIGIN_S, CLOCK_ORIGIN_S,
                                  clock=lambda: CLOCK_ORIGIN_S + wall_interval_s)
    timing.update(SIMULATION_ORIGIN_S + SIMULATED_INTERVAL_S)
    assert messages == [f'Timing: sim=2.000s wall={wall_interval_s:.3f}s RTF={factor:.3f}x']


def test_reporting_interval_and_final_partial_interval(messages):
    now = CLOCK_ORIGIN_S
    timing = RealtimeFactorLogger(SIMULATION_ORIGIN_S, now, clock=lambda: now)
    timing.update(SIMULATION_ORIGIN_S)
    assert not messages
    now += REPORT_INTERVAL_S / 2
    timing.update(SIMULATION_ORIGIN_S + REPORT_INTERVAL_S / 2)
    assert not messages
    now += REPORT_INTERVAL_S / 2
    timing.update(SIMULATION_ORIGIN_S + REPORT_INTERVAL_S)
    assert len(messages) == 1
    now += REPORT_INTERVAL_S / 2
    timing.update(SIMULATION_ORIGIN_S + REPORT_INTERVAL_S * 1.5)
    assert len(messages) == 1
    timing.update(SIMULATION_ORIGIN_S + REPORT_INTERVAL_S * 1.5, final=True)
    assert messages[-1] == 'Final Timing: sim=1.500s wall=1.500s RTF=1.000x'


def test_zero_wall_elapsed_is_unavailable_in_final_summary(messages):
    timing = RealtimeFactorLogger(SIMULATION_ORIGIN_S, CLOCK_ORIGIN_S, clock=lambda: CLOCK_ORIGIN_S)
    timing.update(SIMULATION_ORIGIN_S, final=True)
    assert messages == ['Final Timing: sim=0.000s wall=0.000s RTF=n/a']


@pytest.mark.parametrize('example_name', EXAMPLE_NAMES)
@pytest.mark.parametrize('gui', [False, True])
def test_example_timing_includes_pacing_and_excludes_setup_cleanup(example_name, gui, monkeypatch, messages):
    demo = import_module(f'examples.{example_name}.demo')
    now = CLOCK_ORIGIN_S
    slept = []
    def sleep(duration_s):
        nonlocal now
        slept.append(duration_s)
        now += duration_s
    class Simulation:
        def __init__(self, config, *args):
            self.timestep_s = config.simulation.timestep_s
            self.step_index = 0
        def observation(self):
            return SimpleNamespace(ground_truth=SimpleNamespace(
                step_index=self.step_index, simulation_time_s=self.step_index * self.timestep_s))
        def reset(self):
            nonlocal now
            now += CLOCK_ORIGIN_S
            return self.observation()
        def step(self, command):
            nonlocal now
            now += FAST_STEP_COST_S
            self.step_index += 1
            return self.observation()
        def close(self):
            nonlocal now
            now += CLOCK_ORIGIN_S
    class Mission:
        phase = demo.Phase.SETTLE
        arrivals = ()
        transitions = ()
        def __init__(self, *args, **kwargs):
            pass
        def reset(self, observation):
            nonlocal now
            now += CLOCK_ORIGIN_S
        def command(self, observation, dt_s):
            if observation.ground_truth.step_index == MISSION_STEPS:
                self.phase = demo.Phase.COMPLETE
            return RcInput()
    monkeypatch.setattr(demo, 'RcSimulation', Simulation)
    monkeypatch.setattr(demo, 'AltitudeHoldMission' if example_name == 'altitude_hold' else 'SquareMission', Mission)
    monkeypatch.setattr(demo.time, 'monotonic', lambda: now)
    monkeypatch.setattr(demo.time, 'sleep', sleep)
    if gui:
        gui_module = import_module(f'examples.{example_name}.gui')
        monkeypatch.setattr(gui_module, 'frame_flight', lambda: None)
    result = demo.run_demo(gui=gui)
    simulated_s = MISSION_STEPS * demo.TIMESTEP_S
    wall_s = simulated_s if gui else MISSION_STEPS * FAST_STEP_COST_S
    assert result.observation.ground_truth.step_index == MISSION_STEPS
    assert len(slept) == (MISSION_STEPS if gui else 0)
    final = [message for message in messages if message.startswith('Final Timing:')]
    assert final == [f'Final Timing: sim={simulated_s:.3f}s wall={wall_s:.3f}s RTF={simulated_s / wall_s:.3f}x']
