"""Independent mission, CLI, owned-process cleanup and opt-in live SITL flight."""
import ast
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import Mock, MagicMock

import pytest
from loguru import logger

from examples.altitde_hold_bt.demo_cli import CliParser, DemoOptions
from examples.altitde_hold_bt.mission import AltitudeHoldMission, Phase, throttle_pwm


EXAMPLE_PATH = Path(__file__).resolve().parents[1] / 'examples/altitde_hold_bt'
MAX_THRUST_N = 34.0
MASS_KG = 1.0
GRAVITY_M_S2 = 9.81
TIMESTEP_S = 1 / 240
TIME_TOLERANCE_S = 1e-9
ARMING_RXLOSS_MASK = 0x2000000c
ARM_SWITCH_LATCH_MASK = 0x20000008
PACE_TEST_DURATION_S = 0.05
INJECTED_STALL_S = 0.04
SIMULATED_MSP_DELAY_S = 0.015
STALL_AT_STEP = 3
STALL_INTERVAL_STEPS = 24
MIN_STALL_HEIGHT_M = 0.2


def state(time_s, altitude_m=0.015, speed_m_s=0.0):
    return SimpleNamespace(simulation_time_s=time_s, position_m=(0.0, 0.0, altitude_m),
                           linear_velocity_m_s=(0.0, 0.0, speed_m_s))


def test_mission_waits_for_real_arming_holds_lands_and_waits_for_disarm():
    mission = AltitudeHoldMission(MASS_KG, GRAVITY_M_S2, MAX_THRUST_N)
    status = SimpleNamespace(armed=False, arming_disabled_flags=1)
    command = mission.command(state(1), status, TIMESTEP_S)
    assert mission.phase == Phase.PREARM and command[4] == 1000
    status.arming_disabled_flags = 0
    assert mission.command(state(1.5), status, TIMESTEP_S)[4] == 1000
    command = mission.command(state(2), status, TIMESTEP_S)
    assert mission.phase == Phase.ARMING and command[4] == 2000 and command[2] == 1000
    mission.command(state(3), status, TIMESTEP_S)
    assert mission.phase == Phase.ARMING
    status.armed = True
    assert mission.command(state(4), status, TIMESTEP_S)[2] > 1050
    assert mission.phase == Phase.TAKEOFF
    mission.command(state(5, 3), status, TIMESTEP_S)
    mission.command(state(5.5, 3, .2), status, TIMESTEP_S)
    mission.command(state(6, 3), status, TIMESTEP_S)
    mission.command(state(7, 3), status, TIMESTEP_S)
    assert mission.phase == Phase.HOLD
    mission.command(state(16.9, 3), status, TIMESTEP_S)
    assert mission.phase == Phase.HOLD
    mission.command(state(17, 3), status, TIMESTEP_S)
    assert mission.phase == Phase.LAND
    assert mission.controller.last_output.velocity_target_m_s == -.5
    mission.command(state(18), status, TIMESTEP_S)
    mission.command(state(18.25, speed_m_s=-.1), status, TIMESTEP_S)
    mission.command(state(19), status, TIMESTEP_S)
    command = mission.command(state(19.5), status, TIMESTEP_S)
    assert mission.phase == Phase.DISARMED and command[2] == command[4] == 1000
    mission.command(state(20.5), status, TIMESTEP_S)
    assert mission.phase == Phase.DISARMED
    status.armed = False
    mission.command(state(20.6), status, TIMESTEP_S)
    assert mission.phase == Phase.COMPLETE
    assert [phase for phase, _ in mission.transitions] == list(Phase)


def test_unexpected_disarming_aborts_flight():
    mission = AltitudeHoldMission(MASS_KG, GRAVITY_M_S2, MAX_THRUST_N)
    mission.transition(Phase.TAKEOFF, 0.0)
    with pytest.raises(RuntimeError, match='disarmed during flight'):
        mission.command(state(1), SimpleNamespace(armed=False), TIMESTEP_S)


def test_rxloss_while_arming_lowers_switch_and_waits_for_stable_clear_status():
    mission = AltitudeHoldMission(MASS_KG, GRAVITY_M_S2, MAX_THRUST_N)
    status = SimpleNamespace(armed=False, arming_disabled_flags=0)
    mission.command(state(0), status, TIMESTEP_S)
    assert mission.command(state(.5), status, TIMESTEP_S)[4] == 2000
    mission.command(state(1.025), status, TIMESTEP_S)
    status.arming_disabled_flags = ARMING_RXLOSS_MASK
    channels = mission.command(state(1.137), status, TIMESTEP_S)
    assert mission.phase == Phase.PREARM
    assert channels[4] == channels[2] == 1000
    status.arming_disabled_flags = ARM_SWITCH_LATCH_MASK
    assert mission.command(state(1.496), status, TIMESTEP_S)[4] == 1000
    status.arming_disabled_flags = 0
    assert mission.command(state(1.6), status, TIMESTEP_S)[4] == 1000
    assert mission.command(state(1.8), status, TIMESTEP_S)[4] == 1000
    status.arming_disabled_flags = ARMING_RXLOSS_MASK
    assert mission.command(state(1.9), status, TIMESTEP_S)[4] == 1000
    status.arming_disabled_flags = 0
    assert mission.command(state(2), status, TIMESTEP_S)[4] == 1000
    assert mission.command(state(2.5), status, TIMESTEP_S)[4] == 2000
    status.armed = True
    mission.command(state(2.6), status, TIMESTEP_S)
    assert mission.phase == Phase.TAKEOFF


def test_prearm_keeps_switch_low_until_disarm_is_confirmed():
    mission = AltitudeHoldMission(MASS_KG, GRAVITY_M_S2, MAX_THRUST_N)
    status = SimpleNamespace(armed=True, arming_disabled_flags=0)
    assert mission.command(state(0), status, TIMESTEP_S)[4] == 1000
    assert mission.command(state(1), status, TIMESTEP_S)[4] == 1000
    status.armed = False
    assert mission.command(state(2), status, TIMESTEP_S)[4] == 1000
    assert mission.command(state(2.5), status, TIMESTEP_S)[4] == 2000


def test_pwm_mapping_matches_speed_squared_and_profile_endpoints():
    assert throttle_pwm(0, MAX_THRUST_N) == 1000
    assert throttle_pwm(MAX_THRUST_N, MAX_THRUST_N) == 2000
    assert throttle_pwm(MAX_THRUST_N * 2, MAX_THRUST_N) == 2000
    pwm = throttle_pwm(GRAVITY_M_S2, MAX_THRUST_N)
    motor_speed = .055 + .945 * (pwm - 1050) / 950
    assert MAX_THRUST_N * motor_speed**2 == pytest.approx(GRAVITY_M_S2, abs=.03)


def test_arming_blockers_decode_pinned_flags_and_preserve_unknown_bits():
    from examples.altitde_hold_bt.diagnostics import arming_blockers
    assert arming_blockers(0) == 'NONE'
    assert arming_blockers((1 << 9) | (1 << 12) | (1 << 29)) == 'BOOTGRACE,CALIB,ARM_SWITCH'
    assert arming_blockers(1 << 31) == 'UNKNOWN(0x80000000)'


def test_state_change_contains_terminal_color():
    from examples.altitde_hold_bt.diagnostics import log_transition
    output = io.StringIO()
    sink = logger.add(output, format='{message}', colorize=True)
    try:
        log_transition(Phase.PREARM, Phase.ARMING, 1.25)
    finally:
        logger.remove(sink)
    assert 'PREARM ->' in output.getvalue()
    assert '\x1b[35mARMING\x1b[0m' in output.getvalue()
    assert 'sim=1.250s' in output.getvalue()


def test_status_reports_arming_commands_blockers_and_missing_motors():
    from examples.altitde_hold_bt.diagnostics import log_status
    output = io.StringIO()
    sink = logger.add(output, format='{message}')
    try:
        status = SimpleNamespace(armed=False, arming_disabled_flags=1 << 12, received_at=time.monotonic())
        backend = SimpleNamespace(received_at=None, latest=(0.0,) * 4)
        client = SimpleNamespace(rc=SimpleNamespace(sent=5))
        log_status(Phase.PREARM, state(1), status, (1500, 1500, 1000, 1500, 1000, 2000),
                   backend, client, .9)
    finally:
        logger.remove(sink)
    message = output.getvalue()
    assert 'armed=False' in message and 'blockers=CALIB' in message
    assert '0x00001000' in message
    assert 'arm_pwm=1000' in message and 'throttle_pwm=1000' in message
    assert 'motor_age=NONE' in message and 'RC_sent=5' in message


def test_cli_parse_and_import_has_no_startup_effects():
    assert CliParser().parse([]) == DemoOptions()
    assert CliParser().parse(['--headless']).headless
    code = "from examples.altitde_hold_bt.main import main; import sys; assert 'pybullet' not in sys.modules; assert 'msp_link' not in sys.modules; raise SystemExit(main(['--help']))"
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert 'Take off to 3 m' in result.stdout


def test_no_code_imports_from_other_examples_or_apps():
    for path in EXAMPLE_PATH.glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            names = ([node.module] if isinstance(node, ast.ImportFrom) else
                     [alias.name for alias in node.names] if isinstance(node, ast.Import) else [])
            for name in names:
                assert not name.startswith(('bullet_app', 'tools.zmq_render', 'zmq'))
                assert not name.startswith('examples.') or name.startswith('examples.altitde_hold_bt.'), (path, name)


def test_missing_binary_fails_before_physics(tmp_path, monkeypatch):
    pytest.importorskip('msp_link')
    from examples.altitde_hold_bt import demo
    physics = Mock()
    monkeypatch.setattr(demo, 'PyBulletPhysics', physics)
    with pytest.raises(ValueError, match='setup_betaflight_sitl.sh'):
        demo.run_demo(gui=False, sitl_binary=tmp_path / 'missing')
    physics.assert_not_called()


def test_occupied_port_does_not_start_firmware(tmp_path, monkeypatch):
    pytest.importorskip('msp_link')
    from examples.altitde_hold_bt import sitl
    binary = tmp_path / 'firmware'
    binary.write_text('#!/bin/sh\nexit 0\n')
    binary.chmod(0o755)
    probe = MagicMock()
    probe.__enter__.return_value.bind.side_effect = OSError('in use')
    monkeypatch.setattr(sitl.socket, 'socket', Mock(return_value=probe))
    run = Mock()
    monkeypatch.setattr(sitl.subprocess, 'run', run)
    with pytest.raises(RuntimeError, match='occupied'):
        with sitl.SitlSession(binary):
            pass
    run.assert_not_called()


def test_session_stops_process_and_removes_workdir_on_startup_error(tmp_path, monkeypatch):
    pytest.importorskip('msp_link')
    from examples.altitde_hold_bt import sitl
    binary = tmp_path / 'firmware'
    binary.write_text('#!/bin/sh\nexit 0\n')
    binary.chmod(0o755)
    monkeypatch.setattr(sitl.socket, 'socket', MagicMock())
    monkeypatch.setattr(sitl.subprocess, 'run', Mock())
    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr(sitl.subprocess, 'Popen', Mock(return_value=process))
    client = Mock()
    client.connected.wait.return_value = False
    client.connected.is_set.return_value = False
    monkeypatch.setattr(sitl, 'Client', Mock(return_value=Mock(start=Mock(return_value=client))))
    session = sitl.SitlSession(binary)
    with pytest.raises(RuntimeError, match='connection lost'):
        session.__enter__()
    process.terminate.assert_called_once()
    client.close.assert_called_once()
    assert not Path(session.work.name).exists()


def test_firmware_log_tail_does_not_move_the_shared_write_offset(tmp_path):
    pytest.importorskip('msp_link')
    from examples.altitde_hold_bt.sitl import SitlSession
    binary = tmp_path / 'firmware'
    binary.write_text('#!/bin/sh\nexit 0\n')
    binary.chmod(0o755)
    session = SitlSession(binary)
    with tempfile.TemporaryFile(mode='w+') as output:
        session.log = output
        output.write('before\n')
        output.flush()
        offset = output.tell()
        assert session.recent_output() == 'before'
        assert output.tell() == offset
        output.write('after\n')
        output.flush()
        assert session.recent_output() == 'before\nafter'


@pytest.mark.parametrize('stage,failure', [
    ('observer', RuntimeError('observer failed')), ('observer', KeyboardInterrupt()),
    ('timeout', None), ('mission', RuntimeError('firmware disarmed')),
])
def test_loop_closes_physics_backend_and_session_on_failure_or_timeout(monkeypatch, stage, failure):
    pytest.importorskip('msp_link')
    from examples.altitde_hold_bt import demo
    session = MagicMock()
    session.__enter__.return_value = session
    session.close.return_value = None
    session.__exit__.side_effect = lambda *args: session.close()
    session.commands.read_status.side_effect = lambda: SimpleNamespace(
        armed=False, arming_disabled_flags=1 << 2, received_at=time.monotonic())
    monkeypatch.setattr(demo, 'SitlSession', Mock(return_value=session))
    backend = Mock()
    backend.received_at = None
    backend.latest = (0.0,) * 4
    monkeypatch.setattr(demo, 'BetaflightBackend', Mock(return_value=backend))
    physics = []
    original = demo.PyBulletPhysics
    def create(*args):
        value = original(*args)
        physics.append(value)
        return value
    monkeypatch.setattr(demo, 'PyBulletPhysics', create)
    observer = Mock(side_effect=failure) if stage == 'observer' else None
    if stage == 'mission':
        monkeypatch.setattr(demo.AltitudeHoldMission, 'command', Mock(side_effect=failure))
    output = io.StringIO()
    sink = logger.add(output, format='{message}')
    try:
        with pytest.raises(TimeoutError if failure is None else type(failure)):
            demo.run_demo(gui=False, max_duration_s=TIMESTEP_S, on_observation=observer)
    finally:
        logger.remove(sink)
    if stage == 'mission':
        assert 'blockers=RXLOSS' in output.getvalue()
        assert 'armed=False' in output.getvalue()
    session.close.assert_called_once()
    backend.close.assert_called_once()
    assert physics[0]._client is None


@pytest.mark.parametrize('stall_s', [0.0, INJECTED_STALL_S])
@pytest.mark.skipif(os.environ.get('BULLET_RUN_SITL') != '1', reason='Opt-in real paced SITL flight')
def test_live_headless_flight_and_cleanup(monkeypatch, stall_s):
    pytest.importorskip('msp_link')
    from examples.altitde_hold_bt import demo
    sessions = []
    original = demo.SitlSession
    class RecordingSession(original):
        def __enter__(self):
            result = super().__enter__()
            sessions.append(self)
            return result
    monkeypatch.setattr(demo, 'SitlSession', RecordingSession)
    samples = []
    def observe(observation):
        source = observation.ground_truth
        samples.append(source)
        if (stall_s and source.position_m[2] > MIN_STALL_HEIGHT_M
                and source.step_index % STALL_INTERVAL_STEPS == 0):
            time.sleep(stall_s)
    result = demo.run_demo(gui=False, on_observation=observe)
    times = dict(result.transitions)
    assert list(times) == list(Phase)
    assert 10 <= times[Phase.LAND] - times[Phase.HOLD] <= 10 + TIMESTEP_S + TIME_TOLERANCE_S
    hovering = [s for s in samples if times[Phase.HOLD] <= s.simulation_time_s < times[Phase.LAND]]
    peak_error = max(abs(s.position_m[2] - 3) for s in hovering)
    assert peak_error <= .05
    assert result.observation.ground_truth.position_m[2] <= .025
    assert max(max(s.rotor_speeds_rad_s) for s in samples) > 0
    assert sessions[0].process.poll() is not None
    assert sessions[0].client.stop.is_set()
    assert not Path(sessions[0].work.name).exists()
    print(f'Live SITL: injected stall {stall_s:.3f}s, '
          f'{times[Phase.LAND] - times[Phase.HOLD]:.3f} s hold, peak error {peak_error:.4f} m')


def test_stalled_loop_spaces_fdm_packets_instead_of_bursting_to_catch_up(monkeypatch):
    pytest.importorskip('msp_link')
    from examples.altitde_hold_bt import demo
    now = [0.0]
    sent_at = []
    monkeypatch.setattr(demo.time, 'monotonic', lambda: now[0])
    def sleep(duration_s):
        assert duration_s >= 0
        now[0] += duration_s
    monkeypatch.setattr(demo.time, 'sleep', sleep)
    session = MagicMock()
    session.__enter__.return_value = session
    session.__exit__.return_value = False
    def read_status():
        now[0] += SIMULATED_MSP_DELAY_S
        return SimpleNamespace(armed=False, arming_disabled_flags=1, received_at=now[0])
    session.commands.read_status.side_effect = read_status
    monkeypatch.setattr(demo, 'SitlSession', Mock(return_value=session))
    backend = Mock()
    backend.received_at = None
    backend.latest = (0.0,) * 4
    backend.command.side_effect = lambda observation: sent_at.append(now[0])
    monkeypatch.setattr(demo, 'BetaflightBackend', Mock(return_value=backend))
    def stalled_observer(observation):
        if observation.ground_truth.step_index == STALL_AT_STEP:
            now[0] += INJECTED_STALL_S
    with pytest.raises(TimeoutError):
        demo.run_demo(gui=False, max_duration_s=PACE_TEST_DURATION_S, on_observation=stalled_observer)
    gaps = [later - earlier for earlier, later in zip(sent_at, sent_at[1:])]
    assert len(gaps) > STALL_AT_STEP
    assert min(gaps) >= TIMESTEP_S - TIME_TOLERANCE_S, gaps
    assert max(gaps) >= INJECTED_STALL_S - TIME_TOLERANCE_S
