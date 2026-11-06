"""Standalone mission and owned-process failures, without needing Godot exports."""
import json
from pathlib import Path
import socket
import subprocess
import sys
from unittest.mock import Mock
from uuid import uuid4

import pytest

from bullet_drone.models import DroneObservation, DroneState
from examples.altitude_hold_external_render import demo
from examples.altitude_hold_external_render.demo_cli import CliParser, DemoOptions
from examples.altitude_hold_external_render.mission import Phase
from external_render.renderer import ExternalRenderer
from external_render.build import bundle_dependencies
from bullet_msgs.shm import SLOTS, slot_name


def observation():
    return DroneObservation(DroneState('test-episode', 0, 0., (0., 0., .015),
                                      (0., 0., 0., 1.), (0., 0., 0.), (0., 0., 0.)))


@pytest.fixture
def profile(tmp_path):
    with socket.socket() as poses, socket.socket() as images, socket.socket() as control:
        sockets = (poses, images, control)
        for sock in sockets:
            sock.bind(('127.0.0.1', 0))
        endpoints = [f'tcp://127.0.0.1:{sock.getsockname()[1]}' for sock in sockets]
    transport = dict(zip(('telemetry_endpoint', 'image_endpoint', 'control_endpoint'), endpoints))
    (tmp_path / 'transport.json').write_text(json.dumps(transport))
    config = tmp_path / 'physics.json'
    config.write_text(json.dumps(dict(transport_config='transport.json', fps=30,
        camera=dict(width_px=2, height_px=2, fov_x_deg=90))))
    return config, transport


def executable(tmp_path, source):
    path = tmp_path / 'renderer'
    path.write_text(f'#!{sys.executable}\n' + source)
    path.chmod(0o755)
    return path


def assert_ports_free(transport):
    for endpoint in transport.values():
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(('127.0.0.1', int(endpoint.rsplit(':', 1)[1])))


def test_missing_binary_has_build_instructions(profile):
    config, _ = profile
    with pytest.raises(FileNotFoundError, match='external_render/build.py'):
        ExternalRenderer(config.parent / 'missing', config)


@pytest.mark.parametrize('endpoint', ['image_endpoint', 'control_endpoint', 'telemetry_endpoint'])
def test_occupied_renderer_port_does_not_start_process(profile, tmp_path, monkeypatch, endpoint):
    config, transport = profile
    launch = Mock()
    monkeypatch.setattr(subprocess, 'Popen', launch)
    with socket.socket() as occupied:
        occupied.bind(('127.0.0.1', int(transport[endpoint].rsplit(':', 1)[1])))
        with pytest.raises(RuntimeError, match='port unavailable|Address already in use'):
            with ExternalRenderer(executable(tmp_path, 'pass\n'), config):
                pytest.fail('Occupied image port was accepted')
    launch.assert_not_called()
    assert_ports_free(transport)


@pytest.mark.parametrize('failure', ['timeout', 'exit', 'interrupt'])
def test_renderer_failure_closes_child_and_releases_ports(profile, tmp_path, failure):
    config, transport = profile
    source = 'raise SystemExit(7)\n' if failure == 'exit' else 'import time\ntime.sleep(30)\n'
    error = {'timeout': TimeoutError, 'exit': RuntimeError, 'interrupt': KeyboardInterrupt}[failure]
    renderer = ExternalRenderer(executable(tmp_path, source), config)
    with pytest.raises(error):
        with renderer:
            if failure == 'interrupt':
                raise KeyboardInterrupt
            renderer.wait_ready(observation(), timeout_s=.3 if failure == 'exit' else .05)
    assert renderer.process.poll() is not None
    assert not renderer.publisher.thread.is_alive()
    assert_ports_free(transport)


def test_godot_launch_failure_closes_started_communication(profile, tmp_path, monkeypatch):
    config, transport = profile
    renderer = ExternalRenderer(executable(tmp_path, 'pass\n'), config)
    monkeypatch.setattr(subprocess, 'Popen', Mock(side_effect=OSError('launch failed')))
    with pytest.raises(OSError, match='launch failed'):
        with renderer:
            pytest.fail('Renderer launch unexpectedly succeeded')
    assert not renderer.publisher.thread.is_alive()
    assert renderer.images.closed
    assert renderer.log.closed
    assert_ports_free(transport)


def test_shutdown_removes_only_owned_renderer_shm_slots(profile, tmp_path):
    config, _ = profile
    owned, foreign = uuid4().hex, uuid4().hex
    owned_paths = [Path('/dev/shm') / slot_name(owned, index).lstrip('/') for index in range(SLOTS)]
    foreign_path = Path('/dev/shm') / slot_name(foreign, 0).lstrip('/')
    foreign_path.touch()
    source = ("from pathlib import Path\nimport time\n"
              + ''.join(f'Path({str(path)!r}).touch()\n' for path in owned_paths)
              + f"print('SHM publisher session: {owned}', flush=True)\ntime.sleep(30)\n")
    try:
        with pytest.raises(TimeoutError):
            with ExternalRenderer(executable(tmp_path, source), config) as renderer:
                renderer.wait_ready(observation(), timeout_s=.3)
        assert all(not path.exists() for path in owned_paths)
        assert foreign_path.exists()
    finally:
        for path in (*owned_paths, foreign_path):
            path.unlink(missing_ok=True)


def test_internal_mission_flies_holds_lands_and_closes(monkeypatch):
    viewer = Mock()
    viewer.__enter__ = Mock(return_value=viewer)
    viewer.__exit__ = Mock(return_value=False)
    monkeypatch.setattr(demo, 'ExternalRenderer', Mock(return_value=viewer))
    monkeypatch.setattr(demo.time, 'sleep', lambda duration: None)
    original = demo.RcSimulation
    simulations = []
    def simulation(*args):
        value = original(*args)
        simulations.append(value)
        return value
    monkeypatch.setattr(demo, 'RcSimulation', simulation)
    observations = []
    result = demo.run_demo('unused', 'unused', on_observation=observations.append)
    transitions = {entry.phase: entry.simulation_time_s for entry in result.transitions}
    assert list(transitions) == list(Phase)
    assert transitions[Phase.LAND] - transitions[Phase.HOLD] == pytest.approx(10, abs=demo.TIMESTEP_S)
    hovering = [o.ground_truth.position_m[2] for o in observations
                if transitions[Phase.HOLD] <= o.ground_truth.simulation_time_s < transitions[Phase.LAND]]
    assert hovering and max(abs(z - 3) for z in hovering) <= .05
    assert result.observation.ground_truth.position_m[2] <= .02
    assert result.observation.ground_truth.rotor_speeds_rad_s == (0.,) * 4
    assert simulations[0]._closed
    viewer.wait_ready.assert_called_once()
    viewer.__exit__.assert_called_once()


@pytest.mark.parametrize('failure', [RuntimeError('renderer failed'), KeyboardInterrupt()])
def test_midflight_failure_closes_physics_and_viewer(monkeypatch, failure):
    viewer = Mock()
    viewer.__enter__ = Mock(return_value=viewer)
    viewer.__exit__ = Mock(return_value=False)
    viewer.ensure_running.side_effect = failure
    simulation = Mock()
    simulation.reset.return_value = observation()
    monkeypatch.setattr(demo, 'ExternalRenderer', Mock(return_value=viewer))
    monkeypatch.setattr(demo, 'RcSimulation', Mock(return_value=simulation))
    with pytest.raises(type(failure)):
        demo.run_demo('unused', 'unused')
    simulation.close.assert_called_once()
    viewer.__exit__.assert_called_once()


def test_bundle_dependencies_keeps_sonames_and_system_cpp_runtime(tmp_path, monkeypatch):
    library = tmp_path / 'libzmq.so.5.2.5'
    library.write_bytes(b'library')
    monkeypatch.setattr(subprocess, 'check_output', lambda *a, **k:
        f'libzmq.so.5 => {library} (0x123)\nlibstdc++.so.6 => /system/libstdc++.so.6 (0x456)\n')
    destination = tmp_path / 'lib'
    bundle_dependencies(tmp_path / 'extension.so', destination)
    assert (destination / 'libzmq.so.5').read_bytes() == b'library'
    assert not (destination / 'libstdc++.so.6').exists()


@pytest.mark.parametrize('arguments,status', [(['--help'], 0), (['--unknown'], 2)])
def test_cli_help_and_errors_do_not_start_physics(arguments, status):
    result = subprocess.run([sys.executable, '-m', 'examples.altitude_hold_external_render.main', *arguments],
                            text=True, capture_output=True, timeout=10)
    assert result.returncode == status, result.stderr
    assert 'pybullet build time' not in result.stderr
    assert CliParser().parse([]) == DemoOptions()
