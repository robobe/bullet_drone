import configparser
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from plotjuggler import launch


def test_saved_profile_has_tcp_timestamp_and_plot_preferences():
    config = configparser.ConfigParser(interpolation=None)
    config.read(launch.PROFILE_TEMPLATE / 'PlotJuggler/PlotJuggler4.conf')
    assert config['General']['MainWindow.streamingSource'] == 'ZMQ Subscriber'
    assert config['General'].getboolean('MainWindow.buttonRatio')
    assert config['General'].getint('MainWindow.streamingBufferValue') == 120
    stream = json.loads(json.loads(config['PluginConfig']['ZMQ%20Subscriber']))
    assert (stream['address'], stream['port'], stream['transport'], stream['mode']) == ('127.0.0.1', 5555, 'tcp://', 'connect')
    assert stream['topics'] == 'render/drone_1/pose' and stream['default_encoding'] == 'json'
    assert json.loads(stream['_parser_config']) == {
        'use_embedded_timestamp': True, 'timestamp_field_name': 'simulation_time_s',
    }


@pytest.mark.parametrize('exit_code', [0, 7])
def test_launcher_forwards_arguments_and_isolates_settings(monkeypatch, tmp_path, exit_code):
    original_config = tmp_path / 'normal-settings'
    monkeypatch.setenv('XDG_CONFIG_HOME', str(original_config))
    monkeypatch.setenv('PLOTJUGGLER_PROFILE_TEST', 'preserved')
    copied = []
    expected = (launch.PROFILE_TEMPLATE / 'PlotJuggler/PlotJuggler4.conf').read_bytes()

    def run(command, *, env, check):
        assert command == [str(launch.PLOTJUGGLER_EXECUTABLE), '--layout', 'position.xml']
        assert check is False
        assert env['PLOTJUGGLER_PROFILE_TEST'] == 'preserved'
        runtime = Path(env['XDG_CONFIG_HOME'])
        copied.append(runtime)
        assert runtime != original_config
        assert (runtime / 'PlotJuggler/PlotJuggler4.conf').read_bytes() == expected
        (runtime / 'PlotJuggler/PlotJuggler4.conf').write_text('temporary viewer changes')
        return SimpleNamespace(returncode=exit_code)

    monkeypatch.setattr(launch.subprocess, 'run', run)
    assert launch.launch_plotjuggler(['--layout', 'position.xml']) == exit_code
    assert not copied[0].exists()
    assert not original_config.exists()
    assert (launch.PROFILE_TEMPLATE / 'PlotJuggler/PlotJuggler4.conf').read_bytes() == expected


def test_missing_executable_cleans_up_profile(monkeypatch):
    runtime_dirs = []
    def run(*args, **kwargs):
        runtime_dirs.append(Path(kwargs['env']['XDG_CONFIG_HOME']))
        raise FileNotFoundError('PlotJuggler executable missing')
    monkeypatch.setattr(launch.subprocess, 'run', run)
    with pytest.raises(FileNotFoundError):
        launch.launch_plotjuggler([])
    assert not runtime_dirs[0].exists()


def test_viewer_task_uses_profile_launcher():
    tasks = json.loads(Path('.vscode/tasks.json').read_text())['tasks']
    viewer = next(task for task in tasks if task['label'] == 'Open PlotJuggler ZMQ ground truth layout')
    assert viewer['command'] == 'uv'
    assert viewer['args'] == ['run', '--locked', '--all-packages', '--all-extras', '--all-groups',
                              'python', '${workspaceFolder}/plotjuggler/launch.py',
                              '--layout', '${workspaceFolder}/plotjuggler/layouts/ground_truth_position.xml']
