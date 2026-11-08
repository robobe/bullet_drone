from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import subprocess

import pytest

from examples.takeoff_image_viewer import main as runner
from examples.takeoff_image_viewer.viewer import ppm_frame


@pytest.mark.parametrize('encoding,pixels', [
    ('rgb8', b'\xff\0\0x\0\xff\0x'),
    ('bgr8', b'\0\0\xffx\0\xff\0x'),
])
def test_ppm_converts_channels_and_excludes_stride_padding(encoding, pixels):
    from bullet_image_transport.shm import FrameReader, FrameWriter
    writer = FrameWriter(1, 2, encoding=encoding, step=4)
    reader = FrameReader('camera/front/image')
    try:
        metadata, copied = reader.read(writer.write('camera/front/image', pixels))
        assert ppm_frame(metadata, copied) == b'P6\n1 2\n255\n\xff\0\0\0\xff\0'
    finally:
        reader.close()
        writer.close()


@pytest.mark.parametrize('failure', [None, RuntimeError('renderer failed'), KeyboardInterrupt()])
def test_child_cleanup_on_completion_failure_and_interrupt(monkeypatch, failure):
    from examples.altitude_hold_external_render import demo
    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr(runner.subprocess, 'Popen', Mock(return_value=process))
    def mission(*args, on_observation):
        on_observation(None)
        if failure is not None:
            raise failure
        return 'complete'
    monkeypatch.setattr(demo, 'run_demo', mission)
    options = SimpleNamespace(renderer=Path('render'), render_config=Path('physics.json'))
    if failure is None:
        assert runner.run_example(options) == 'complete'
    else:
        with pytest.raises(type(failure)):
            runner.run_example(options)
    process.terminate.assert_called_once()
    process.wait.assert_called_once()


def test_early_viewer_exit_stops_mission(monkeypatch):
    from examples.altitude_hold_external_render import demo
    process = Mock()
    process.poll.return_value = 1
    monkeypatch.setattr(runner.subprocess, 'Popen', Mock(return_value=process))
    monkeypatch.setattr(demo, 'run_demo', lambda *args, on_observation: on_observation(None))
    with pytest.raises(RuntimeError, match='Camera viewer closed'):
        runner.run_example(SimpleNamespace(renderer=Path('render'), render_config=Path('physics.json')))
    process.terminate.assert_not_called()


def test_unresponsive_viewer_is_killed(monkeypatch):
    from examples.altitude_hold_external_render import demo
    process = Mock()
    process.poll.return_value = None
    process.wait.side_effect = [subprocess.TimeoutExpired('viewer', 3), 0]
    monkeypatch.setattr(runner.subprocess, 'Popen', Mock(return_value=process))
    monkeypatch.setattr(demo, 'run_demo', lambda *args, **kwargs: None)
    runner.run_example(SimpleNamespace(renderer=Path('render'), render_config=Path('physics.json')))
    process.kill.assert_called_once()


def test_fps_counts_only_displayed_frames_over_actual_elapsed_time():
    from examples.takeoff_image_viewer.viewer import FpsCounter
    counter = FpsCounter(10.0)
    assert counter.update(10.2, displayed=True) is None
    assert counter.update(10.5, displayed=False) is None
    assert counter.update(11.0, displayed=True) == 2.0
    assert counter.update(12.0, displayed=False) == 0.0
    assert counter.update(14.0, displayed=True) == 0.5
