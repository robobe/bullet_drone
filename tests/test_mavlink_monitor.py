from unittest.mock import Mock

import pytest

from examples.terrain_following_mavlink.telemetry import MavlinkMonitor, mavutil


def test_missing_heartbeat_closes_connection(monkeypatch):
    connection = Mock()
    connection.wait_heartbeat.return_value = None
    monkeypatch.setattr(mavutil, 'mavlink_connection', Mock(return_value=connection))
    with pytest.raises(TimeoutError, match='heartbeat'):
        with MavlinkMonitor():
            pass
    connection.close.assert_called_once()


def test_monitor_stops_worker_before_connection_close(monkeypatch):
    connection = Mock()
    connection.recv_match.return_value = None
    monkeypatch.setattr(mavutil, 'mavlink_connection', Mock(return_value=connection))
    monitor = MavlinkMonitor()
    with monitor:
        assert monitor.thread.is_alive()
    assert not monitor.thread.is_alive()
    connection.close.assert_called_once()


def test_mavlink_example_reuses_acro_mission_and_wiring(monkeypatch):
    from examples.terrain_following_mavlink import demo
    from examples.terrain_following_mavlink.sitl import SitlSession
    from tests.test_terrain_following_bt_example import mission, observation, DT_S
    from examples.terrain_following_bt.mission import Phase
    from types import SimpleNamespace

    value = mission()
    value.reset(observation())
    value._transition(Phase.OUTBOUND, 0)
    channels = value.command(observation(.1, clearance_m=3),
                             SimpleNamespace(armed=True, arming_disabled_flags=0), DT_S, False)
    assert channels[4:6] == (2000, 1000)
    run = Mock()
    monkeypatch.setattr(demo, 'run_terrain', run)
    demo.run_demo()
    assert run.call_args.kwargs['sitl_session_factory'] is SitlSession


def test_missing_messages_and_stale_heartbeat_are_reported(monkeypatch):
    from examples.terrain_following_mavlink import telemetry

    monitor = MavlinkMonitor()
    monitor.connection = Mock()
    monitor.connection.recv_match.return_value = None
    monkeypatch.setattr(telemetry.time, 'monotonic', Mock(side_effect=[0.0, 4.0]))
    log = Mock(side_effect=lambda *args: monitor.stop.set())
    monkeypatch.setattr(telemetry.logger, 'log', log)
    monitor._run()
    assert log.call_args.args[0] == 'WARNING'
    assert log.call_args.args[3] is True
    assert set(log.call_args.args[4].values()) == {'MISSING'}
