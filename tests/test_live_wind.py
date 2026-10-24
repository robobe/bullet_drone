"""Runtime wind updates preserve physics ownership, continuity and TCP identity."""
from dataclasses import asdict, replace
from math import exp
from types import SimpleNamespace
from unittest.mock import Mock
import json
import time

import pytest
import zmq

from bullet_drone.config import WindSettings, ForcesSettings, BodyDragSettings
from bullet_drone.forces.wind import WindModel
from bullet_drone.commands import RcInput
from bullet_drone.simulation import RcSimulation
from examples.position_square.demo import FLIGHT_CONTROL_SETTINGS
from tools.zmq_render.control import SimulationControl
from tools.zmq_render.wind import WindControl
from bullet_msgs.wind import REQUEST_TOPIC, validate_state
from bullet_msgs.control import HEARTBEAT_TOPIC


def test_live_edits_and_snapshot_reads_preserve_gust_history():
    settings = WindSettings(behavior='gusts', gust_stddev_m_s=(1.,)*3)
    wind, reference = WindModel(settings), WindModel(settings)
    wind.velocity(1.)
    reference.velocity(1.)
    for _ in range(100):
        assert wind.snapshot.simulation_time_s == 1.
    assert wind.velocity(2.) == reference.velocity(2.)
    before = wind.snapshot.velocity_world_m_s
    new = replace(settings, velocity_world_m_s=(5.,0.,0.), gust_stddev_m_s=(0.,)*3, correlation_time_s=2.)
    wind.set_settings(new, 2.)
    assert wind.snapshot.velocity_world_m_s == (before[0]+5.,before[1],before[2])
    assert wind.velocity(3.) == pytest.approx((5.+before[0]*exp(-.5),before[1]*exp(-.5),before[2]*exp(-.5)))
    # Turning gusts off clears the old fluctuation, rather than carrying it into constant wind.
    wind.set_settings(replace(new, behavior='constant'),3.)
    assert wind.snapshot.velocity_world_m_s == (5.,0.,0.)
    wind.set_settings(new,3.)
    assert wind.snapshot.velocity_world_m_s == (5.,0.,0.)
    wind.set_settings(replace(new, enabled=False),3.)
    assert wind.snapshot.velocity_world_m_s == (0.,0.,0.)


def test_seed_change_and_episode_reset_restart_selected_sequence():
    first = WindSettings(behavior='gusts', gust_stddev_m_s=(1.,)*3)
    second = replace(first, seed=17)
    wind = WindModel(first)
    wind.velocity(1.)
    wind.set_settings(second,1.)
    fresh = WindModel(second)
    assert wind.velocity(2.) == fresh.velocity(1.)
    wind.reset()
    assert wind.snapshot.settings == second
    assert wind.snapshot.velocity_world_m_s == (0.,0.,0.)
    assert wind.velocity(1.) == fresh.snapshot.velocity_world_m_s


def test_public_update_moves_body_without_applying_a_second_force(config):
    forces = replace(ForcesSettings.ideal(), body_drag=BodyDragSettings(enabled=True))
    simulation = RcSimulation(replace(config,forces=forces),FLIGHT_CONTROL_SETTINGS)
    try:
        before = simulation.reset()
        settings = WindSettings(velocity_world_m_s=(5.,0.,0.))
        simulation.set_wind(settings)
        assert simulation.wind.velocity_world_m_s == (5.,0.,0.)
        # Update itself does not step or teleport the body.
        assert simulation._observation is before
        after = simulation.step(RcInput())
        expected_force = .5*config.forces.air_density_kg_m3*forces.body_drag.cd_area_m2[0]*25
        assert after.ground_truth.linear_velocity_m_s[0] == pytest.approx(expected_force/config.drone.mass_kg*config.simulation.timestep_s)
        simulation.reset()
        assert simulation.wind.settings == settings
    finally:
        simulation.close()


def test_real_tcp_live_requests_retry_and_stale_ownership(tmp_path):
    with zmq.Context() as context, context.socket(zmq.XPUB) as publisher:
        publisher.setsockopt(zmq.LINGER,0)
        publisher.bind_to_random_port('tcp://127.0.0.1')
        path=tmp_path/'transport.json'
        path.write_text(json.dumps(dict(control_endpoint=publisher.getsockopt_string(zmq.LAST_ENDPOINT),telemetry_endpoint='tcp://127.0.0.1:5555',image_endpoint='tcp://127.0.0.1:5556')))
        clock=[time.monotonic()]
        wind=WindControl()
        control=SimulationControl(path,Mock(),clock=lambda:clock[0],wind=wind)
        try:
            control.episode_id='episode'
            for _ in range(3):
                assert publisher.poll(2000)
                publisher.recv()
            def send(value,topic=REQUEST_TOPIC):
                publisher.send_multipart([topic,json.dumps(value).encode()])
                assert control.socket.poll(2000)
                return control.poll()
            heartbeat=dict(schema_version=1,controller_id='godot',sequence=1,action='heartbeat',issued_ms=int(clock[0]*1000))
            assert send(heartbeat,HEARTBEAT_TOPIC) is None
            packet=dict(schema_version=1,physics_id=control.physics_id,episode_id='episode',controller_id='godot',request_id='wind1',issued_ms=int(clock[0]*1000),settings=asdict(WindSettings(velocity_world_m_s=(3.,0.,0.))))
            assert send(packet,REQUEST_TOPIC+b'/suffix') is None
            for changes in [dict(physics_id='other'),dict(controller_id='other'),dict(issued_ms=0),dict(settings={'bad':True})]:
                assert send(dict(packet,**changes)) is None
                assert wind.pending is None
            control.state='running'
            assert send(packet)=='wind'
            wind.finish(True,'wind_applied')
            assert send(packet) is None and wind.result=='applied'
            assert send(dict(packet,settings=asdict(WindSettings()))) is None
            assert send(dict(packet,request_id='old',episode_id='old')) is None
            assert wind.reason=='stale_episode'
            control.state='stopped'
            assert send(dict(packet,request_id='stopped'))=='wind'
            wind.finish(True,'wind_applied')
            snapshot=WindModel(WindSettings()).snapshot
            state=wind.packet(control,SimpleNamespace(ground_truth=SimpleNamespace(episode_id='episode')),snapshot)
            validate_state(state)
            clock[0]+=4
            assert send(dict(packet,request_id='expired',issued_ms=int(clock[0]*1000))) is None
        finally:
            control.close()
