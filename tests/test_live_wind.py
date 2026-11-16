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
