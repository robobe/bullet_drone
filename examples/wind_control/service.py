"""Application-owned command, wind and spawn dispatch at physics step boundaries."""
from dataclasses import asdict, replace
import time

from bullet_drone.config import WindSettings, ConfigurationError
from bullet_simulation_protocol.transport import ZmqIo
from bullet_simulation_protocol.control import SimulationCommand, GetSimulationState, SimulationStatus, CommandReply
from bullet_simulation_protocol.spawn import SpawnRequest, SpawnState, SpawnReply, GetSpawnRequest
from bullet_simulation_protocol.wind import (
    GetWindRequest, SetWindRequest, WindSettingsMessage, WindReadback, WindReply,
)

DEFAULT_REQUEST_MAX_AGE_S = 1.
DEFAULT_SETUP_TAKEOFF_HEIGHT_M = 3.


class SimulationService:
    def __init__(self, transport, physics_id, *, clock=time.monotonic):
        if 'wind_endpoint' not in transport:
            raise ConfigurationError('Wind control requires a transport profile with wind_endpoint')
        self.physics_id = physics_id
        self.clock = clock
        self.max_age_s = transport.get('wind_io', {}).get('request_max_age_s', DEFAULT_REQUEST_MAX_AGE_S)
        self.spawn_state = None
        self.running = False
        self.ready_to_start = True
        self.reason = 'setup'
        self.io = ZmqIo(reply_endpoint=transport['wind_endpoint'])

    def __enter__(self):
        self.io.start()
        return self

    def __exit__(self, *args):
        self.io.close()

    def snapshot(self, simulation, observation):
        state, wind = observation.ground_truth, simulation.wind
        return WindReadback(self.physics_id, state.episode_id, state.step_index,
                            wind.simulation_time_s,
                            WindSettingsMessage.from_dict(asdict(wind.settings)),
                            tuple(wind.velocity_world_m_s))

    def update(self, simulation, observation, *, replace_spawn=None, reset_simulation=None, start_mission=None):
        if self.io.error:
            raise RuntimeError(self.io.error)
        message = self.io.poll_request()
        if message is None:
            return
        action = message['payload'].get('action')
        if action in ('start', 'stop', 'reset', 'get_simulation_state'):
            return self._command(message, observation, reset_simulation, start_mission)
        if message['payload'].get('action') in ('get_spawn', 'set_spawn'):
            return self._spawn(message, observation, not self.running, replace_spawn)
        status, reason = 'current', 'current_wind' 
        try:
            value = message['payload']
            if value.get('action') == 'get_wind':
                GetWindRequest.from_dict(value)
            else:
                request = SetWindRequest.from_dict(value)
                if request.physics_id != self.physics_id or request.episode_id != observation.ground_truth.episode_id:
                    raise ValueError('simulation identity changed')
                if not 0 <= self.clock() - request.issued_ms / 1000 <= self.max_age_s:
                    raise ValueError('expired or future wind request')
                settings = WindSettings(**asdict(request.settings))
                simulation.set_wind(settings)
                status, reason = 'applied', 'wind_applied'
        except (ValueError, TypeError) as error:
            status, reason = 'rejected', str(error)[:256] or 'invalid_wind_request'
        self.io.reply(message['request_id'], WindReply(status, reason, self.snapshot(simulation, observation)))

    def _spawn(self, message, observation, stopped, replace_spawn):
        if self.spawn_state is None:
            self.spawn_state = SpawnState(2, self.physics_id, None,
                observation.ground_truth.episode_id, 'Default',
                tuple(observation.ground_truth.position_m), DEFAULT_SETUP_TAKEOFF_HEIGHT_M, None, 'idle', 'initial_spawn')
        self.spawn_state = replace(self.spawn_state, episode_id=observation.ground_truth.episode_id)
        status, reason = 'current', 'current_spawn'
        request = None
        try:
            value = message['payload']
            if value.get('action') == 'get_spawn':
                GetSpawnRequest.from_dict(value)
            else:
                request = SpawnRequest.from_dict(value)
                if request.physics_id != self.physics_id or request.episode_id != observation.ground_truth.episode_id:
                    raise ValueError('simulation identity changed')
                if not 0 <= self.clock() - request.issued_ms / 1000 <= self.max_age_s:
                    raise ValueError('expired or future spawn request')
                if not stopped or replace_spawn is None:
                    raise ValueError('spawn requires stopped simulation')
                observation = replace_spawn(request)
                self.spawn_state = SpawnState(2, self.physics_id, None,
                    observation.ground_truth.episode_id, request.scenario_name,
                    request.position_m, request.takeoff_height_m, request.request_id,
                    'applied', 'spawn_applied')
                self.ready_to_start = True
                self.reason = 'spawn_applied'
                status, reason = 'applied', 'spawn_applied'
        except (ValueError, TypeError, RuntimeError, ConfigurationError) as error:
            status, reason = 'rejected', str(error)[:256] or 'invalid_spawn_request'
            if request is not None:
                self.spawn_state = replace(self.spawn_state, request_id=request.request_id,
                                           result='rejected', reason=reason)
        self.io.reply(message['request_id'], SpawnReply(status, reason, self.spawn_state))
        return observation

    def status(self, observation):
        return SimulationStatus(self.physics_id, observation.ground_truth.episode_id,
            'running' if self.running else 'stopped', self.reason,
            observation.ground_truth.simulation_time_s, self.ready_to_start)

    def complete(self):
        self.running = False
        self.ready_to_start = False
        self.reason = 'completed'

    def _command(self, message, observation, reset_simulation, start_mission):
        status, reason = 'current', 'current_simulation_state'
        try:
            value = message['payload']
            if value.get('action') == 'get_simulation_state':
                GetSimulationState.from_dict(value)
            else:
                request = SimulationCommand.from_dict(value)
                if request.physics_id != self.physics_id or request.episode_id != observation.ground_truth.episode_id:
                    raise ValueError('simulation identity changed')
                if not 0 <= self.clock() - request.issued_ms / 1000 <= self.max_age_s:
                    raise ValueError('expired or future command')
                if request.action == 'start':
                    if self.running:
                        reason = 'already_running'
                    else:
                        if not self.ready_to_start:
                            raise ValueError('reset_required')
                        if start_mission is None:
                            raise ValueError('start_unavailable')
                        start_mission(observation)
                        self.running, self.ready_to_start = True, False
                        self.reason, status, reason = 'started', 'applied', 'started'
                elif request.action == 'stop':
                    if self.running:
                        self.running, self.ready_to_start = False, False
                        self.reason, status, reason = 'stopped_by_command', 'applied', 'stopped_by_command'
                    else:
                        reason = 'already_stopped'
                else:
                    if self.running:
                        raise ValueError('reset_requires_stopped')
                    if reset_simulation is None:
                        raise ValueError('reset_unavailable')
                    observation = reset_simulation()
                    self.ready_to_start = True
                    self.reason, status, reason = 'reset', 'applied', 'reset'
        except (ValueError, TypeError, RuntimeError, ConfigurationError) as error:
            status, reason = 'rejected', str(error)[:256] or 'invalid_command'
        self.io.reply(message['request_id'], CommandReply(status, reason, self.status(observation)))
        return observation
