"""Application-owned wind requests, applied only between physics steps."""
from dataclasses import asdict
import time

from bullet_drone.config import WindSettings, ConfigurationError
from bullet_simulation_protocol.transport import ZmqIo
from bullet_simulation_protocol.wind import (
    GetWindRequest, SetWindRequest, WindSettingsMessage, WindReadback, WindReply,
)

DEFAULT_REQUEST_MAX_AGE_S = 1.


class WindService:
    def __init__(self, transport, physics_id, *, clock=time.monotonic):
        if 'wind_endpoint' not in transport:
            raise ConfigurationError('Wind control requires a transport profile with wind_endpoint')
        self.physics_id = physics_id
        self.clock = clock
        self.max_age_s = transport.get('wind_io', {}).get('request_max_age_s', DEFAULT_REQUEST_MAX_AGE_S)
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

    def update(self, simulation, observation):
        if self.io.error:
            raise RuntimeError(self.io.error)
        message = self.io.poll_request()
        if message is None:
            return
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
