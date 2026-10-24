"""Wind request bookkeeping; the runner alone calls the physics update API."""
from collections import OrderedDict
from dataclasses import asdict

from bullet_msgs.control import COMMAND_MAX_AGE_S, HEARTBEAT_TIMEOUT_S
from bullet_msgs.wind import validate_request

MAX_REQUEST_HISTORY = 128


class WindControl:
    def __init__(self):
        self.request_id = None
        self.result, self.reason = 'idle', 'startup'
        self.history = OrderedDict()
        self.pending = None

    def accept(self, value, control):
        validate_request(value)
        now = control.clock()
        if (value['physics_id'] != control.physics_id or value['controller_id'] != control.controller_id
                or control.last_heartbeat is None or now - control.last_heartbeat > HEARTBEAT_TIMEOUT_S
                or not 0 <= now - value['issued_ms'] / 1000 <= COMMAND_MAX_AGE_S):
            return False
        key = (value['controller_id'], value['request_id'])
        fingerprint = {k: v for k, v in value.items() if k != 'issued_ms'}
        previous = self.history.get(key)
        if previous is not None:
            if previous[0] == fingerprint:
                self.request_id, self.result, self.reason = value['request_id'], previous[1], previous[2]
            return False
        self.pending = value
        if value['episode_id'] != control.episode_id:
            self.finish(False, 'stale_episode')
            return False
        if control.state not in ('running', 'stopped'):
            self.finish(False, 'physics_unavailable')
            return False
        return True

    def finish(self, applied, reason):
        value = self.pending
        self.request_id = value['request_id']
        self.result, self.reason = ('applied' if applied else 'rejected'), reason
        fingerprint = {k: v for k, v in value.items() if k != 'issued_ms'}
        self.history[(value['controller_id'], self.request_id)] = (fingerprint, self.result, reason)
        while len(self.history) > MAX_REQUEST_HISTORY:
            self.history.popitem(last=False)
        self.pending = None

    def packet(self, control, observation, snapshot):
        return dict(schema_version=1, physics_id=control.physics_id, controller_id=control.controller_id,
                    episode_id=observation.ground_truth.episode_id, request_id=self.request_id,
                    settings=asdict(snapshot.settings), velocity_world_m_s=list(snapshot.velocity_world_m_s),
                    simulation_time_s=snapshot.simulation_time_s, result=self.result, reason=self.reason)
