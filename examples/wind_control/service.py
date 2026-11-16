"""Apply one validated mutation at an application-owned step boundary."""
from dataclasses import asdict
import time

from bullet_drone.config import WindSettings, ConfigurationError
from bullet_simulation_protocol.transport import ZmqIo
from bullet_simulation_protocol.requests import parse_request, OperationReply

DEFAULT_REQUEST_MAX_QUEUE_AGE_S = 1.
DEFAULT_SETUP_TAKEOFF_HEIGHT_M = 3.
MAX_FAILURE_REASON_LENGTH = 256


class SimulationService:
    def __init__(self, transport, *, clock=time.monotonic):
        if 'request_endpoint' not in transport:
            raise ConfigurationError('Simulation control requires request_endpoint')
        self.clock = clock
        self.max_queue_age_s = transport.get('request_io', {}).get('request_max_queue_age_s', DEFAULT_REQUEST_MAX_QUEUE_AGE_S)
        self.running = False
        self.ready_to_start = True
        self.reason = 'setup'
        self.io = ZmqIo(reply_endpoint=transport['request_endpoint'])

    def __enter__(self):
        self.io.start()
        return self

    def __exit__(self, *args):
        self.io.close()

    def complete(self):
        self.running, self.ready_to_start = False, False
        self.reason = 'completed'

    def update(self, simulation, observation, *, replace_spawn=None, reset_simulation=None, start_mission=None):
        if self.io.error:
            raise RuntimeError(self.io.error)
        message = self.io.poll_request()
        if message is None:
            return observation
        try:
            if not 0 <= self.clock() - message['received_at'] <= self.max_queue_age_s:
                raise ValueError('request_expired_in_queue')
            request = parse_request(message['payload'])
            if request.drone_id != observation.ground_truth.drone_id:
                raise ValueError('unknown_drone')
            if request.topic == 'wind':
                simulation.set_wind(WindSettings(**asdict(request.payload)))
            elif request.topic == 'spawn':
                if self.running:
                    raise ValueError('spawn_requires_stopped')
                if replace_spawn is None:
                    raise ValueError('spawn_unavailable')
                observation = replace_spawn(request.payload)
                self.ready_to_start, self.reason = True, 'spawn_applied'
            else:
                action = request.payload.action
                if action == 'start':
                    if not self.running:
                        if not self.ready_to_start:
                            raise ValueError('reset_required')
                        if start_mission is None:
                            raise ValueError('start_unavailable')
                        start_mission(observation)
                        self.running, self.ready_to_start, self.reason = True, False, 'started'
                elif action == 'stop':
                    if self.running:
                        self.running, self.ready_to_start, self.reason = False, False, 'stopped_by_command'
                else:
                    if self.running:
                        raise ValueError('reset_requires_stopped')
                    if reset_simulation is None:
                        raise ValueError('reset_unavailable')
                    observation = reset_simulation()
                    self.ready_to_start, self.reason = True, 'reset'
            reply = OperationReply(True)
        except (ValueError, TypeError, RuntimeError, ConfigurationError) as error:
            reply = OperationReply(False, str(error)[:MAX_FAILURE_REASON_LENGTH] or 'request_failed')
        self.io.reply(message['token'], reply)
        return observation
