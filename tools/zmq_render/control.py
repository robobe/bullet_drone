"""Application-side command input; physics remains owned by each example loop."""
import time
from uuid import uuid4

import zmq
from loguru import logger
from bullet_msgs import load_transport
from bullet_msgs.scenario import COLLISION_TOPIC, validate_collision
from bullet_msgs.control import (COMMAND_TOPIC, HEARTBEAT_TOPIC, HEARTBEAT_TIMEOUT_S,
    COMMAND_MAX_AGE_S, STATE_INTERVAL_S, MAX_CONTROL_BYTES, decode_json, validate_command)

POLL_TIMEOUT_MS = 25
MAX_POLL_MESSAGES = 16


class SimulationControl:
    def __init__(self, transport_path, publisher, *, clock=time.monotonic, scenario=None, spawn=None, wind=None):
        self.clock = clock
        self.scenario = scenario
        self.spawn = spawn
        self.wind = wind
        self.episode_id = None
        self.publisher = publisher
        self.physics_id = uuid4().hex
        self.controller_id = None
        self.last_sequence = 0
        self.last_heartbeat = None
        self.retired = set()
        self.state = 'stopped'
        self.reason = 'startup'
        self.last_state = float('-inf')
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        try:
            profile = load_transport(transport_path)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.RCVHWM, 16)
            self.socket.setsockopt(zmq.MAXMSGSIZE, MAX_CONTROL_BYTES)
            for topic in (COMMAND_TOPIC, HEARTBEAT_TOPIC):
                self.socket.setsockopt(zmq.SUBSCRIBE, topic)
            if spawn is not None:
                from bullet_msgs.spawn import REQUEST_TOPIC
                self.socket.setsockopt(zmq.SUBSCRIBE, REQUEST_TOPIC)
            if wind is not None:
                from bullet_msgs.wind import REQUEST_TOPIC
                self.socket.setsockopt(zmq.SUBSCRIBE, REQUEST_TOPIC)
            if scenario is not None:
                self.socket.setsockopt(zmq.SUBSCRIBE, COLLISION_TOPIC)
            self.socket.connect(profile['control_endpoint'])
        except BaseException:
            self.close()
            raise

    def transition(self, state, reason, observation):
        self.state, self.reason = state, reason
        self.publish(observation, force=True)

    def publish(self, observation, *, force=False):
        now = self.clock()
        self.episode_id = observation.ground_truth.episode_id
        if self.spawn is not None:
            self.publisher.set_spawn(self.spawn.packet(self, observation))
        if force or now - self.last_state >= STATE_INTERVAL_S:
            state = observation.ground_truth
            packet = dict(schema_version=2 if self.scenario is not None else 1, physics_id=self.physics_id,
                controller_id=self.controller_id, episode_id=state.episode_id, state=self.state,
                reason=self.reason, simulation_time_s=state.simulation_time_s)
            if self.scenario is not None:
                packet['terminal_result'] = self.scenario.result
            self.publisher.set_state(packet)
            self.last_state = now

    def poll(self):
        """Return at most one action, never touch a physics object on this socket path."""
        now = self.clock()
        if self.state != 'running':
            self.socket.poll(POLL_TIMEOUT_MS)
        for _ in range(MAX_POLL_MESSAGES):
            try:
                parts = self.socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            if self.wind is not None:
                from bullet_msgs.wind import REQUEST_TOPIC
                if len(parts) == 2 and parts[0] == REQUEST_TOPIC:
                    try:
                        if self.wind.accept(decode_json(parts[1]), self):
                            return 'wind'
                    except (ValueError, TypeError, UnicodeError, RecursionError):
                        pass
                    continue
            if self.spawn is not None:
                from bullet_msgs.spawn import REQUEST_TOPIC
                if len(parts) == 2 and parts[0] == REQUEST_TOPIC:
                    try:
                        if self.spawn.accept(decode_json(parts[1]), self):
                            return 'spawn'
                    except (ValueError, TypeError, UnicodeError, RecursionError):
                        pass
                    continue
            if len(parts) == 2 and parts[0] == COLLISION_TOPIC and self.scenario is not None:
                try:
                    report = validate_collision(decode_json(parts[1]))
                    live = self.last_heartbeat is not None and now - self.last_heartbeat <= HEARTBEAT_TIMEOUT_S
                    if self.scenario.accept(report, controller_id=self.controller_id,
                                            running=self.state == 'running', controller_live=live):
                        return 'collision'
                except (ValueError, TypeError, UnicodeError, RecursionError):
                    pass
                continue
            if len(parts) != 2 or parts[0] not in (COMMAND_TOPIC, HEARTBEAT_TOPIC):
                continue
            try:
                value = validate_command(decode_json(parts[1]))
                expected_topic = HEARTBEAT_TOPIC if value['action'] == 'heartbeat' else COMMAND_TOPIC
                if parts[0] != expected_topic:
                    continue
                now = self.clock()
                age = now - value['issued_ms'] / 1000
                if not 0 <= age <= COMMAND_MAX_AGE_S:
                    continue
            except (ValueError, UnicodeError, TypeError) as error:
                logger.warning('Rejected simulation control message: {}', error)
                continue
            controller = value['controller_id']
            if controller in self.retired:
                continue
            if controller != self.controller_id:
                if value['action'] != 'heartbeat':
                    continue
                if self.controller_id is not None:
                    self.retired.add(self.controller_id)
                self.controller_id = controller
                self.last_sequence = 0
                self.last_heartbeat = now
                self.last_sequence = value['sequence']
                if self.state == 'running':
                    return 'controller_changed'
                continue
            if value['sequence'] <= self.last_sequence:
                continue
            self.last_sequence = value['sequence']
            if value['action'] == 'heartbeat':
                self.last_heartbeat = now
                continue
            if self.last_heartbeat is not None and now - self.last_heartbeat <= HEARTBEAT_TIMEOUT_S:
                return value['action']
        if self.last_heartbeat is not None and now - self.last_heartbeat > HEARTBEAT_TIMEOUT_S:
            self.controller_id = None
            self.last_heartbeat = None
            self.last_sequence = 0
            if self.state == 'running':
                return 'controller_timeout'
        return None

    def close(self):
        self.socket.close(linger=0)
        self.context.term()
