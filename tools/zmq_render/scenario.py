"""Current-episode collision acceptance; no physics or geometry mutation."""
from collections import deque
from copy import deepcopy
import time

from bullet_msgs.scenario import COLLISION_MAX_AGE_S, validate_collision, validate_end


HISTORY_CAPACITY = 241
MICROSECONDS_PER_SECOND = 1_000_000


class ScenarioTermination:
    def __init__(self, *, clock=time.monotonic):
        self.clock = clock
        self.history = deque(maxlen=HISTORY_CAPACITY)
        self.result = None

    def reset(self):
        self.history.clear()
        self.result = None

    def observe(self, source):
        self.history.append(dict(source))
        cutoff = source['measurement_us'] - COLLISION_MAX_AGE_S * MICROSECONDS_PER_SECOND
        while len(self.history) > 1 and self.history[0]['measurement_us'] < cutoff:
            self.history.popleft()

    def accept(self, report, *, controller_id, running, controller_live):
        validate_collision(report)
        if self.result is not None or not running or not controller_live or report['controller_id'] != controller_id:
            return False
        now = self.clock()
        if any(not 0 <= now - report[k] / MICROSECONDS_PER_SECOND <= COLLISION_MAX_AGE_S
               for k in ('issued_us',)):
            return False
        source = report['source']
        if not 0 <= now - source['measurement_us'] / MICROSECONDS_PER_SECOND <= COLLISION_MAX_AGE_S:
            return False
        if source not in self.history:
            return False
        result = dict(schema_version=1, event_id=report['event_id'], collision=deepcopy(report),
                      final_source=dict(self.history[-1]), outcome='success', reason='target_hit',
                      ended_us=int(now * MICROSECONDS_PER_SECOND))
        validate_end(result)
        self.result = result
        return True
