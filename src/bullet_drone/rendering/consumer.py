"""Simulation-time scheduling with latest state and initial/final publication."""
from bullet_drone.rendering.contracts import Renderer
from bullet_drone.rendering.models import SnapshotBuilder


class RenderConsumer:
    def __init__(self, builder: SnapshotBuilder, renderer: Renderer, rate_hz: float):
        self.builder, self.renderer, self.rate_hz = builder, renderer, rate_hz
        self._last = None
        self._sent = None
        self._next = 0.
        self._closed = False

    def reset(self, observation):
        if self._closed:
            raise RuntimeError('consumer is closed')
        self._last = self.builder.build(observation)
        self.renderer.reset(self._last)
        self._sent = self._last
        self._next = self._last.simulation_time_s + 1 / self.rate_hz

    def consume(self, observation):
        if self._closed or self._last is None:
            raise RuntimeError('consumer is not active')
        self._last = self.builder.build(observation)
        t = self._last.simulation_time_s
        if t + 1e-12 >= self._next:
            self.renderer.render(self._last)
            self._sent = self._last
            self._next = (int((t + 1e-12) * self.rate_hz) + 1) / self.rate_hz

    def close(self):
        if not self._closed:
            self._closed = True
            try:
                if self._last is not None and self._last != self._sent:
                    self.renderer.render(self._last)
            finally:
                self.renderer.close()
