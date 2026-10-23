"""Parent-side owner of the bounded queue and sender child lifecycle."""

from dataclasses import dataclass
import multiprocessing
from queue import Full
import time

from loguru import logger

from bullet_drone.models import DroneObservation
from tools.telemetry_sender.config import SenderSettings
from tools.telemetry_sender.worker import END_OF_STREAM, FINISHED, READY, run_sender


PROCESS_NAME = 'plotjuggler-udp-sender'
PROCESS_START_METHOD = 'spawn'
TERMINATE_JOIN_TIMEOUT_S = 0.5


@dataclass(frozen=True, slots=True)
class SenderStatistics:
    submitted: int
    overflow_drops: int
    skipped: int
    sent: int
    errors: int
    unsent: int
    error: str | None


class TelemetrySender:
    """Single-use optional sender. Runtime failures never escape into physics."""

    def __init__(self, settings: SenderSettings) -> None:
        self.settings = settings
        self._context = multiprocessing.get_context(PROCESS_START_METHOD)
        self._queue = None
        self._status = None
        self._process = None
        self._active = False
        self._started = False
        self._closed = False
        self._submitted = 0
        self._overflow = 0
        self._worker = {'sent': 0, 'skipped': 0, 'errors': 0, 'error': None}
        self._error = None
        self._latest = None
        self._last_queued = None

    @property
    def statistics(self) -> SenderStatistics:
        return SenderStatistics(self._submitted, self._overflow, self._worker['skipped'],
                                self._worker['sent'], max(self._worker['errors'], int(self._error is not None)),
                                max(0, self._submitted - self._worker['sent'] - self._worker['skipped']),
                                self._error or self._worker['error'])

    def _disable(self, error: str) -> None:
        self._active = False
        if self._error is None:
            self._error = error
            logger.error('PlotJuggler streaming disabled; flight continues: {}', error)

    def _poll(self) -> None:
        try:
            if self._status is not None and self._status.poll():
                kind, payload = self._status.recv()
                if kind == FINISHED:
                    self._worker = payload
                    self._active = False
                    if payload['error']:
                        self._disable(payload['error'])
        except (EOFError, OSError) as exc:
            self._disable(f'Sender status connection closed: {exc}')
        if self._active and not self._process.is_alive():
            self._disable(f'Sender process exited with code {self._process.exitcode}')

    def start(self) -> None:
        if self._started or self._closed:
            raise RuntimeError('TelemetrySender can only be started once')
        self._started = True
        child_status = None
        try:
            self._queue = self._context.Queue(maxsize=self.settings.queue_capacity)
            self._status, child_status = self._context.Pipe(duplex=False)
            self._process = self._context.Process(target=run_sender,
                args=(self._queue, child_status, self.settings), name=PROCESS_NAME)
            self._process.start()
            child_status.close()
            child_status = None
            if not self._status.poll(self.settings.startup_timeout_s):
                self._disable('Sender startup timed out')
                return
            kind, payload = self._status.recv()
            if kind != READY:
                if kind == FINISHED:
                    self._worker = payload
                self._disable(self._worker['error'] or 'Sender failed during startup')
                return
            self._active = True
        except Exception as exc:
            self._disable(f'Cannot start sender: {exc}')
        finally:
            if child_status is not None:
                child_status.close()

    def consume(self, observation: DroneObservation) -> None:
        if not self._active:
            return
        self._poll()
        if not self._active:
            return
        self._latest = observation.ground_truth
        try:
            self._queue.put_nowait(self._latest)
            self._submitted += 1
            self._last_queued = self._latest
        except Full:
            self._overflow += 1
        except (OSError, ValueError) as exc:
            self._disable(f'Cannot enqueue telemetry: {exc}')

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        deadline = time.monotonic() + self.settings.shutdown_timeout_s
        try:
            self._poll()
            if self._active:
                try:
                    # Only shutdown may wait for queue capacity: preserve a rejected final state.
                    if self._latest is not self._last_queued:
                        self._queue.put(self._latest, timeout=max(0.0, deadline - time.monotonic()))
                        self._submitted += 1
                    self._queue.put(END_OF_STREAM, timeout=max(0.0, deadline - time.monotonic()))
                except (Full, OSError, ValueError) as exc:
                    self._disable(f'Cannot finish sender before shutdown deadline: {exc}')
            if self._process is not None and self._process.pid is not None:
                self._process.join(timeout=max(0.0, deadline - time.monotonic()))
                self._poll()
                if self._process.is_alive():
                    self._disable('Sender shutdown timed out; pending samples were not delivered')
                    self._terminate()
                self._process.close()
        except BaseException:
            self._terminate()
            raise
        finally:
            self._active = False
            if self._status is not None:
                self._status.close()
                self._status = None
            if self._queue is not None:
                # A failed child may leave a full pipe; never wait forever for its feeder.
                self._queue.cancel_join_thread()
                self._queue.close()
                self._queue = None
            stats = self.statistics
            logger.info('PlotJuggler sender: {} submitted, {} sent, {} rate skips, {} overflow drops, '
                        '{} unsent, {} errors', stats.submitted, stats.sent, stats.skipped,
                        stats.overflow_drops, stats.unsent, stats.errors)

    def _terminate(self) -> None:
        if self._process is None or self._process.pid is None or not self._process.is_alive():
            return
        self._process.terminate()
        self._process.join(timeout=TERMINATE_JOIN_TIMEOUT_S)
        if self._process.is_alive():
            self._process.kill()
            self._process.join(timeout=TERMINATE_JOIN_TIMEOUT_S)
