"""Poll MSP status without pausing physics packet delivery."""
from threading import Event, Thread


STATUS_PERIOD_S = 0.1


class StatusWorker:
    def __init__(self, commands):
        self.commands = commands
        self.latest = None
        self.error = None
        self.stop = Event()
        self.thread = Thread(target=self._run, name='red-truck-status', daemon=True)

    def __enter__(self):
        self.latest = self.commands.read_status()
        self.thread.start()
        return self

    def _run(self):
        while not self.stop.wait(STATUS_PERIOD_S):
            try:
                self.latest = self.commands.read_status()
            except Exception as error:
                self.error = error
                return

    def read(self):
        if self.error is not None:
            raise RuntimeError(f'MSP status polling failed: {self.error}') from self.error
        return self.latest

    def __exit__(self, *exc):
        self.stop.set()
        self.thread.join()
