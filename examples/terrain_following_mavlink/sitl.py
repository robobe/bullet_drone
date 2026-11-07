"""Own MAVLink monitoring inside the existing firmware lifecycle."""
from pathlib import Path

from examples.terrain_following_bt.sitl import SitlSession as BaseSession
from examples.terrain_following_mavlink.telemetry import MavlinkMonitor

PROFILE_PATH = Path(__file__).resolve().parents[2] / 'configs/betaflight-mavlink.config'
TELEMETRY_PORT = 5762


class SitlSession(BaseSession):
    def __init__(self, binary):
        super().__init__(binary, profile_path=PROFILE_PATH, extra_ports=(TELEMETRY_PORT,))
        self.monitor = None

    def __enter__(self):
        super().__enter__()
        try:
            self.monitor = MavlinkMonitor()
            self.monitor.__enter__()
            return self
        except BaseException:
            self.close()
            raise

    def close(self):
        try:
            if self.monitor is not None:
                self.monitor.__exit__(None, None, None)
        finally:
            super().close()
