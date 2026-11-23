"""Pace FDM packets at the transport boundary without catch-up bursts."""
import time

from bullet_drone.betaflight.bridge import UdpSitlBridge


class PacedSitlBridge(UdpSitlBridge):
    def __init__(self, period_s, *, clock=time.monotonic, sleep=time.sleep):
        super().__init__(clock=clock)
        self.period_s = period_s
        self.sleep = sleep
        self.last_send = None
        self.sent = 0

    def send_fdm(self, packet):
        if self.last_send is not None:
            remaining = self.last_send + self.period_s - self.clock()
            while remaining > 0:
                self.sleep(remaining)
                remaining = self.last_send + self.period_s - self.clock()
        super().send_fdm(packet)
        self.last_send = self.clock()
        self.sent += 1
