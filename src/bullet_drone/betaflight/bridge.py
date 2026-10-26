"""Exclusive bounded UDP transport. No physics or sensor interpretation."""
import socket
import time

HOST = '127.0.0.1'
MOTOR_PORT = 9002
FDM_PORT = 9003
MAX_DRAIN_PACKETS = 256
MAX_PACKET_BYTES = 256


class UdpSitlBridge:
    def __init__(self, *, host=HOST, motor_port=MOTOR_PORT, fdm_port=FDM_PORT, clock=time.monotonic):
        self.host, self.motor_port, self.fdm_port = host, motor_port, fdm_port
        self.clock = clock
        self.socket = None

    def open(self):
        self.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((self.host, self.motor_port))
            sock.setblocking(False)
        except BaseException:
            sock.close()
            raise
        self.socket = sock

    def send_fdm(self, packet):
        self.socket.sendto(packet, (self.host, self.fdm_port))

    def receive_motor_packets(self):
        packets = []
        for _ in range(MAX_DRAIN_PACKETS):
            try:
                packet, address = self.socket.recvfrom(MAX_PACKET_BYTES)
            except BlockingIOError:
                break
            if address[0] == self.host:
                packets.append((packet, self.clock()))
        return packets

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
