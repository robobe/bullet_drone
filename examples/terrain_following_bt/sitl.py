"""Example-owned Betaflight process and MSP connection."""

from contextlib import suppress
import os
from pathlib import Path
import socket
import subprocess
import tempfile

from bullet_drone.betaflight.firmware import validate_gps_firmware
from loguru import logger
from examples.common.msp import Client, FlightCommands, MspError, TcpTransport


PROFILE_PATH = Path(__file__).resolve().parents[2] / 'configs/betaflight.config'
HOST = '127.0.0.1'
MSP_PORT = 5761
UDP_PORTS = (9002, 9003, 9004)
STARTUP_TIMEOUT_S = 10.0
STOP_TIMEOUT_S = 3.0
SAFE_CHANNELS = (1500, 1500, 1000, 1500, 1000, 1000) + (1000,) * 12
EXPECTED_VARIANT = b'BTFL'
EXPECTED_VERSION = bytes((26, 6, 2))
LOG_TAIL_BYTES = 2000


class SitlSession:
    def __init__(self, binary, *, profile_path=PROFILE_PATH, extra_ports=(), gps_enabled=False):
        self.profile_path = profile_path
        self.extra_ports = extra_ports
        self.binary = Path(binary).resolve()
        if not self.binary.is_file() or not os.access(self.binary, os.X_OK):
            raise ValueError('SITL binary missing or not executable. Run tools/setup_betaflight_sitl.sh first.')
        validate_gps_firmware(self.binary, self.profile_path, gps_enabled=gps_enabled)
        self.process = self.client = self.work = self.log = None
        self.commands = None

    def __enter__(self):
        for kind, port in ((socket.SOCK_STREAM, MSP_PORT),
                           *((socket.SOCK_STREAM, port) for port in self.extra_ports),
                           *((socket.SOCK_DGRAM, port) for port in UDP_PORTS)):
            with socket.socket(socket.AF_INET, kind) as probe:
                if kind == socket.SOCK_STREAM:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    probe.bind((HOST, port))
                except OSError as error:
                    raise RuntimeError(f'SITL port {port} is occupied. Stop the other simulation first.') from error
        try:
            self.work = tempfile.TemporaryDirectory(prefix='terrain-following-bt-')
            self.log = tempfile.TemporaryFile(mode='w+')
            logger.info('Starting SITL: binary={} profile={} workdir={}',
                        self.binary, self.profile_path, self.work.name)
            subprocess.run([str(self.binary), '--config', str(self.profile_path)], cwd=self.work.name,
                           stdout=self.log, stderr=self.log, timeout=STARTUP_TIMEOUT_S, check=True)
            self.process = subprocess.Popen([str(self.binary), '--ip', HOST], cwd=self.work.name,
                                            stdout=self.log, stderr=self.log)
            self.client = Client(TcpTransport(HOST, MSP_PORT)).start()
            if not self.client.connected.wait(STARTUP_TIMEOUT_S):
                self.check_running()
                raise RuntimeError(f'SITL MSP connection timed out: {self.client.error}')
            self.commands = FlightCommands(self.client)
            identity = self.commands.identify()
            if identity['variant'] != EXPECTED_VARIANT or identity['version'][:3] != EXPECTED_VERSION:
                raise ValueError('Expected Betaflight 2026.6.2 SITL')
            self.client.rc.update(SAFE_CHANNELS)
            logger.info('SITL ready: pid={} version={}.{}.{} MSP={}:{}; Acro arm=1000 throttle=1000',
                        self.process.pid, *EXPECTED_VERSION, HOST, MSP_PORT)
            return self
        except BaseException:
            logger.error('SITL startup failed. Recent firmware output:\n{}', self.recent_output())
            self.close()
            raise

    def recent_output(self):
        if self.log is None or self.log.closed:
            return '(no firmware output)'
        size = os.fstat(self.log.fileno()).st_size
        return os.pread(self.log.fileno(), LOG_TAIL_BYTES, max(0, size - LOG_TAIL_BYTES)).decode(
            errors='replace').strip() or '(no firmware output)'

    def check_running(self):
        if self.process.poll() is not None:
            raise RuntimeError('SITL exited: ' + self.recent_output())
        if self.client is not None and not self.client.connected.is_set():
            raise RuntimeError(f'MSP connection lost: {self.client.error}')

    def close(self):
        try:
            if self.client is not None:
                logger.info('Sending Acro disarm and closing MSP')
                try:
                    self.client.rc.clear()
                    if self.client.connected.is_set() and self.commands is not None:
                        with suppress(MspError, TimeoutError, OSError):
                            self.commands.send_rc(SAFE_CHANNELS)
                finally:
                    self.client.close()
        finally:
            try:
                if self.process is not None and self.process.poll() is None:
                    logger.info('Stopping owned SITL process pid={}', self.process.pid)
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=STOP_TIMEOUT_S)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=STOP_TIMEOUT_S)
            finally:
                if self.log is not None:
                    self.log.close()
                if self.work is not None:
                    self.work.cleanup()

    def __exit__(self, exc_type, exc, traceback):
        try:
            if exc_type is not None:
                logger.error('SITL output before teardown:\n{}', self.recent_output())
        finally:
            self.close()
