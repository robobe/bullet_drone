import subprocess
import sys


def test_telemetry_imports_start_no_workers_or_io():
    code = '''
import builtins
import multiprocessing
import socket
import sys
import threading
from unittest.mock import patch

def forbidden(*args, **kwargs):
    raise AssertionError("telemetry import started recording resources")

with patch.object(threading.Thread, "start", forbidden), \\
     patch.object(multiprocessing.Process, "start", forbidden), \\
     patch.object(builtins, "open", forbidden), \\
     patch.object(socket, "socket", forbidden):
    import bullet_drone.simulation
    import bullet_drone.telemetry.topics
    assert "jsonschema" not in sys.modules
'''
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
