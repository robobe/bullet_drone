"""Launch installed PlotJuggler with temporary copies of project preferences."""

from collections.abc import Sequence
import os
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory


PROFILE_TEMPLATE = Path(__file__).resolve().parent / 'profile'
PLOTJUGGLER_EXECUTABLE = Path.home() / '.local/bin/plotjuggler'


def launch_plotjuggler(arguments: Sequence[str]) -> int:
    """Forward native CLI arguments; protect user preferences and tracked defaults."""
    with TemporaryDirectory(prefix='bullet-drone-plotjuggler-') as config_home:
        shutil.copytree(PROFILE_TEMPLATE, config_home, dirs_exist_ok=True)
        environment = {**os.environ, 'XDG_CONFIG_HOME': config_home}
        result = subprocess.run([str(PLOTJUGGLER_EXECUTABLE), *arguments],
                                env=environment, check=False)
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(launch_plotjuggler(sys.argv[1:]))
