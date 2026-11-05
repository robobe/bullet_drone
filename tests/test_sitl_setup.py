"""Exercise setup without downloading or compiling firmware."""
import os
from pathlib import Path
import subprocess

import pytest


SETUP_SCRIPT = Path(__file__).resolve().parents[1] / 'tools/setup_betaflight_sitl.sh'


@pytest.mark.parametrize('dirty', [False, True])
def test_setup_builds_clean_upstream_or_refuses_local_changes(tmp_path, dirty):
    source = tmp_path / 'firmware'
    (source / '.git').mkdir(parents=True)
    commands = tmp_path / 'commands'
    commands.mkdir()
    scripts = {
        'git': '''#!/bin/sh
printf '%s\\n' "$*" >> "$SITL_TEST_GIT_LOG"
case "$*" in
  *'status --porcelain') printf '%s' "$SITL_TEST_STATUS" ;;
  *'rev-parse HEAD') echo e0b7bb01b17b21351057e9ead2d1ab39dd44fa16 ;;
esac
''',
        'make': '''#!/bin/sh
printf '%s\\n' "$*" >> "$SITL_TEST_MAKE_LOG"
mkdir -p "$SITL_TEST_SOURCE/obj/main"
touch "$SITL_TEST_SOURCE/obj/main/betaflight_SITL.elf"
chmod +x "$SITL_TEST_SOURCE/obj/main/betaflight_SITL.elf"
''',
    }
    for name, script in scripts.items():
        executable = commands / name
        executable.write_text(script)
        executable.chmod(0o755)
    git_log, make_log = tmp_path / 'git.log', tmp_path / 'make.log'
    env = dict(os.environ, PATH=str(commands) + os.pathsep + os.environ['PATH'],
               SITL_TEST_GIT_LOG=str(git_log), SITL_TEST_MAKE_LOG=str(make_log),
               SITL_TEST_SOURCE=str(source), SITL_TEST_STATUS=' M sitl.c' if dirty else '')
    result = subprocess.run(['bash', str(SETUP_SCRIPT), str(source)], env=env,
                            capture_output=True, text=True)
    if dirty:
        assert result.returncode != 0
        assert 'clean upstream checkout' in result.stderr
        assert not make_log.exists()
        assert 'checkout' not in git_log.read_text()
    else:
        assert result.returncode == 0, result.stderr
        calls = make_log.read_text().splitlines()
        assert len(calls) == 2
        assert calls[0].endswith('TARGET=SITL clean')
        assert 'TARGET=SITL -j' in calls[1]
        assert 'SITL_SENSOR_ONLY' not in make_log.read_text()
        assert 'apply' not in git_log.read_text()
