"""Exercise setup without downloading or compiling firmware."""
import os
from pathlib import Path
import subprocess

import pytest


SETUP_SCRIPT = Path(__file__).resolve().parents[1] / 'tools/setup_betaflight_sitl.sh'


@pytest.mark.parametrize('gps', [False, True])
@pytest.mark.parametrize('mavlink', [False, True])
@pytest.mark.parametrize('failure', ['none', 'dirty', 'build', 'marker'])
def test_setup_builds_clean_upstream_or_refuses_local_changes(tmp_path, mavlink, failure, gps):
    if failure == "marker" and not gps:
        pytest.skip("marker verification applies only to GPS builds")
    dirty = failure == "dirty"
    filename = "setup_betaflight_mavlink_sitl.sh" if mavlink else "setup_betaflight_sitl.sh"
    isolated = tmp_path / "repo"
    (isolated / "tools").mkdir(parents=True)
    setup = isolated / "tools" / filename
    setup.write_text((SETUP_SCRIPT.parent / filename).read_text())
    binary_name = "betaflight_mavlink_SITL.elf" if mavlink else "betaflight_SITL.elf"
    if gps:
        binary_name = binary_name.replace("_SITL", "_gps_SITL")
    installed = isolated / "bin" / binary_name
    installed.parent.mkdir()
    installed.write_text("previous firmware")
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
[ "$SITL_TEST_FAIL" = "build" ] && exit 1
mkdir -p "$SITL_TEST_SOURCE/obj/main"
if [ "$SITL_TEST_FAIL" = "marker" ]; then
    printf upstream > "$SITL_TEST_SOURCE/obj/main/betaflight_SITL.elf"
else
    printf BULLET_GPS_FDM_V1 > "$SITL_TEST_SOURCE/obj/main/betaflight_SITL.elf"
fi
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
               SITL_TEST_SOURCE=str(source), SITL_TEST_FAIL=failure, SITL_TEST_STATUS=' M sitl.c' if dirty else '')
    result = subprocess.run(['bash', str(setup), *(['--gps'] if gps else []), str(source)], env=env,
                            capture_output=True, text=True)
    if failure != "none":
        assert installed.read_text() == "previous firmware"
    if dirty:
        assert result.returncode != 0
        assert 'clean upstream checkout' in result.stderr
        assert not make_log.exists()
        assert 'checkout' not in git_log.read_text()
    elif failure in ("build", "marker"):
        assert result.returncode != 0
        if gps:
            assert 'apply --reverse' in git_log.read_text()
    else:
        assert installed.read_bytes() == b"BULLET_GPS_FDM_V1"
        assert os.access(installed, os.X_OK)
        assert str(installed) in result.stdout
        assert result.returncode == 0, result.stderr
        calls = make_log.read_text().splitlines()
        assert len(calls) == 2
        assert calls[0].endswith('TARGET=SITL clean')
        assert 'TARGET=SITL' in calls[1] and '-j' in calls[1]
        assert ('USE_TELEMETRY_MAVLINK' in calls[1]) == mavlink
        assert 'SITL_SENSOR_ONLY' not in make_log.read_text()
        assert ('apply' in git_log.read_text()) == gps
        assert ('BULLET_GPS_FDM' in calls[1]) == gps
