"""Build a relocatable Linux renderer package; runtime needs no source checkout."""
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parent
RENDER_SOURCE = ROOT.parents[1] / 'bullet_render'
TERRAIN_SOURCE = ROOT.parents[1] / 'terrain/terrain_heightmap.f32'
DISTRIBUTION = ROOT / 'dist/linux-x86_64'
NATIVE_LIBRARY = 'libzmq_camera.so'
SYSTEM_LIBRARIES = {
    'libc.so.6', 'libm.so.6', 'libstdc++.so.6', 'libgcc_s.so.1',
    'libpthread.so.0', 'librt.so.1', 'libdl.so.2',
}
LAUNCHER = '''#!/bin/sh
set -eu
package_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
export LD_LIBRARY_PATH="$package_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$package_dir/bin/bullet-render.x86_64" "$@"
'''


def bundle_dependencies(library: Path, destination: Path) -> None:
    output = subprocess.check_output(['ldd', str(library)], text=True)
    destination.mkdir()
    for line in output.splitlines():
        if '=> not found' in line:
            raise RuntimeError(f'Missing native runtime dependency: {line.strip()}')
        fields = line.split()
        if len(fields) >= 3 and fields[1] == '=>' and fields[0] not in SYSTEM_LIBRARIES:
            shutil.copy2(Path(fields[2]).resolve(), destination / fields[0])


def main() -> None:
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        raise RuntimeError('This export supports Linux x86_64 only')
    for command in ('godot', 'cmake', 'ninja', 'ldd'):
        if shutil.which(command) is None:
            raise RuntimeError(f'Install {command} before building the renderer')
    if not TERRAIN_SOURCE.is_file():
        raise FileNotFoundError(f'Missing shared terrain: {TERRAIN_SOURCE}')
    subprocess.run(['cmake', '--preset', 'debug'], cwd=RENDER_SOURCE, check=True)
    subprocess.run(['cmake', '--build', '--preset', 'debug'], cwd=RENDER_SOURCE, check=True)
    DISTRIBUTION.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='export-', dir=DISTRIBUTION.parent) as work:
        stage = Path(work) / 'project'
        output = Path(work) / 'package'
        shutil.copytree(RENDER_SOURCE / 'godot', stage,
                        ignore=shutil.ignore_patterns('.godot', 'bin', 'test_*', '*_probe*'))
        (stage / 'bin').mkdir()
        native = stage / 'bin' / NATIVE_LIBRARY
        shutil.copy2(RENDER_SOURCE / 'godot/bin' / NATIVE_LIBRARY, native)
        shutil.copy2(ROOT / 'export_presets.cfg', stage / 'export_presets.cfg')
        shutil.copy2(ROOT / 'configs/transport.json', stage / 'transport.json')
        (stage / 'terrain').mkdir(exist_ok=True)
        shutil.copy2(TERRAIN_SOURCE, stage / 'terrain/terrain_heightmap.f32')
        script = stage / 'main.gd'
        source = script.read_text()
        old_path = 'res://../../terrain/terrain_heightmap.f32'
        if old_path not in source:
            raise RuntimeError('Renderer terrain path changed; update the export recipe')
        script.write_text(source.replace(old_path, 'res://terrain/terrain_heightmap.f32'))
        scene = stage / 'main.tscn'
        scene.write_text(scene.read_text() + '\nviewer_only = true\ntransport_config_path = "res://transport.json"\n')
        project = stage / 'project.godot'
        # Keep ownership markers available if startup fails before the first image.
        project.write_text(project.read_text().replace('[application]',
            '[application]\nrun/flush_stdout_on_print=true', 1))
        (output / 'bin').mkdir(parents=True)
        shutil.copytree(ROOT / 'configs', output / 'configs')
        bundle_dependencies(native, output / 'lib')
        subprocess.run(['godot', '--headless', '--path', str(stage), '--editor',
                        '--import', '--frame-delay', '1000', '--quit'], check=True)
        subprocess.run(['godot', '--headless', '--path', str(stage), '--export-release',
                        'Linux', str(output / 'bin/bullet-render.x86_64')], check=True)
        launcher = output / 'render'
        launcher.write_text(LAUNCHER)
        launcher.chmod(0o755)
        (output / 'BUILD.txt').write_text(
            subprocess.check_output(['godot', '--version'], text=True)
            + 'Renderer source commit: '
            + subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=RENDER_SOURCE, text=True)
            + 'Requires Ubuntu 24.04 x86_64 system C/C++ runtimes and OpenGL drivers.\n')
        # Only replace generated output after the complete export has succeeded.
        if DISTRIBUTION.exists():
            shutil.rmtree(DISTRIBUTION)
        shutil.move(str(output), DISTRIBUTION)
    print(f'Renderer ready: {DISTRIBUTION / "render"}')


if __name__ == '__main__':
    main()
