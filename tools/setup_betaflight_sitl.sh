#!/usr/bin/env bash
# Build unmodified, source-pinned Betaflight SITL for the Gazebo FDM protocol.
set -euo pipefail

readonly RELEASE_TAG="2026.6.2"
readonly RELEASE_COMMIT="e0b7bb0"
readonly REPOSITORY_URL="https://github.com/betaflight/betaflight.git"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="${1:-$repo_root/.sitl/betaflight-source}"

if ! command -v git >/dev/null || ! command -v make >/dev/null; then
    echo "Install git, make, and a C compiler before building Betaflight SITL." >&2
    exit 1
fi

if [[ -d "$source_dir/.git" ]]; then
    if [[ -n "$(git -C "$source_dir" status --porcelain)" ]]; then
        echo "SITL requires a clean upstream checkout: $source_dir" >&2
        echo "Preserve local changes and move the old checkout aside before rerunning setup." >&2
        exit 1
    fi
    git -C "$source_dir" fetch --depth 1 origin "refs/tags/$RELEASE_TAG:refs/tags/$RELEASE_TAG"
else
    git clone --depth 1 --branch "$RELEASE_TAG" "$REPOSITORY_URL" "$source_dir"
fi

git -C "$source_dir" checkout --detach "$RELEASE_TAG"
actual_commit="$(git -C "$source_dir" rev-parse HEAD)"
if [[ "$actual_commit" != "$RELEASE_COMMIT"* ]]; then
    echo "Expected $RELEASE_TAG at $RELEASE_COMMIT, found $actual_commit" >&2
    exit 1
fi

make -C "$source_dir" TARGET=SITL clean
make -C "$source_dir" TARGET=SITL -j"$(nproc)"
binary="$source_dir/obj/main/betaflight_SITL.elf"
if [[ ! -x "$binary" ]]; then
    echo "SITL build completed without $binary" >&2
    exit 1
fi
mkdir -p "$repo_root/bin"
installed_binary="$repo_root/bin/betaflight_SITL.elf"
staged_binary="$(mktemp "$repo_root/bin/.sitl-install.XXXXXX")"
trap 'rm -f "$staged_binary"' EXIT
install -m 755 "$binary" "$staged_binary"
mv -f "$staged_binary" "$installed_binary"
echo "SITL ready: $installed_binary"
