#!/usr/bin/env bash
# Build the source-pinned Betaflight SITL binary used by bullet_drone.
set -euo pipefail

readonly RELEASE_TAG="2026.6.2"
readonly RELEASE_COMMIT="e0b7bb0"
readonly REPOSITORY_URL="https://github.com/betaflight/betaflight.git"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
course_patch="$repo_root/tools/patches/betaflight-sensor-only.patch"
source_dir="${1:-$repo_root/.sitl/betaflight-source}"

if ! command -v git >/dev/null || ! command -v make >/dev/null; then
    echo "Install git, make, and a C compiler before building Betaflight SITL." >&2
    exit 1
fi

if [[ -d "$source_dir/.git" ]]; then
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

if git -C "$source_dir" apply --check "$course_patch" 2>/dev/null; then
    git -C "$source_dir" apply "$course_patch"
elif ! git -C "$source_dir" apply --reverse --check "$course_patch"; then
    echo "Course sensor patch conflicts with local firmware changes: $course_patch" >&2
    exit 1
fi

make -C "$source_dir" TARGET=SITL EXTRA_FLAGS="-DSITL_SENSOR_ONLY" -j"$(nproc)"
binary="$source_dir/obj/main/betaflight_SITL.elf"
if [[ ! -x "$binary" ]]; then
    echo "SITL build completed without $binary" >&2
    exit 1
fi
echo "SITL ready: $binary"
