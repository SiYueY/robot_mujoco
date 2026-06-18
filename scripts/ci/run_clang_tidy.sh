#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

if ! command -v clang-tidy >/dev/null 2>&1; then
  echo "clang-tidy not found" >&2
  exit 1
fi

BUILD_PACKAGES=(
  mujoco_simulation
  robot_mujoco_ros2
)
mapfile -d '' -t BUILD_OVERRIDE_ARGS < <(workspace_override_args "${BUILD_PACKAGES[@]}")

colcon_cmd build \
  --packages-select "${BUILD_PACKAGES[@]}" \
  "${BUILD_OVERRIDE_ARGS[@]}" \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cd "${WORKSPACE_ROOT}"
mapfile -t SOURCES < <(rg --files \
  mujoco_simulation/src \
  mujoco_simulation/include \
  robot_mujoco_ros2/src \
  robot_mujoco_ros2/include \
  -g'*.cpp' -g'*.cc' -g'*.hpp')

for source_file in "${SOURCES[@]}"; do
  build_dir="build/$(dirname "${source_file}" | cut -d/ -f1)"
  clang-tidy -p "${build_dir}" "${source_file}"
done
