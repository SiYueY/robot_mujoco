#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

PACKAGES=(
  mujoco_simulation
  mujoco_simulation_ros
  mujoco_hardware
  robot_mujoco
)
mapfile -d '' -t BUILD_OVERRIDE_ARGS < <(workspace_override_args "${PACKAGES[@]}")

colcon_cmd build --packages-select "${PACKAGES[@]}" "${BUILD_OVERRIDE_ARGS[@]}" --event-handlers console_direct+
colcon_cmd test --packages-select "${PACKAGES[@]:0:3}" --return-code-on-test-failure
colcon_cmd test-result --all --verbose --test-result-base build
