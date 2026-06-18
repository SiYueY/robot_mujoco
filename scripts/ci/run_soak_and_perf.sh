#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

SOAK_SECONDS="${SOAK_SECONDS:-3600}"
OUTPUT_PATH="${OUTPUT_PATH:-build/performance_baseline.json}"
BASELINE_REFERENCE="${BASELINE_REFERENCE:-}"
COMPARISON_OUTPUT="${COMPARISON_OUTPUT:-build/performance_comparison.json}"
COMPARISON_MARKDOWN_OUTPUT="${COMPARISON_MARKDOWN_OUTPUT:-build/performance_comparison.md}"
BUILD_PACKAGES=(mujoco_simulation robot_mujoco_ros2)
mapfile -d '' -t BUILD_OVERRIDE_ARGS < <(workspace_override_args "${BUILD_PACKAGES[@]}")

colcon_cmd build --packages-select "${BUILD_PACKAGES[@]}" "${BUILD_OVERRIDE_ARGS[@]}"

prepare_environment
cd "${WORKSPACE_ROOT}"
python3 mujoco_simulation/test/performance/run_baseline.py \
  --workspace-root "${WORKSPACE_ROOT}" \
  --output "${OUTPUT_PATH}" \
  --soak-seconds "${SOAK_SECONDS}"

if [[ -n "${BASELINE_REFERENCE}" ]]; then
  python3 scripts/ci/compare_performance_baseline.py \
    --baseline "${BASELINE_REFERENCE}" \
    --candidate "${OUTPUT_PATH}" \
    --output "${COMPARISON_OUTPUT}" \
    --markdown-output "${COMPARISON_MARKDOWN_OUTPUT}"
fi
