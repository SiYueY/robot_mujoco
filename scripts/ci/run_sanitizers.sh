#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

SANITIZERS="${SANITIZERS:-address,undefined}"
BUILD_BASE="${BUILD_BASE:-build_asan_ubsan}"
INSTALL_BASE="${INSTALL_BASE:-install_asan_ubsan}"
LOG_BASE="${LOG_BASE:-log_asan_ubsan}"
BUILD_PACKAGES=(
  mujoco_simulation
  robot_mujoco_ros2
)
TEST_PACKAGES=(
  mujoco_simulation
)
CTEST_EXCLUDE_REGEX="${CTEST_EXCLUDE_REGEX:-test_camera_runtime|test_viewer}"
mapfile -d '' -t BUILD_OVERRIDE_ARGS < <(workspace_override_args "${BUILD_PACKAGES[@]}")

configure_asan_ubsan_runtime

colcon_cmd \
  --log-base "${LOG_BASE}" \
  build \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --packages-select "${BUILD_PACKAGES[@]}" \
  "${BUILD_OVERRIDE_ARGS[@]}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    "-DCMAKE_CXX_FLAGS=-fsanitize=${SANITIZERS} -fno-omit-frame-pointer" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=${SANITIZERS}" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=${SANITIZERS}"

source_if_exists "${WORKSPACE_ROOT}/${INSTALL_BASE}/setup.bash"
cd "${WORKSPACE_ROOT}"
# Keep the ASan/UBSan gate on the runtime core. ROS 2 node construction inside
# `robot_mujoco_ros2` currently triggers upstream
# `rclcpp` / `rmw` sanitizer failures on Humble, while viewer-backed tests hit
# MuJoCo simulate / Mesa driver sanitizer noise that is outside the runtime
# core we own here.
colcon \
  --log-base "${LOG_BASE}" \
  test \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --packages-select "${TEST_PACKAGES[@]}" \
  --ctest-args -E "(${CTEST_EXCLUDE_REGEX})" \
  --return-code-on-test-failure
colcon test-result --all --verbose --test-result-base "${BUILD_BASE}/mujoco_simulation"
