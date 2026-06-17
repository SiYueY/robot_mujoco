#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OUTER_WORKSPACE_ROOT="$(cd "${WORKSPACE_ROOT}/.." && pwd)"

source_if_exists() {
  local candidate="$1"
  if [[ -f "${candidate}" ]]; then
    : "${AMENT_TRACE_SETUP_FILES:=}"
    : "${AMENT_PYTHON_EXECUTABLE:=/usr/bin/python3}"
    : "${COLCON_TRACE:=}"
    export AMENT_TRACE_SETUP_FILES
    export AMENT_PYTHON_EXECUTABLE
    export COLCON_TRACE
    local had_nounset=0
    if [[ $- == *u* ]]; then
      had_nounset=1
      set +u
    fi
    # shellcheck disable=SC1090
    source "${candidate}"
    if [[ ${had_nounset} -eq 1 ]]; then
      set -u
    fi
  fi
}

prepare_environment() {
  source_if_exists "/opt/ros/humble/setup.bash"
  source_if_exists "${OUTER_WORKSPACE_ROOT}/install/setup.bash"
  source_if_exists "${WORKSPACE_ROOT}/install/setup.bash"
}

configure_asan_ubsan_runtime() {
  local lsan_suppressions="${SCRIPT_DIR}/lsan.supp"
  export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1:new_delete_type_mismatch=0:strict_string_checks=1:check_initialization_order=1}"
  export UBSAN_OPTIONS="${UBSAN_OPTIONS:-print_stacktrace=1:halt_on_error=1}"
  if [[ -f "${lsan_suppressions}" ]]; then
    export LSAN_OPTIONS="${LSAN_OPTIONS:-suppressions=${lsan_suppressions}:print_suppressions=0}"
  fi
}

workspace_override_args() {
  local packages=("$@")
  local overrides=()
  local package=""
  for package in "${packages[@]}"; do
    if [[ -d "${WORKSPACE_ROOT}/install/${package}" || -d "${OUTER_WORKSPACE_ROOT}/install/${package}" ]]; then
      overrides+=("${package}")
    fi
  done
  if [[ ${#overrides[@]} -gt 0 ]]; then
    printf '%s\0' --allow-overriding "${overrides[@]}"
  fi
}

colcon_cmd() {
  prepare_environment
  cd "${WORKSPACE_ROOT}"
  colcon "$@"
}
