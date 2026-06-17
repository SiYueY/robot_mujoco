#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

BUILD_BASE="${BUILD_BASE:-build_tsan}"
INSTALL_BASE="${INSTALL_BASE:-install_tsan}"
LOG_BASE="${LOG_BASE:-log_tsan}"
TSAN_PREFLIGHT_REPORT="${TSAN_PREFLIGHT_REPORT:-${LOG_BASE}/tsan_preflight.json}"
TSAN_SUMMARY_JSON="${TSAN_SUMMARY_JSON:-${LOG_BASE}/tsan_summary.json}"
TSAN_SUMMARY_MARKDOWN="${TSAN_SUMMARY_MARKDOWN:-${LOG_BASE}/tsan_summary.md}"
TSAN_CTEST_REGEX="${TSAN_CTEST_REGEX:-(test_status|test_command_buffer|test_sensor_scheduler|test_simulation_scheduler|test_simulation_ros_bridge)}"
TSAN_PACKAGES=(mujoco_simulation mujoco_simulation_ros mujoco_hardware)
TSAN_TEST_PACKAGES=(mujoco_simulation mujoco_simulation_ros)
mapfile -d '' -t BUILD_OVERRIDE_ARGS < <(workspace_override_args "${TSAN_PACKAGES[@]}")

workspace_path() {
  local candidate="$1"
  if [[ "${candidate}" = /* ]]; then
    printf '%s\n' "${candidate}"
  else
    printf '%s/%s\n' "${WORKSPACE_ROOT}" "${candidate}"
  fi
}

TSAN_PREFLIGHT_REPORT_PATH="$(workspace_path "${TSAN_PREFLIGHT_REPORT}")"
TSAN_SUMMARY_JSON_PATH="$(workspace_path "${TSAN_SUMMARY_JSON}")"
TSAN_SUMMARY_MARKDOWN_PATH="$(workspace_path "${TSAN_SUMMARY_MARKDOWN}")"

mkdir -p "$(workspace_path "${LOG_BASE}")"
mkdir -p "$(dirname "${TSAN_PREFLIGHT_REPORT_PATH}")" "$(dirname "${TSAN_SUMMARY_JSON_PATH}")" "$(dirname "${TSAN_SUMMARY_MARKDOWN_PATH}")"

write_tsan_summary() {
  local build_exit_code="${1:-0}"
  local test_exit_code="${2:-0}"
  local test_result_exit_code="${3:-0}"
  python3 "${SCRIPT_DIR}/summarize_tsan_results.py" \
    --workspace-root "${WORKSPACE_ROOT}" \
    --build-base "${WORKSPACE_ROOT}/${BUILD_BASE}" \
    --log-base "${WORKSPACE_ROOT}/${LOG_BASE}" \
    --preflight-report "${TSAN_PREFLIGHT_REPORT_PATH}" \
    --ctest-regex "${TSAN_CTEST_REGEX}" \
    --packages "${TSAN_PACKAGES[@]}" \
    --build-exit-code "${build_exit_code}" \
    --test-exit-code "${test_exit_code}" \
    --test-result-exit-code "${test_result_exit_code}" \
    --json-output "${TSAN_SUMMARY_JSON_PATH}" \
    --markdown-output "${TSAN_SUMMARY_MARKDOWN_PATH}"
}

run_tsan_preflight() {
  local probe_dir
  probe_dir="$(mktemp -d)"
  local probe_src="${probe_dir}/tsan_probe.cc"
  local probe_bin="${probe_dir}/tsan_probe"
  local probe_stdout="${probe_dir}/stdout.txt"
  local probe_stderr="${probe_dir}/stderr.txt"
  local compiler="${CXX:-g++}"

  cat >"${probe_src}" <<'EOF'
#include <thread>

int main() {
  int value = 0;
  std::thread worker([&value]() { value = 1; });
  worker.join();
  return value == 1 ? 0 : 1;
}
EOF

  if ! "${compiler}" -std=c++17 -fsanitize=thread -fno-omit-frame-pointer \
      "${probe_src}" -o "${probe_bin}" >"${probe_stdout}" 2>"${probe_stderr}"; then
    python3 - "${TSAN_PREFLIGHT_REPORT_PATH}" "${compiler}" "compile_failed" \
      "${probe_stdout}" "${probe_stderr}" <<'PY'
import json
import pathlib
import sys

report_path = pathlib.Path(sys.argv[1])
payload = {
    "status": sys.argv[3],
    "compiler": sys.argv[2],
    "stdout": pathlib.Path(sys.argv[4]).read_text(),
    "stderr": pathlib.Path(sys.argv[5]).read_text(),
}
report_path.write_text(json.dumps(payload, indent=2) + "\n")
PY
    cat "${probe_stderr}" >&2
    rm -rf "${probe_dir}"
    return 1
  fi

  if "${probe_bin}" >"${probe_stdout}" 2>"${probe_stderr}"; then
    rm -rf "${probe_dir}"
    return 0
  fi

  python3 - "${TSAN_PREFLIGHT_REPORT_PATH}" "${compiler}" "runtime_unsupported" \
    "${probe_stdout}" "${probe_stderr}" <<'PY'
import json
import pathlib
import sys

report_path = pathlib.Path(sys.argv[1])
payload = {
    "status": sys.argv[3],
    "compiler": sys.argv[2],
    "stdout": pathlib.Path(sys.argv[4]).read_text(),
    "stderr": pathlib.Path(sys.argv[5]).read_text(),
}
report_path.write_text(json.dumps(payload, indent=2) + "\n")
PY

  if rg -q "unexpected memory mapping" "${probe_stderr}"; then
    cat "${probe_stderr}" >&2
    echo "TSan runtime is unsupported on this host; wrote ${TSAN_PREFLIGHT_REPORT_PATH}" >&2
    rm -rf "${probe_dir}"
    return 2
  fi

  cat "${probe_stderr}" >&2
  rm -rf "${probe_dir}"
  return 1
}

preflight_status=0
run_tsan_preflight || preflight_status=$?
if [[ ${preflight_status} -ne 0 ]]; then
  write_tsan_summary 0 0 0
  if [[ ${preflight_status} -eq 2 ]]; then
    exit 0
  fi
  exit "${preflight_status}"
fi

build_status=0
test_status=0
test_result_status=0

colcon_cmd \
  --log-base "${LOG_BASE}" \
  build \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --packages-select "${TSAN_PACKAGES[@]}" \
  "${BUILD_OVERRIDE_ARGS[@]}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    "-DCMAKE_CXX_FLAGS=-fsanitize=thread -fno-omit-frame-pointer" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread" || build_status=$?

if [[ ${build_status} -eq 0 ]]; then
  source_if_exists "${WORKSPACE_ROOT}/${INSTALL_BASE}/setup.bash"
  cd "${WORKSPACE_ROOT}"
  colcon \
    --log-base "${LOG_BASE}" \
    test \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" \
    --packages-select "${TSAN_TEST_PACKAGES[@]}" \
    --ctest-args -R "${TSAN_CTEST_REGEX}" \
    --return-code-on-test-failure || test_status=$?
  colcon test-result --all --verbose --test-result-base "${BUILD_BASE}" || test_result_status=$?
fi

write_tsan_summary "${build_status}" "${test_status}" "${test_result_status}"

if [[ ${build_status} -ne 0 ]]; then
  exit "${build_status}"
fi
if [[ ${test_status} -ne 0 ]]; then
  exit "${test_status}"
fi
exit "${test_result_status}"
