#!/usr/bin/env bash

set -Eeuo pipefail

readonly EXPECTED_VERSION="3.12.0"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly MUJOCO_DIR="${PROJECT_DIR}/third_party/mujoco"
readonly BUILD_DIR="${MUJOCO_DIR}/build"
readonly INSTALL_DIR="${PROJECT_DIR}/mujoco"
readonly LEGACY_INSTALL_DIR="${MUJOCO_DIR}/install"

BUILD_TYPE="Release"
JOBS=""
GENERATOR=""

log_info() { printf 'info: %s\n' "$*"; }
log_warn() { printf 'warning: %s\n' "$*" >&2; }
log_error() { printf 'error: %b\n' "$*" >&2; }
die() { log_error "$*"; exit 1; }

check_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command was not found: $1"
}

check_source_tree() {
  [[ -f "${MUJOCO_DIR}/CMakeLists.txt" ]] || die "bundled MuJoCo source tree is missing.\n\nExpected:\n  ${MUJOCO_DIR}\n\nRun:\n  git submodule update --init --recursive"
  [[ -f "${MUJOCO_DIR}/include/mujoco/mujoco.h" ]] || die "bundled MuJoCo headers are missing: ${MUJOCO_DIR}/include/mujoco/mujoco.h"
}

check_build_tree() {
  [[ -f "${BUILD_DIR}/CMakeCache.txt" ]] || die "bundled MuJoCo build tree does not exist.\n\nExpected:\n  ${BUILD_DIR}\n\nRun:\n  ./scripts/mujoco.sh build"
}

check_install_tree() {
  [[ -f "${INSTALL_DIR}/include/mujoco/mujoco.h" ]] || die "bundled MuJoCo staging installation is missing.\n\nExpected:\n  ${INSTALL_DIR}\n\nRun:\n  ./scripts/mujoco.sh build"
  [[ -f "${INSTALL_DIR}/lib/cmake/mujoco/mujocoConfig.cmake" ]] || die "bundled MuJoCo staging package is incomplete: ${INSTALL_DIR}/lib/cmake/mujoco/mujocoConfig.cmake"
  compgen -G "${INSTALL_DIR}/lib/libmujoco.so*" >/dev/null || die "bundled MuJoCo staging library is missing from ${INSTALL_DIR}/lib"
}

version_from_header() {
  local header="$1"
  local encoded
  encoded="$(awk '$2 == "mjVERSION_HEADER" { print $3; exit }' "${header}")"
  [[ "${encoded}" =~ ^[0-9]+$ ]] || return 1
  printf '%d.%d.%d\n' "$((encoded / 1000000))" "$(((encoded / 1000) % 1000))" "$((encoded % 1000))"
}

read_source_version() { version_from_header "${MUJOCO_DIR}/include/mujoco/mujoco.h"; }
read_installed_version() { version_from_header "${INSTALL_DIR}/include/mujoco/mujoco.h"; }

validate_version() {
  local source_version installed_version
  source_version="$(read_source_version)" || die "could not determine bundled MuJoCo source version."
  [[ "${source_version}" == "${EXPECTED_VERSION}" ]] || die "bundled MuJoCo source version is ${source_version}; expected ${EXPECTED_VERSION}."
  if [[ -d "${INSTALL_DIR}" ]]; then
    installed_version="$(read_installed_version)" || die "could not determine staged MuJoCo version."
    [[ "${installed_version}" == "${EXPECTED_VERSION}" ]] || die "bundled MuJoCo staging install is ${installed_version}; expected ${EXPECTED_VERSION}.\n\nRun:\n  ./scripts/mujoco.sh rebuild"
  fi
}

validate_install() {
  check_install_tree
  validate_version
}

safe_remove_tree() {
  local target="$1"
  [[ -n "${target}" && "${target}" != "/" && "${target}" != "${HOME}" ]] || die "refusing unsafe deletion target: ${target}"
  [[ "${target}" == "${BUILD_DIR}" || "${target}" == "${INSTALL_DIR}" || "${target}" == "${LEGACY_INSTALL_DIR}" ]] || die "refusing unexpected deletion target: ${target}"
  [[ -e "${target}" ]] || return 0
  rm -rf -- "${target}"
}

configure_mujoco() {
  check_command cmake
  check_source_tree
  validate_version
  local args=(-S "${MUJOCO_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_STUDIO=OFF)
  [[ -n "${GENERATOR}" ]] && args+=(-G "${GENERATOR}")
  log_info "configuring bundled MuJoCo (${BUILD_TYPE})"
  # FetchContent invokes Git in child processes. HTTP/1.1 avoids interrupted
  # HTTP/2 pack transfers on common corporate proxies without changing the
  # caller's global Git configuration.
  cmake -E env \
    GIT_CONFIG_COUNT=1 \
    GIT_CONFIG_KEY_0=http.version \
    GIT_CONFIG_VALUE_0=HTTP/1.1 \
    cmake "${args[@]}"
}

build_mujoco() {
  check_build_tree
  local args=(--build "${BUILD_DIR}")
  [[ -n "${JOBS}" ]] && args+=(--parallel "${JOBS}")
  log_info "building bundled MuJoCo"
  cmake "${args[@]}"
}

install_mujoco() {
  check_build_tree
  log_info "installing bundled MuJoCo into staging prefix"
  cmake --install "${BUILD_DIR}"
  validate_install
}

show_version() {
  check_source_tree
  local source_version installed_version="not installed" status="matched"
  source_version="$(read_source_version)" || die "could not determine bundled MuJoCo source version."
  if [[ -d "${INSTALL_DIR}" ]]; then
    installed_version="$(read_installed_version)" || installed_version="invalid"
  fi
  if [[ "${source_version}" != "${EXPECTED_VERSION}" || "${installed_version}" != "${EXPECTED_VERSION}" ]]; then status="stale"; fi
  printf 'Bundled MuJoCo\n\nExpected:   %s\nSource:     %s\nInstalled:  %s\nStatus:     %s\n' "${EXPECTED_VERSION}" "${source_version}" "${installed_version}" "${status}"
  [[ "${status}" == "matched" ]] || { log_error "Run: ./scripts/mujoco.sh rebuild"; return 1; }
}

show_status() {
  local configured="no" built="no" installed="no"
  [[ -f "${BUILD_DIR}/CMakeCache.txt" ]] && configured="yes"
  compgen -G "${BUILD_DIR}/lib*/libmujoco.so*" >/dev/null && built="yes"
  if [[ -f "${INSTALL_DIR}/include/mujoco/mujoco.h" ]] && compgen -G "${INSTALL_DIR}/lib/libmujoco.so*" >/dev/null; then installed="yes"; fi
  printf 'Bundled MuJoCo status\n\nSource:       %s\nConfigured:   %s\nBuilt:        %s\nInstalled:    %s\n\nSource:\n  %s\n\nBuild:\n  %s\n\nInstall:\n  %s\n' "$([[ -f "${MUJOCO_DIR}/CMakeLists.txt" ]] && echo ready || echo missing)" "${configured}" "${built}" "${installed}" "${MUJOCO_DIR}" "${BUILD_DIR}" "${INSTALL_DIR}"
}

clean_mujoco() { safe_remove_tree "${BUILD_DIR}"; }
purge_mujoco() { safe_remove_tree "${BUILD_DIR}"; safe_remove_tree "${INSTALL_DIR}"; safe_remove_tree "${LEGACY_INSTALL_DIR}"; }
rebuild_mujoco() { purge_mujoco; configure_mujoco; build_mujoco; install_mujoco; }

print_usage() {
  cat <<'EOF'
Usage: scripts/mujoco.sh <command> [options]

Commands: build, configure, install, version, status, clean, purge, rebuild
Options: --build-type <Release|Debug|RelWithDebInfo>, -j <jobs>, --jobs <jobs>, --generator <name>
EOF
}

parse_arguments() {
  COMMAND="${1:-}"; [[ -n "${COMMAND}" ]] || { print_usage; exit 2; }; shift || true
  while (($#)); do
    case "$1" in
      --build-type) BUILD_TYPE="${2:-}"; shift 2 ;;
      -j|--jobs) JOBS="${2:-}"; shift 2 ;;
      --generator) GENERATOR="${2:-}"; shift 2 ;;
      -h|--help) print_usage; exit 0 ;;
      *) die "unknown argument: $1" ;;
    esac
  done
  [[ "${BUILD_TYPE}" =~ ^(Release|Debug|RelWithDebInfo)$ ]] || die "invalid build type: ${BUILD_TYPE}"
  if [[ -z "${JOBS}" ]]; then JOBS="$(command -v nproc >/dev/null 2>&1 && nproc || echo 2)"; fi
  [[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || die "jobs must be a positive integer: ${JOBS}"
}

main() {
  parse_arguments "$@"
  case "${COMMAND}" in
    configure) configure_mujoco ;;
    build) configure_mujoco; build_mujoco; install_mujoco ;;
    install) install_mujoco ;;
    version) show_version ;;
    status) show_status ;;
    clean) clean_mujoco ;;
    purge) purge_mujoco ;;
    rebuild) rebuild_mujoco ;;
    *) die "unknown command: ${COMMAND}. Run scripts/mujoco.sh --help." ;;
  esac
}

main "$@"
