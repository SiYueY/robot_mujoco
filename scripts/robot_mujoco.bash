#!/usr/bin/env bash

# Source ROS 2, the local virtualenv, and the colcon workspace in a consistent
# order so `ros2 launch` can see Python packages installed only in `.venv`.

_robot_mujoco_return_or_exit() {
  return "$1" 2>/dev/null || exit "$1"
}

_robot_mujoco_prepend_pythonpath() {
  if [ -z "$1" ] || [ ! -e "$1" ]; then
    return 0
  fi

  case ":${PYTHONPATH:-}:" in
    *":$1:"*) ;;
    *) export PYTHONPATH="$1${PYTHONPATH:+:$PYTHONPATH}" ;;
  esac
}

_robot_mujoco_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_robot_mujoco_root="$(cd "$_robot_mujoco_script_dir/.." && pwd)"
export ROBOT_MUJOCO_ROOT="$_robot_mujoco_root"
export PYTHONNOUSERSITE=1

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "robot_mujoco.bash: missing /opt/ros/humble/setup.bash" >&2
  _robot_mujoco_return_or_exit 1
fi
source /opt/ros/humble/setup.bash

if [ -f "$_robot_mujoco_root/.venv/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "$_robot_mujoco_root/.venv/bin/activate"
else
  echo "robot_mujoco.bash: warning: .venv not found at $_robot_mujoco_root/.venv" >&2
fi

if [ -f "$_robot_mujoco_root/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$_robot_mujoco_root/install/setup.bash"
else
  echo "robot_mujoco.bash: warning: install/setup.bash not found; run colcon build first" >&2
fi

if command -v python >/dev/null 2>&1; then
  _robot_mujoco_site_packages="$(
    python - <<'PY'
import site

paths = []
try:
    paths.extend(site.getsitepackages())
except Exception:
    pass

seen = set()
for path in paths:
    if path and "site-packages" in path and path not in seen:
        print(path)
        seen.add(path)
PY
  )"

  if [ -n "$_robot_mujoco_site_packages" ]; then
    while IFS= read -r _robot_mujoco_path; do
      _robot_mujoco_prepend_pythonpath "$_robot_mujoco_path"
    done <<< "$_robot_mujoco_site_packages"
  fi
fi

# Editable installs in `.venv` are often represented via `.pth` files inside
# site-packages. `/usr/bin/python3` does not resolve those when the directory is
# only injected through `PYTHONPATH`, so add the vendored source roots directly.
_robot_mujoco_prepend_pythonpath "$_robot_mujoco_root/third_party/robosuite"
_robot_mujoco_prepend_pythonpath "$_robot_mujoco_root/third_party/robocasa"

unset _robot_mujoco_path
unset _robot_mujoco_script_dir
unset _robot_mujoco_site_packages
unset _robot_mujoco_root
