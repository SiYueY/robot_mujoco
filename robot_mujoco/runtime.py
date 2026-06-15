"""Runtime helpers for launching robot_mujoco viewer simulations."""

from __future__ import annotations

from pathlib import Path
import subprocess


def repo_root() -> Path:
    """Return the repository root."""

    return Path(__file__).resolve().parent.parent


def default_fr3_model_path() -> Path:
    """Return the default FR3 MJCF path used for RoboCasa composition."""

    candidates = []

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        get_package_share_directory = None

    if get_package_share_directory is not None:
        try:
            candidates.append(
                Path(get_package_share_directory("franka_fr3_mujoco")) / "model" / "fr3.xml"
            )
        except Exception:
            pass

    candidates.extend(
        [
            repo_root() / "example" / "franka_fr3_mujoco" / "model" / "fr3.xml",
            Path.cwd() / "example" / "franka_fr3_mujoco" / "model" / "fr3.xml",
        ]
    )

    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if resolved.exists():
            return resolved

    return candidates[0].expanduser().resolve()


def package_share_dir() -> Path:
    """Return the installed robot_mujoco share directory when available."""

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return repo_root() / "robot_mujoco"

    try:
        return Path(get_package_share_directory("robot_mujoco"))
    except Exception:
        return repo_root() / "robot_mujoco"


def default_robot_description_template() -> Path:
    """Return the default FR3 robot description template path."""

    share_dir = package_share_dir()
    candidates = [
        share_dir / "config" / "fr3.urdf.template",
        share_dir / "fr3.urdf.template",
        repo_root() / "robot_mujoco" / "config" / "fr3.urdf.template",
    ]
    return _first_existing_path(candidates)


def default_controllers_yaml() -> Path:
    """Return the default controller config path."""

    share_dir = package_share_dir()
    candidates = [
        share_dir / "config" / "fr3_controllers.yaml",
        share_dir / "fr3_controllers.yaml",
        repo_root() / "robot_mujoco" / "config" / "fr3_controllers.yaml",
    ]
    return _first_existing_path(candidates)


def _first_existing_path(candidates: list[Path]) -> Path:
    """Return the first existing path, or the first candidate if none exist yet."""

    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if resolved.exists():
            return resolved
    return candidates[0].expanduser().resolve()


def launch_viewer_simulation(
    *,
    mujoco_model_path: str | Path,
    robot_description_template: str | Path | None = None,
    controllers_yaml: str | Path | None = None,
    initial_keyframe: str = "",
) -> int:
    """Launch the ROS 2 MuJoCo viewer pipeline and return its exit code."""

    template_path = Path(
        robot_description_template
        if robot_description_template is not None
        else default_robot_description_template()
    ).expanduser().resolve()
    controllers_path = Path(
        controllers_yaml if controllers_yaml is not None else default_controllers_yaml()
    ).expanduser().resolve()
    model_path = Path(mujoco_model_path).expanduser().resolve()
    launch_path = default_launch_file()

    command = [
        "ros2",
        "launch",
        str(launch_path),
        f"mujoco_model_path:={model_path}",
        f"robot_description_template:={template_path}",
        f"controllers_yaml:={controllers_path}",
    ]

    if initial_keyframe:
        command.append(f"initial_keyframe:={initial_keyframe}")

    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


def default_launch_file() -> Path:
    """Return the installed or source launch file path."""

    share_dir = package_share_dir()
    candidates = [
        share_dir / "launch" / "sim.launch.py",
        share_dir / "sim.launch.py",
        repo_root() / "robot_mujoco" / "launch" / "sim.launch.py",
    ]
    return _first_existing_path(candidates)
