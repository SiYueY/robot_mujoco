"""Runtime helpers for launching robot_mujoco viewer simulations."""

from __future__ import annotations

from pathlib import Path
import subprocess

from robot_mujoco.launch_utils import package_share_dir

FRANKA_FR3_PRIMARY_CONTROLLER = "arm_position_controller"


def repo_root() -> Path:
    """Return the repository root."""

    return Path(__file__).resolve().parent.parent


def default_fr3_model_path() -> Path:
    """Return the default FR3 MJCF path used for RoboCasa composition."""

    share_dir = package_share_dir()
    candidates = [
        share_dir / "model" / "franka_fr3" / "franka_fr3.xml",
        repo_root() / "robot_mujoco" / "model" / "franka_fr3" / "franka_fr3.xml",
    ]
    return _first_existing_path(candidates)


def default_robot_description_template() -> Path:
    """Return the default FR3 robot description template path."""

    share_dir = package_share_dir()
    candidates = [
        share_dir / "config" / "franka_fr3" / "franka_fr3.urdf.template",
        repo_root() / "robot_mujoco" / "config" / "franka_fr3" / "franka_fr3.urdf.template",
    ]
    return _first_existing_path(candidates)


def default_controllers_yaml() -> Path:
    """Return the default controller config path."""

    share_dir = package_share_dir()
    candidates = [
        share_dir / "config" / "franka_fr3" / "controllers.yaml",
        repo_root() / "robot_mujoco" / "config" / "franka_fr3" / "controllers.yaml",
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
    robot_name: str = "franka_fr3",
    primary_controller: str = FRANKA_FR3_PRIMARY_CONTROLLER,
    use_rviz: bool = False,
    rviz_config: str | Path | None = None,
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
        f"robot_name:={robot_name}",
        f"mujoco_model_path:={model_path}",
        f"robot_description_template:={template_path}",
        f"controllers_yaml:={controllers_path}",
        f"primary_controller:={primary_controller}",
        f"use_rviz:={'true' if use_rviz else 'false'}",
        "use_robocasa_scene:=false",
    ]

    if initial_keyframe:
        command.append(f"initial_keyframe:={initial_keyframe}")
    if rviz_config is not None:
        command.append(f"rviz_config:={Path(rviz_config).expanduser().resolve()}")

    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


def default_launch_file() -> Path:
    """Return the installed or source launch file path."""

    share_dir = package_share_dir()
    candidates = [
        share_dir / "launch" / "robot_mujoco.launch.py",
        repo_root() / "robot_mujoco" / "launch" / "robot_mujoco.launch.py",
    ]
    return _first_existing_path(candidates)
