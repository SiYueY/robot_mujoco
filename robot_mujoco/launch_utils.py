"""Shared helpers for robot_mujoco launch entry points."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import tempfile
from typing import Any

from robot_mujoco.robocasa.scene_config import config_from_mapping

DEFAULT_ROBOT_XML_SENTINELS = {"", "/path/to/robot.xml", "__DEFAULT_FR3_XML__"}


def repo_root() -> Path:
    """Return the repository root."""

    return Path(__file__).resolve().parent.parent


def package_share_dir() -> Path:
    """Return the installed robot_mujoco share directory when available."""

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return repo_root()

    try:
        return Path(get_package_share_directory("robot_mujoco"))
    except Exception:
        return repo_root()


def package_file(*parts: str) -> Path:
    """Return a package resource path from install or source."""

    candidates = [
        package_share_dir().joinpath(*parts),
        repo_root().joinpath(*parts),
    ]
    return first_existing_path(candidates)


def first_existing_path(candidates: list[Path]) -> Path:
    """Return the first existing path, or the first candidate if none exist yet."""

    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if resolved.exists():
            return resolved
    return candidates[0].expanduser().resolve()


def parse_bool(value: Any) -> bool:
    """Return a launch-compatible boolean from strings or python values."""

    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def load_robot_description(
    mujoco_model_path: str | Path,
    robot_description_template: str | Path,
    initial_keyframe: str,
    render_mode: str,
) -> str:
    """Load and fill one robot description template."""

    template_path = Path(robot_description_template).expanduser().resolve()
    robot_description = template_path.read_text(encoding="utf-8")
    robot_description = robot_description.replace(
        "__MUJOCO_MODEL_PATH__", str(Path(mujoco_model_path).expanduser().resolve())
    )
    robot_description = robot_description.replace("__RENDER_MODE__", render_mode)
    if initial_keyframe:
        robot_description = robot_description.replace("__INITIAL_KEYFRAME__", initial_keyframe)
    else:
        lines = robot_description.splitlines()
        filtered_lines = [
            line for line in lines if "__INITIAL_KEYFRAME__" not in line
        ]
        robot_description = "\n".join(filtered_lines)
        if robot_description and not robot_description.endswith("\n"):
            robot_description += "\n"
    return robot_description


def default_franka_fr3_model_path() -> Path:
    """Return the default FR3 robot XML path used by RoboCasa composition."""

    return package_file("model", "franka_fr3", "franka_fr3.xml")


def generate_robocasa_scene(
    *,
    config_path: str | Path,
    output_path: str | Path | None = None,
    task_name: str = "",
    layout_id: str | int | None = None,
    style_id: str | int | None = None,
) -> Path:
    """Generate a RoboCasa scene and return the XML path."""

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "PyYAML is required to load RoboCasa scene config. Install it with: pip install pyyaml"
        ) from exc

    resolved_config_path = Path(config_path).expanduser().resolve()
    if not resolved_config_path.exists():
        raise RuntimeError(f"RoboCasa scene config does not exist: {resolved_config_path}")
    if not resolved_config_path.is_file():
        raise RuntimeError(f"RoboCasa scene config is not a file: {resolved_config_path}")

    try:
        raw = yaml.safe_load(resolved_config_path.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            f"Failed to read RoboCasa scene config: {resolved_config_path}"
        ) from exc

    if not isinstance(raw, dict):
        raise RuntimeError(
            f"RoboCasa scene config must be a mapping: {resolved_config_path}"
        )

    robocasa_section = raw.setdefault("robocasa", {})
    robot_section = raw.setdefault("robot", {})
    output_section = raw.setdefault("output", {})

    if not isinstance(robocasa_section, dict):
        raise RuntimeError("robocasa section must be a mapping.")
    if not isinstance(robot_section, dict):
        raise RuntimeError("robot section must be a mapping.")
    if not isinstance(output_section, dict):
        raise RuntimeError("output section must be a mapping.")

    robot_xml_path = str(robot_section.get("xml_path", "")).strip()
    if robot_xml_path in DEFAULT_ROBOT_XML_SENTINELS:
        robot_section["xml_path"] = str(default_franka_fr3_model_path().resolve())

    if not str(robot_section.get("root_body_name", "")).strip():
        robot_section["root_body_name"] = "base"

    if task_name:
        robocasa_section["task_name"] = task_name
    if layout_id not in (None, ""):
        robocasa_section["layout_id"] = int(layout_id)
    if style_id not in (None, ""):
        robocasa_section["style_id"] = int(style_id)

    if output_path is None or not str(output_path).strip():
        output_path = Path(tempfile.gettempdir()) / "robot_mujoco_robocasa_franka_fr3.xml"
    else:
        output_path = Path(output_path).expanduser().resolve()

    output_section["write_generated_xml"] = True
    output_section["generated_xml_path"] = str(output_path)

    config = config_from_mapping(raw, base_dir=resolved_config_path.parent)
    config = replace(
        config,
        write_generated_xml=True,
        generated_xml_path=str(output_path),
    )
    config.validate()

    from robot_mujoco.robocasa.scene_generator import SceneGenerator

    scene = SceneGenerator().generate(config)
    resolved_output_path = Path(config.generated_xml_path).expanduser().resolve()
    if not resolved_output_path.exists():
        resolved_output_path.write_text(scene.xml, encoding="utf-8")
    return resolved_output_path
