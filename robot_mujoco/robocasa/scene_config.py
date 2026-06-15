"""Configuration loading for RoboCasa scene generation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .exceptions import RoboCasaSceneConfigError


@dataclass(frozen=True)
class SceneConfig:
    """Configuration for generating one RoboCasa-based MuJoCo scene.

    Attributes:
        task_name: RoboCasa task name, for example ``PickPlaceCounterToCabinet``.
        layout_id: RoboCasa layout id.
        style_id: RoboCasa style id.
        robot_xml_path: Project robot MJCF XML path.
        robot_root_body_name: Root body name of the project robot in MJCF.
        placeholder_robot: Temporary RoboCasa / robosuite robot used to generate the
            kitchen scene before replacement. The default remains PandaMobile because
            RoboCasa examples are still built around Panda-based placeholder robots.
        placeholder_robot_body_name: Root body name of the placeholder robot that
            should be removed from the RoboCasa MJCF.
        controller: robosuite controller name used for the placeholder robot.
        control_freq: robosuite control frequency used during environment creation.
        write_generated_xml: Whether to save the final adapted MJCF XML.
        generated_xml_path: Output path for the final adapted MJCF XML.
    """

    task_name: str = "PickPlaceCounterToCabinet"
    layout_id: int = 2
    style_id: int = 2
    robot_xml_path: str = ""
    robot_root_body_name: str = "robot_base"
    placeholder_robot: str = "PandaMobile"
    placeholder_robot_body_name: str = "robot0_base"
    controller: str = "OSC_POSE"
    control_freq: int = 20
    write_generated_xml: bool = False
    generated_xml_path: str | None = None

    def validate(self) -> None:
        """Validate this configuration.

        Raises:
            RoboCasaSceneConfigError: If a required field is missing or invalid.
        """

        if not self.task_name:
            raise RoboCasaSceneConfigError("RoboCasa task_name must not be empty.")

        if self.layout_id < 0:
            raise RoboCasaSceneConfigError("RoboCasa layout_id must be non-negative.")

        if self.style_id < 0:
            raise RoboCasaSceneConfigError("RoboCasa style_id must be non-negative.")

        if not self.robot_xml_path:
            raise RoboCasaSceneConfigError("robot_xml_path must not be empty.")

        if not Path(self.robot_xml_path).expanduser().exists():
            raise RoboCasaSceneConfigError(
                f"robot_xml_path does not exist: {self.robot_xml_path}"
            )

        if not self.robot_root_body_name:
            raise RoboCasaSceneConfigError("robot_root_body_name must not be empty.")

        if not self.placeholder_robot:
            raise RoboCasaSceneConfigError("placeholder_robot must not be empty.")

        if not self.placeholder_robot_body_name:
            raise RoboCasaSceneConfigError(
                "placeholder_robot_body_name must not be empty."
            )

        if self.control_freq <= 0:
            raise RoboCasaSceneConfigError("control_freq must be greater than zero.")

        if self.write_generated_xml and not self.generated_xml_path:
            raise RoboCasaSceneConfigError(
                "generated_xml_path must be set when write_generated_xml is true."
            )


def load_config(path: str | Path) -> SceneConfig:
    """Load a :class:`SceneConfig` from a YAML file.

    Expected YAML shape::

        robocasa:
          task_name: PickPlaceCounterToCabinet
          layout_id: 2
          style_id: 2
          placeholder_robot: PandaMobile
          placeholder_robot_body_name: robot0_base
          controller: OSC_POSE
          control_freq: 20

        robot:
          xml_path: robot.xml
          root_body_name: robot_base

        output:
          write_generated_xml: true
          generated_xml_path: /tmp/robocasa_generated_scene.xml

    Raises:
        RoboCasaSceneConfigError: If PyYAML is unavailable, the YAML is malformed,
            or the resulting configuration is invalid.
    """

    try:
        import yaml
    except ImportError as exc:
        raise RoboCasaSceneConfigError(
            "PyYAML is required to load RoboCasa scene YAML config. "
            "Install it with: pip install pyyaml"
        ) from exc

    config_path = Path(path).expanduser()
    if not config_path.exists():
        raise RoboCasaSceneConfigError(f"Config file does not exist: {config_path}")

    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001
        raise RoboCasaSceneConfigError(
            f"Failed to read RoboCasa scene config: {config_path}"
        ) from exc

    if not isinstance(raw, dict):
        raise RoboCasaSceneConfigError(
            f"RoboCasa scene config must be a mapping: {config_path}"
        )

    config = config_from_mapping(raw, base_dir=config_path.parent)
    config.validate()
    return config


def config_from_mapping(
    data: dict[str, Any],
    *,
    base_dir: str | Path | None = None,
) -> SceneConfig:
    """Create :class:`SceneConfig` from a nested mapping.

    Args:
        data: Nested mapping loaded from YAML or another source.
        base_dir: Base directory used to resolve relative ``robot.xml_path`` and
            ``output.generated_xml_path`` values.

    Returns:
        Parsed scene configuration.
    """

    robocasa = _as_mapping(data.get("robocasa", {}), "robocasa")
    robot = _as_mapping(data.get("robot", {}), "robot")
    output = _as_mapping(data.get("output", {}), "output")

    robot_xml_path = str(robot.get("xml_path", ""))
    generated_xml_path = output.get("generated_xml_path")

    if base_dir is not None:
        base = Path(base_dir)
        if robot_xml_path and not Path(robot_xml_path).expanduser().is_absolute():
            robot_xml_path = str((base / robot_xml_path).resolve())

        if generated_xml_path and not Path(str(generated_xml_path)).expanduser().is_absolute():
            generated_xml_path = str((base / str(generated_xml_path)).resolve())

    return SceneConfig(
        task_name=str(robocasa.get("task_name", "PickPlaceCounterToCabinet")),
        layout_id=_as_int(robocasa.get("layout_id", 2), "robocasa.layout_id"),
        style_id=_as_int(robocasa.get("style_id", 2), "robocasa.style_id"),
        robot_xml_path=robot_xml_path,
        robot_root_body_name=str(robot.get("root_body_name", "robot_base")),
        placeholder_robot=str(robocasa.get("placeholder_robot", "PandaMobile")),
        placeholder_robot_body_name=str(
            robocasa.get("placeholder_robot_body_name", "robot0_base")
        ),
        controller=str(robocasa.get("controller", "OSC_POSE")),
        control_freq=_as_int(robocasa.get("control_freq", 20), "robocasa.control_freq"),
        write_generated_xml=bool(output.get("write_generated_xml", False)),
        generated_xml_path=(
            str(generated_xml_path) if generated_xml_path is not None else None
        ),
    )


def _as_mapping(value: Any, name: str) -> dict[str, Any]:
    """Validate that a value is a mapping and return it as a dictionary."""

    if value is None:
        return {}

    if not isinstance(value, dict):
        raise RoboCasaSceneConfigError(f"{name} must be a mapping.")

    return value


def _as_int(value: Any, name: str) -> int:
    """Convert a config value to int and raise a project config error."""

    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise RoboCasaSceneConfigError(f"{name} must be an integer.") from exc
