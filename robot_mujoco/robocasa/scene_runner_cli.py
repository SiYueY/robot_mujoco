"""Unified RoboCasa scene generation and launch entry point."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys

from robot_mujoco.runtime import default_fr3_model_path, launch_viewer_simulation

from .cli_options import TASK_CHOICES, choose_option, choose_randomized_scene_ids
from .exceptions import RoboCasaIntegrationError
from .scene_config import config_from_mapping


DEFAULT_ROBOT_XML_SENTINELS = {"", "/path/to/robot.xml", "__DEFAULT_FR3_XML__"}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="RoboCasa YAML config path.")
    parser.add_argument("--output", required=True, help="Generated MJCF XML path.")
    parser.add_argument("--task", help="Override RoboCasa task name.")
    parser.add_argument("--layout", type=int, help="Override RoboCasa layout id.")
    parser.add_argument("--style", type=int, help="Override RoboCasa style id.")
    parser.add_argument(
        "--no-launch",
        action="store_true",
        help="Only generate the combined MJCF and skip viewer launch.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        config = _load_scene_config(args.config)
        task_name = args.task
        if task_name is None:
            task_name = choose_option(
                TASK_CHOICES,
                "task",
                show_keys=True,
                default=config.task_name,
                default_message=config.task_name,
            )

        layout_id, style_id = choose_randomized_scene_ids(
            layout=args.layout,
            style=args.style,
        )

        print(
            "Selected RoboCasa scene:",
            f"task={task_name}, layout={layout_id}, style={style_id}",
        )

        config = replace(
            config,
            task_name=task_name,
            layout_id=layout_id,
            style_id=style_id,
            write_generated_xml=True,
            generated_xml_path=str(Path(args.output).expanduser().resolve()),
        )
        config.validate()

        from .scene_generator import SceneGenerator

        scene = SceneGenerator().generate(config)
        output_path = Path(config.generated_xml_path).expanduser().resolve()
        if not output_path.exists():
            output_path.write_text(scene.xml, encoding="utf-8")

        print(output_path)

        if args.no_launch:
            return 0

        return launch_viewer_simulation(mujoco_model_path=output_path)
    except RoboCasaIntegrationError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except Exception as exc:  # noqa: BLE001
        print(f"Unexpected robot_mujoco RoboCasa scene failure: {exc}", file=sys.stderr)
        return 1


def _load_scene_config(path: str | Path):
    try:
        import yaml
    except ImportError as exc:
        raise RoboCasaIntegrationError(
            "PyYAML is required to load RoboCasa scene config. Install it with: pip install pyyaml"
        ) from exc

    config_path = Path(path).expanduser().resolve()
    if not config_path.exists():
        raise RoboCasaIntegrationError(f"Config file does not exist: {config_path}")

    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001
        raise RoboCasaIntegrationError(
            f"Failed to read RoboCasa scene config: {config_path}"
        ) from exc

    if not isinstance(raw, dict):
        raise RoboCasaIntegrationError(
            f"RoboCasa scene config must be a mapping: {config_path}"
        )

    robot = raw.get("robot")
    if robot is None:
        robot = {}
        raw["robot"] = robot
    if not isinstance(robot, dict):
        raise RoboCasaIntegrationError("robot section must be a mapping.")

    robot_xml_path = str(robot.get("xml_path", "")).strip()
    if robot_xml_path in DEFAULT_ROBOT_XML_SENTINELS:
        robot["xml_path"] = str(default_fr3_model_path().resolve())

    if not str(robot.get("root_body_name", "")).strip():
        robot["root_body_name"] = "base"

    return config_from_mapping(raw, base_dir=config_path.parent)


if __name__ == "__main__":
    raise SystemExit(main())
