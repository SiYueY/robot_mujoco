"""CLI for generating MuJoCo-ready RoboCasa scenes."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys

from .cli_options import TASK_CHOICES, choose_option, get_layout_choices, get_style_choices
from .exceptions import RoboCasaIntegrationError
from .scene_config import load_config


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="Base RoboCasa YAML config path.")
    parser.add_argument("--output", required=True, help="Output MJCF XML path.")
    parser.add_argument("--task", help="RoboCasa task name.")
    parser.add_argument("--layout", type=int, help="RoboCasa layout id.")
    parser.add_argument("--style", type=int, help="RoboCasa style id.")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Prompt for missing task/layout/style values in the terminal.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the CLI."""

    args = build_parser().parse_args(argv)

    try:
        config = load_config(args.config)
        interactive = args.interactive or any(
            value is None for value in (args.task, args.layout, args.style)
        )

        task = args.task
        layout = args.layout
        style = args.style
        if interactive:
            task = task or choose_option(TASK_CHOICES, "task")
            layout_choices = get_layout_choices()
            layout = layout if layout is not None else choose_option(layout_choices, "layout")
            style = style if style is not None else choose_option(get_style_choices(), "style")

        if task is None or layout is None or style is None:
            raise RoboCasaIntegrationError(
                "task, layout, and style must all be specified when not using interactive mode."
            )

        config = replace(
            config,
            task_name=task,
            layout_id=layout,
            style_id=style,
            write_generated_xml=True,
            generated_xml_path=str(Path(args.output).expanduser().resolve()),
        )

        from .scene_generator import SceneGenerator

        scene = SceneGenerator().generate(config)
        output_path = Path(config.generated_xml_path).expanduser().resolve()
        if not output_path.exists():
            output_path.write_text(scene.xml, encoding="utf-8")

        print(output_path)
        return 0
    except RoboCasaIntegrationError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except Exception as exc:  # noqa: BLE001
        print(f"Unexpected RoboCasa scene generation failure: {exc}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    raise SystemExit(main())
