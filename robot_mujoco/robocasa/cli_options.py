"""Shared CLI option helpers for RoboCasa scene tools."""

from __future__ import annotations

from collections import OrderedDict
import random
from typing import Callable

from .exceptions import RoboCasaIntegrationError


TASK_CHOICES = OrderedDict(
    [
        ("PickPlaceCounterToCabinet", "Pick and place from counter to cabinet"),
        ("OpenDrawer", "Open drawer"),
        ("CloseDrawer", "Close drawer"),
        ("OpenFridgeDrawer", "Open fridge drawer"),
        ("CloseFridgeDrawer", "Close fridge drawer"),
        ("TurnOnMicrowave", "Turn on microwave"),
        ("TurnOffMicrowave", "Turn off microwave"),
        ("TurnOnSinkFaucet", "Turn on sink faucet"),
        ("TurnOffSinkFaucet", "Turn off sink faucet"),
        ("TurnOnStove", "Turn on stove"),
        ("TurnOffStove", "Turn off stove"),
        ("TurnOnElectricKettle", "Turn on electric kettle"),
    ]
)


def get_layout_choices() -> OrderedDict[int, str]:
    """Return available RoboCasa layout ids."""

    try:
        from robocasa.models.scenes.scene_registry import LayoutType
    except ImportError as exc:
        raise RoboCasaIntegrationError(
            "robocasa is required to enumerate layout choices."
        ) from exc

    choices = OrderedDict()
    for item in sorted(LayoutType, key=lambda entry: entry.value):
        if item.value < 0:
            continue
        choices[item.value] = item.name.lower().capitalize()
    return choices


def get_style_choices() -> OrderedDict[int, str]:
    """Return available RoboCasa style ids."""

    try:
        from robocasa.models.scenes.scene_registry import StyleType
    except ImportError as exc:
        raise RoboCasaIntegrationError(
            "robocasa is required to enumerate style choices."
        ) from exc

    choices = OrderedDict()
    for item in sorted(StyleType, key=lambda entry: entry.value):
        if item.value <= 0:
            continue
        choices[item.value] = item.name.title().replace("Style", "Style ")
    return choices


def choose_option(
    options: OrderedDict[object, str],
    option_name: str,
    *,
    show_keys: bool = False,
    default: object | None = None,
    default_message: str | None = None,
    input_func: Callable[[str], str] = input,
    output_func: Callable[[str], None] = print,
) -> object:
    """Prompt for one option and return its selected key.

    Menus use 1-based numbering. Empty input or `0` falls back to the default.
    """

    items = list(options.items())
    if not items:
        raise RoboCasaIntegrationError(f"No {option_name} choices are available.")

    if default is None:
        default = items[0][0]
    if default_message is None:
        default_message = str(default)

    output_func(f"{option_name.capitalize()}s:")
    for index, (key, label) in enumerate(items, start=1):
        if show_keys:
            output_func(f"[{index}] {key}: {label}")
        else:
            output_func(f"[{index}] {label}")
    output_func("")

    prompt = (
        f"Choose {option_name} 1-{len(items)}, or 0/Enter for default "
        f"({default_message}): "
    )
    while True:
        raw = input_func(prompt).strip()
        if raw == "" or raw == "0":
            output_func(f"Use {default_message}.\n")
            return default
        try:
            choice = int(raw)
        except ValueError:
            output_func("Invalid input. Enter a number.")
            continue
        if 1 <= choice <= len(items):
            return items[choice - 1][0]
        output_func("Choice out of range.")


def choose_randomized_scene_ids(
    *,
    layout: int | None,
    style: int | None,
    input_func: Callable[[str], str] = input,
    output_func: Callable[[str], None] = print,
) -> tuple[int, int]:
    """Resolve layout/style ids, prompting when values are missing."""

    layouts = get_layout_choices()
    styles = get_style_choices()

    selected_layout = layout
    if selected_layout is None:
        selected_layout = choose_option(
            layouts,
            "kitchen layout",
            default=-1,
            default_message="random layouts",
            input_func=input_func,
            output_func=output_func,
        )

    selected_style = style
    if selected_style is None:
        selected_style = choose_option(
            styles,
            "kitchen style",
            default=-1,
            default_message="random styles",
            input_func=input_func,
            output_func=output_func,
        )

    if selected_layout == -1:
        selected_layout = random.choice(list(layouts.keys()))
    if selected_style == -1:
        selected_style = random.choice(list(styles.keys()))

    return int(selected_layout), int(selected_style)
