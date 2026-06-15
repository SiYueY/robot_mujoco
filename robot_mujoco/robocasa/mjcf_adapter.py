"""Adapt RoboCasa-generated MJCF for MuJoCo scenes."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

from .exceptions import RoboCasaMjcfAdaptationError
from .scene_data import ObjectPlacement, SpawnPose


RUNTIME_ELEMENT_TAGS = ("actuator", "sensor", "option")
TRANSPARENT_RGBA = "1 1 1 0"
DEBUG_MARKER_RGBA_VALUES = {
    "1 0 0 0.5",
    "0 0 1 0.5",
    "1 0 0 1",
    "0 0 1 1",
    "1 0 0 0.3",
    "0 0 1 0.3",
}


def adapt_mjcf(
    xml: str,
    *,
    robot_xml_path: str | Path,
    placeholder_body_name: str = "robot0_base",
    object_placements: Iterable[ObjectPlacement] = (),
    spawn_pose: SpawnPose | None = None,
    robot_body_name: str | None = None,
    require_placeholder_robot: bool = True,
    add_floor: bool = True,
) -> str:
    """Adapt RoboCasa-generated MJCF for the project robot.

    Args:
        xml: Raw MJCF XML string returned by RoboCasa / robosuite.
        robot_xml_path: Project robot MJCF XML path inserted as ``<include>``.
        placeholder_body_name: Root body of the RoboCasa placeholder robot.
        object_placements: Object placements to write into RoboCasa object bodies.
        spawn_pose: Project robot root pose to write into the final XML.
        robot_body_name: Root body receiving ``spawn_pose``.
        require_placeholder_robot: Whether missing placeholder robot is an error.
        add_floor: Whether to ensure a basic floor geom exists.

    Returns:
        Adapted MJCF XML string.

    Raises:
        RoboCasaMjcfAdaptationError: If the XML is invalid or adaptation fails.
    """

    root = _parse_mjcf(xml)
    _remove_top_level_runtime_elements(root)

    removed_robot = _remove_body_by_name(root, placeholder_body_name)
    if require_placeholder_robot and removed_robot is None:
        raise RoboCasaMjcfAdaptationError(
            f"Placeholder robot body not found: {placeholder_body_name}"
        )

    _hide_debug_markers(root)
    _apply_object_placements(root, object_placements)

    if add_floor:
        _ensure_floor_geom(root)

    _merge_robot_mjcf(root, robot_xml_path)
    _apply_spawn_pose(root, spawn_pose, robot_body_name)

    return _serialize_mjcf(root)


def _parse_mjcf(xml: str) -> ET.Element:
    """Parse MJCF XML text and return the root element."""

    try:
        root = ET.fromstring(xml)
    except ET.ParseError as exc:
        raise RoboCasaMjcfAdaptationError("Failed to parse RoboCasa MJCF XML.") from exc

    if root.tag != "mujoco":
        raise RoboCasaMjcfAdaptationError(
            f"Expected MJCF root tag 'mujoco', got: {root.tag}"
        )

    return root


def _serialize_mjcf(root: ET.Element) -> str:
    """Serialize an MJCF XML tree to text."""

    return ET.tostring(root, encoding="unicode")


def _remove_top_level_runtime_elements(root: ET.Element) -> None:
    """Remove top-level runtime elements that should not be reused.

    RoboCasa / robosuite may generate actuators, sensors, and options that are
    tied to the placeholder robot or its controller. The project simulator should
    define its own actuators and sensors through its robot MJCF or runtime layer.
    """

    for tag in RUNTIME_ELEMENT_TAGS:
        for child in list(root):
            if child.tag == tag:
                root.remove(child)


def _merge_robot_mjcf(root: ET.Element, robot_xml_path: str | Path) -> None:
    """Merge a project robot MJCF into the generated RoboCasa MJCF tree."""

    path = Path(robot_xml_path).expanduser().resolve()
    if not path.exists():
        raise RoboCasaMjcfAdaptationError(f"Robot MJCF does not exist: {path}")

    robot_root = _parse_mjcf(path.read_text(encoding="utf-8"))
    _merge_compiler(root, robot_root)
    _merge_top_level_children(root, robot_root, "default")
    _merge_asset_block(root, robot_root, path.parent)
    _merge_top_level_children(root, robot_root, "option")
    _merge_top_level_children(root, robot_root, "sensor")
    _merge_top_level_children(root, robot_root, "actuator")
    _merge_top_level_children(root, robot_root, "contact")
    _merge_top_level_children(root, robot_root, "equality")
    _merge_top_level_children(root, robot_root, "tendon")
    _merge_worldbody(root, robot_root)


def _merge_compiler(root: ET.Element, robot_root: ET.Element) -> None:
    """Merge non-path compiler attributes from the robot MJCF."""

    robot_compiler = robot_root.find("compiler")
    if robot_compiler is None:
        return

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)

    for key, value in robot_compiler.attrib.items():
        if key in {"meshdir", "texturedir", "assetdir"}:
            continue
        if key not in compiler.attrib:
            compiler.set(key, value)


def _merge_top_level_children(root: ET.Element, robot_root: ET.Element, tag: str) -> None:
    """Append deep-copied top-level MJCF children of one tag."""

    for child in robot_root.findall(tag):
        root.append(copy.deepcopy(child))


def _merge_asset_block(root: ET.Element, robot_root: ET.Element, robot_dir: Path) -> None:
    """Merge the robot asset block and rewrite file paths to absolute paths."""

    robot_asset = robot_root.find("asset")
    if robot_asset is None:
        return

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")

    compiler = robot_root.find("compiler")
    meshdir = compiler.get("meshdir") if compiler is not None else None
    texturedir = compiler.get("texturedir") if compiler is not None else None
    assetdir = compiler.get("assetdir") if compiler is not None else None

    for child in robot_asset:
        cloned = copy.deepcopy(child)
        file_value = cloned.get("file")
        if file_value:
            cloned.set(
                "file",
                str(
                    _resolve_asset_file(
                        robot_dir=robot_dir,
                        relative_path=file_value,
                        tag=cloned.tag,
                        meshdir=meshdir,
                        texturedir=texturedir,
                        assetdir=assetdir,
                    )
                ),
            )
        asset.append(cloned)


def _resolve_asset_file(
    *,
    robot_dir: Path,
    relative_path: str,
    tag: str,
    meshdir: str | None,
    texturedir: str | None,
    assetdir: str | None,
) -> Path:
    """Resolve one asset file path relative to the robot MJCF source directory."""

    file_path = Path(relative_path).expanduser()
    if file_path.is_absolute():
        return file_path.resolve()

    base_dir = robot_dir
    if tag == "mesh" and meshdir:
        base_dir = robot_dir / meshdir
    elif tag == "texture" and texturedir:
        base_dir = robot_dir / texturedir
    elif assetdir:
        base_dir = robot_dir / assetdir

    return (base_dir / file_path).resolve()


def _merge_worldbody(root: ET.Element, robot_root: ET.Element) -> None:
    """Append the robot worldbody children into the scene worldbody."""

    robot_worldbody = robot_root.find("worldbody")
    if robot_worldbody is None:
        return

    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")

    for child in robot_worldbody:
        worldbody.append(copy.deepcopy(child))


def _apply_object_placements(
    root: ET.Element,
    object_placements: Iterable[ObjectPlacement],
) -> None:
    """Write RoboCasa task object placements into matching object bodies."""

    for object_placement in object_placements:
        body_name = f"{object_placement.name}_main"
        body = _find_body_by_name(root, body_name)
        if body is None:
            continue

        body.set("pos", _format_float_sequence(object_placement.pos))
        body.set("quat", _format_float_sequence(object_placement.quat))


def _apply_spawn_pose(
    root: ET.Element,
    spawn_pose: SpawnPose | None,
    robot_body_name: str | None,
) -> None:
    """Write the project robot root pose into the final XML tree."""

    if spawn_pose is None:
        return

    if not robot_body_name:
        raise RoboCasaMjcfAdaptationError(
            "robot_body_name must be set when spawn_pose is provided."
        )

    body = _find_body_by_name(root, robot_body_name)
    if body is None:
        raise RoboCasaMjcfAdaptationError(
            f"Project robot body not found: {robot_body_name}"
        )

    body.set("pos", _format_float_sequence(spawn_pose.pos))
    body.set("quat", _format_float_sequence(spawn_pose.quat))


def _remove_body_by_name(root: ET.Element, body_name: str) -> ET.Element | None:
    """Remove the first MJCF body with the given name.

    Args:
        root: Root ``<mujoco>`` element.
        body_name: Body name to remove.

    Returns:
        Removed body element, or ``None`` if no matching body exists.
    """

    for body in root.iter("body"):
        if body.get("name") != body_name:
            continue

        parent = _find_parent(root, body)
        if parent is None:
            return None

        parent.remove(body)
        return body

    return None


def _find_body_by_name(root: ET.Element, body_name: str) -> ET.Element | None:
    """Find the first MJCF body with the given name."""

    for body in root.iter("body"):
        if body.get("name") == body_name:
            return body

    return None


def _ensure_floor_geom(root: ET.Element) -> None:
    """Ensure a basic floor plane exists under ``<worldbody>``."""

    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")

    for geom in worldbody.findall("geom"):
        if geom.get("name") == "floor":
            return

    floor = ET.SubElement(worldbody, "geom")
    floor.set("name", "floor")
    floor.set("type", "plane")
    floor.set("size", "10 10 0.05")
    floor.set("pos", "0 0 0")
    floor.set("rgba", "0.8 0.8 0.8 1")


def _hide_debug_markers(root: ET.Element) -> None:
    """Make common RoboCasa debug markers transparent.

    RoboCasa scenes may include sites or geoms used as visual markers. This
    function keeps the elements in the model while making common red/blue marker
    colors transparent.
    """

    for elem in root.iter():
        if elem.tag not in {"geom", "site"}:
            continue

        rgba = elem.get("rgba")
        if rgba in DEBUG_MARKER_RGBA_VALUES:
            elem.set("rgba", TRANSPARENT_RGBA)


def _find_parent(root: ET.Element, target: ET.Element) -> ET.Element | None:
    """Find the parent element of ``target`` within an XML tree."""

    for parent in root.iter():
        for child in list(parent):
            if child is target:
                return parent

    return None


def _format_float_sequence(values: Iterable[float]) -> str:
    """Format numeric MJCF attributes consistently without avoidable noise."""

    return " ".join(f"{float(value):.12g}" for value in values)
