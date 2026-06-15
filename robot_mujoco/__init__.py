"""Workspace-level Python helpers for ``robot_mujoco``.

This package serves as the unified Python entry point for the robot_mujoco
project. It provides:

- ``robot_mujoco.robocasa`` — RoboCasa kitchen scene generation and MJCF adaptation
- ``robot-mujoco-robocasa-scene`` — unified generate-and-launch CLI
- ``robot_mujoco.SceneConfig`` — Alias for the most-used scene generation type

.. code-block:: python

    from robot_mujoco import SceneConfig, load_config
    from robot_mujoco.robocasa import SceneGenerator

    config = load_config("scene.yaml")
    scene = SceneGenerator().generate(config)
"""

from robot_mujoco.robocasa.scene_config import SceneConfig, config_from_mapping, load_config
from robot_mujoco.robocasa.scene_data import (
    GeneratedScene,
    ObjectPlacement,
    SceneMetadata,
    SpawnPose,
)

__all__ = [
    "GeneratedScene",
    "ObjectPlacement",
    "SceneConfig",
    "SceneMetadata",
    "SpawnPose",
    "config_from_mapping",
    "load_config",
]
