"""Generate MuJoCo-ready scenes from RoboCasa tasks."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .exceptions import (
    RoboCasaDependencyError,
    RoboCasaIntegrationError,
    RoboCasaSceneGenerationError,
)
from .mjcf_adapter import adapt_mjcf
from .scene_config import SceneConfig
from .scene_data import (
    GeneratedScene,
    ObjectPlacement,
    SceneMetadata,
    SpawnPose,
)


class SceneGenerator:
    """Generate project-ready MuJoCo scenes from RoboCasa kitchen tasks."""

    def generate(
        self,
        config: SceneConfig,
        *,
        spawn_pose: SpawnPose | None = None,
    ) -> GeneratedScene:
        """Generate and adapt a RoboCasa scene.

        Args:
            config: RoboCasa scene generation configuration.
            spawn_pose: Optional explicit spawn pose for the project robot.
                If omitted, the generator tries to use RoboCasa's recommended
                initial robot anchor.

        Returns:
            A generated scene containing ``mujoco.MjModel``, final XML, and
            scene information.

        Raises:
            RoboCasaSceneGenerationError: If RoboCasa scene generation fails.
            RoboCasaDependencyError: If required Python dependencies are missing.
        """

        config.validate()

        mujoco = self._import_mujoco()
        env = self._make_robocasa_env(config)

        try:
            self._ensure_model_loaded(env)

            raw_xml = self._get_model_xml(env)
            object_placements = self._extract_object_placements(env)

            if spawn_pose is None:
                spawn_pose = self._extract_spawn_pose(env)

            final_xml = adapt_mjcf(
                raw_xml,
                robot_xml_path=config.robot_xml_path,
                placeholder_body_name=config.placeholder_robot_body_name,
                object_placements=object_placements,
                spawn_pose=spawn_pose,
                robot_body_name=config.robot_root_body_name,
            )

            if config.write_generated_xml and config.generated_xml_path:
                self._write_xml(final_xml, config.generated_xml_path)

            model = mujoco.MjModel.from_xml_string(final_xml)

            metadata = SceneMetadata(
                task_name=config.task_name,
                layout_id=config.layout_id,
                style_id=config.style_id,
                spawn_pose=spawn_pose,
                object_placements=tuple(object_placements),
                raw_object_cfgs=tuple(getattr(env, "object_cfgs", []) or []),
            )

            return GeneratedScene(
                model=model,
                xml=final_xml,
                metadata=metadata,
            )
        except Exception as exc:
            if isinstance(exc, RoboCasaIntegrationError):
                raise

            raise RoboCasaSceneGenerationError(
                "Failed to generate RoboCasa scene "
                f"task={config.task_name}, layout={config.layout_id}, "
                f"style={config.style_id}."
            ) from exc
        finally:
            self._close_env(env)

    def _make_robocasa_env(self, config: SceneConfig) -> Any:
        """Create a RoboCasa / robosuite environment for model generation."""

        try:
            import robosuite
            from robosuite.controllers import load_part_controller_config
        except ImportError as exc:
            raise RoboCasaDependencyError(
                "robosuite is required for RoboCasa scene generation. "
                "Install it in .venv and source robot_mujoco.bash before launching."
            ) from exc

        # Importing robocasa registers RoboCasa environments with robosuite.
        try:
            import robocasa  # noqa: F401
        except Exception as exc:
            raise RoboCasaDependencyError(
                "robocasa could not be imported for RoboCasa scene generation. "
                "Install it in .venv, pin mujoco==3.3.1, and source robot_mujoco.bash before launching."
            ) from exc

        try:
            controller_configs = load_part_controller_config(
                default_controller=config.controller
            )

            return robosuite.make(
                env_name=config.task_name,
                robots=config.placeholder_robot,
                controller_configs=controller_configs,
                translucent_robot=False,
                layout_and_style_ids=[[config.layout_id, config.style_id]],
                has_offscreen_renderer=False,
                render_camera=None,
                ignore_done=True,
                use_camera_obs=False,
                control_freq=config.control_freq,
            )
        except Exception as exc:  # noqa: BLE001
            raise RoboCasaSceneGenerationError(
                "Failed to create RoboCasa / robosuite environment "
                f"for task={config.task_name}, layout={config.layout_id}, "
                f"style={config.style_id}."
            ) from exc

    def _ensure_model_loaded(self, env: Any) -> None:
        """Ensure that the RoboCasa environment has generated its model.

        Prefer public ``reset()`` first. Use the private ``_load_model()`` only as
        a compatibility fallback because some RoboCasa / robosuite versions make
        the generated XML available through that path.
        """

        if getattr(env, "model", None) is not None:
            return

        try:
            env.reset()
        except Exception:
            # Fallback below may still work for model generation without a full
            # successful environment reset.
            pass

        if getattr(env, "model", None) is not None:
            return

        if not hasattr(env, "_load_model"):
            raise RoboCasaSceneGenerationError(
                "RoboCasa environment did not expose a generated model, and "
                "no _load_model fallback exists."
            )

        env._load_model()

    def _get_model_xml(self, env: Any) -> str:
        """Return the generated RoboCasa MJCF XML string."""

        model = getattr(env, "model", None)
        if model is None:
            raise RoboCasaSceneGenerationError(
                "RoboCasa environment has no model after loading."
            )

        if not hasattr(model, "get_xml"):
            raise RoboCasaSceneGenerationError(
                "RoboCasa model does not provide get_xml()."
            )

        xml = model.get_xml()
        if not isinstance(xml, str) or not xml.strip():
            raise RoboCasaSceneGenerationError("RoboCasa generated empty MJCF XML.")

        return xml

    def _extract_object_placements(self, env: Any) -> list[ObjectPlacement]:
        """Extract object placements generated by the RoboCasa task."""

        object_cfgs = getattr(env, "object_cfgs", []) or []
        object_placements = getattr(env, "object_placements", {}) or {}

        results: list[ObjectPlacement] = []

        for cfg in object_cfgs:
            if not isinstance(cfg, dict):
                continue

            name = cfg.get("name")
            if not name or name not in object_placements:
                continue

            placement = object_placements[name]
            if not placement or len(placement) < 2:
                continue

            pos = placement[0]
            quat = placement[1]

            category = None
            info = cfg.get("info")
            if isinstance(info, dict):
                category = info.get("cat")

            results.append(
                ObjectPlacement(
                    name=str(name),
                    category=str(category) if category is not None else None,
                    pos=_as_float_tuple(pos, expected_len=3),
                    quat=_as_float_tuple(quat, expected_len=4),
                )
            )

        return results

    def _extract_spawn_pose(self, env: Any) -> SpawnPose | None:
        """Extract RoboCasa's recommended initial robot base pose if available."""

        pos = getattr(env, "init_robot_base_pos_anchor", None)
        ori = getattr(env, "init_robot_base_ori_anchor", None)

        if pos is None or ori is None:
            return None

        np = _import_numpy()

        pos_array = np.asarray(pos, dtype=float)
        ori_array = np.asarray(ori, dtype=float)

        if pos_array.shape[0] != 3 or ori_array.shape[0] != 3:
            return None

        quat = _euler_xyz_to_quat_wxyz(ori_array)

        return SpawnPose(
            pos=_as_float_tuple(pos_array, expected_len=3),
            quat=_as_float_tuple(quat, expected_len=4),
        )

    def _write_xml(self, xml: str, path: str | Path) -> None:
        """Write generated MJCF XML to disk."""

        output_path = Path(path).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(xml, encoding="utf-8")

    def _close_env(self, env: Any) -> None:
        """Close a RoboCasa / robosuite environment if possible."""

        close = getattr(env, "close", None)
        if callable(close):
            close()

    def _import_mujoco(self) -> Any:
        """Import MuJoCo lazily to keep dependency errors explicit."""

        try:
            import mujoco
        except ImportError as exc:
            raise RoboCasaDependencyError(
                "mujoco is required to build the final MjModel. "
                "Install it with your project dependencies."
            ) from exc

        return mujoco


def _as_float_tuple(value: Any, *, expected_len: int) -> tuple[float, ...]:
    """Convert an array-like value to a fixed-length tuple of floats."""

    np = _import_numpy()
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape[0] != expected_len:
        raise RoboCasaSceneGenerationError(
            f"Expected {expected_len} values, got {array.shape[0]}."
        )

    return tuple(float(x) for x in array)


def _euler_xyz_to_quat_wxyz(euler_xyz: Any) -> Any:
    """Convert XYZ Euler angles to a MuJoCo ``wxyz`` quaternion."""

    np = _import_numpy()
    try:
        from scipy.spatial.transform import Rotation
    except ImportError as exc:
        raise RoboCasaDependencyError(
            "scipy is required to convert RoboCasa robot spawn orientation. "
            "Install it with: pip install scipy"
        ) from exc

    quat_xyzw = Rotation.from_euler("xyz", euler_xyz).as_quat()
    return np.array(
        [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
        dtype=float,
    )


def _import_numpy() -> Any:
    """Import numpy lazily so package import stays lightweight."""

    try:
        import numpy as np
    except ImportError as exc:
        raise RoboCasaDependencyError(
            "numpy is required for RoboCasa scene generation. "
            "Install it with your project Python dependencies."
        ) from exc

    return np
