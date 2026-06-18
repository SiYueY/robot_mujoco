# robot_mujoco package

`robot_mujoco/robot_mujoco` is the unified ROS 2 product package for this repository.
It owns:

- product launch entry points
- packaged robot configs and MuJoCo models
- the Python runtime helpers
- RoboCasa scene generation and launch integration

It depends on:

- `mujoco_simulation` for the MuJoCo runtime
- `robot_mujoco_ros2` for the `ros2_control` hardware plugin and ROS bridge

## Launch entry points

Use the repository-level environment script first:

```bash
source /home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/scripts/robot_mujoco.bash
```

After that, use one of these launch files:

```bash
ros2 launch robot_mujoco robot_mujoco.launch.py ...
ros2 launch robot_mujoco franka_fr3_mujoco.launch.py
ros2 launch robot_mujoco turtlebot3_mujoco.launch.py
```

Roles:

- `robot_mujoco.launch.py`: shared low-level launcher
- `franka_fr3_mujoco.launch.py`: FR3 defaults
- `turtlebot3_mujoco.launch.py`: TurtleBot3 defaults

## RoboCasa switch

All three launch files expose the same RoboCasa-related arguments:

- `use_robocasa_scene`
- `robocasa_config`
- `robocasa_output_xml`
- `robocasa_task`
- `robocasa_layout`
- `robocasa_style`

Behavior:

- `use_robocasa_scene:=false`: load the provided static MuJoCo XML directly
- `use_robocasa_scene:=true`: generate a RoboCasa scene XML first, then launch that scene
- in RoboCasa mode, the FR3 wrapper no longer applies the static scene's default `initial_keyframe:=home`
- when `render_mode:=viewer` is requested without `DISPLAY` or `WAYLAND_DISPLAY`, launch falls back to `headless`
- when `use_rviz:=true` is requested without `DISPLAY` or `WAYLAND_DISPLAY`, launch skips `rviz2`

Current limitation:

- RoboCasa scene generation is shared by both `franka_fr3` and `turtlebot3`
- robot-specific behavior still depends on each robot's packaged MJCF and controller config

Example:

```bash
source /home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/scripts/robot_mujoco.bash
ros2 launch robot_mujoco franka_fr3_mujoco.launch.py \
  use_robocasa_scene:=true
```

## Resource layout

Key packaged resources:

- `launch/`
- `config/franka_fr3/`
- `config/turtlebot3/`
- `model/franka_fr3/`
- `model/turtlebot3/`
- `robocasa/`

Notable files:

- `config/franka_fr3/franka_fr3.urdf.template`
- `config/franka_fr3/controllers.yaml`
- `config/franka_fr3/franka_fr3_mujoco.rviz`
- `config/franka_fr3/robocasa_scene.yaml`
- `model/franka_fr3/franka_fr3.xml`
- `model/franka_fr3/franka_fr3_scene.xml`
- `config/turtlebot3/turtlebot3_waffle_pi.urdf.template`
- `config/turtlebot3/controllers.yaml`
- `config/turtlebot3/robocasa_scene.yaml`
- `config/turtlebot3/turtlebot3_mujoco.rviz`
- `model/turtlebot3/scene_turtlebot3_waffle_pi.xml`

## Python helpers

Important Python entry points:

- `runtime.py`: launch helper functions and default resource resolution
- `launch_utils.py`: shared launch-time helpers, including RoboCasa scene generation
- `robocasa/scene_runner_cli.py`: generate-and-launch CLI path

The default runtime launch target is `launch/robot_mujoco.launch.py`.

## Notes

- `sim.launch.py` is intentionally removed; use `robot_mujoco.launch.py` instead.
- `scripts/robot_mujoco.bash` exists to bridge ROS 2's system Python with packages installed only in the local `.venv`.
- This README is package-local. For workspace setup, dependencies, and full RoboCasa environment preparation, use the repository-level [README.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/README.md).
