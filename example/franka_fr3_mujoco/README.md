# robot_mujoco FR3 Example Resources

This package mainly provides the default FR3 model resources reused by the workspace-level `robot_mujoco` runtime and RoboCasa scene flow.

## Contents

- `model/`: copied MuJoCo Menagerie FR3 model and assets
- `config/fr3.urdf.template`: minimal ROS 2 robot description used by the `robot_mujoco` FR3 example
- `config/controllers.yaml`: joint state broadcaster and position command controller
- `launch/fr3_sim.launch.py`: legacy example launch for directly starting the FR3 scene

The generated `ros2_control` system name is `robot_mujoco_fr3_system`.

## Run

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_mujoco franka_fr3_mujoco
```

Direct example launch:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_mujoco fr3_sim.launch.py
```

For the current recommended RoboCasa integration flow, prefer the unified workspace command instead:

```bash
uv run robot-mujoco-robocasa-scene \
  --config robot_mujoco/config/robocasa_fr3_scene.yaml \
  --output /tmp/robot_mujoco_robocasa_scene.xml
```

The current project runtime is viewer-only. If `gladLoadGL error` appears, fix the local OpenGL / display environment first.

Send a position command:

```bash
ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.2, 0.0, -1.8, 0.0, 1.6, 0.7]}"
```
