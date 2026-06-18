# franka_fr3_mujoco

Standalone FR3 MuJoCo example under `robot_mujoco/examples/franka_fr3_mujoco`.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_mujoco franka_fr3_mujoco
```

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_mujoco fr3_sim.launch.py
```

Send a position command:

```bash
ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.2, 0.0, -1.8, 0.0, 1.6, 0.7]}"
```
