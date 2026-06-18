# turtlebot3_mujoco

Standalone TurtleBot3 Waffle Pi MuJoCo example under `robot_mujoco/examples/turtlebot3_mujoco`.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_mujoco turtlebot3_mujoco
```

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch turtlebot3_mujoco tb3_sim.launch.py
```

Start with RViz:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch turtlebot3_mujoco tb3_sim.launch.py use_rviz:=true
```

Drive the base:

```bash
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.3}}"
```
