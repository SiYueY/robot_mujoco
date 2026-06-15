cmake_minimum_required(VERSION 3.16)
project(franka_fr3_mujoco)

find_package(ament_cmake REQUIRED)
find_package(robot_mujoco REQUIRED)

install(DIRECTORY
  config
  launch
  model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
