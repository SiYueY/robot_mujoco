cmake_minimum_required(VERSION 3.16)

project(mujoco_simulation_package_consumer LANGUAGES CXX)

find_package(mujoco_simulation CONFIG REQUIRED)

add_executable(mujoco_simulation_package_consumer main.cpp)
target_link_libraries(mujoco_simulation_package_consumer
  PRIVATE mujoco_simulation::mujoco_simulation)
