cmake_minimum_required(VERSION 3.16)

project(romujoco_package_consumer LANGUAGES CXX)

find_package(romujoco CONFIG REQUIRED)

add_executable(romujoco_package_consumer main.cpp)
target_link_libraries(romujoco_package_consumer
  PRIVATE romujoco::romujoco)
