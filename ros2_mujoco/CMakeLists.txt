cmake_minimum_required(VERSION 3.16)
project(ros2_mujoco)

# mujoco_simulation is a standalone CMake package (not an ament package), so
# preserve its discovered library directory for installed ROS executables and
# pluginlib-loaded shared libraries.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(mujoco_simulation CONFIG QUIET)
if(NOT mujoco_simulation_FOUND)
  find_package(mujoco_simulation CONFIG REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../build/mujoco_simulation" NO_DEFAULT_PATH)
endif()

add_library(${PROJECT_NAME}_hardware SHARED
  src/mujoco_hardware.cpp
  src/hardware/joint.cpp src/hardware/mobile_base.cpp src/hardware/imu.cpp
  src/hardware/camera.cpp src/hardware/lidar.cpp)
target_compile_features(${PROJECT_NAME}_hardware PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_hardware PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_hardware controller_manager geometry_msgs hardware_interface nav_msgs pluginlib rclcpp rosgraph_msgs sensor_msgs tf2_ros yaml-cpp)
target_link_libraries(${PROJECT_NAME}_hardware mujoco_simulation::mujoco_simulation yaml-cpp)
# The standalone core links MuJoCo transitively.  DT_RPATH (rather than the
# non-transitive DT_RUNPATH default) keeps that dependency discoverable when
# pluginlib loads this library.
target_link_options(${PROJECT_NAME}_hardware PRIVATE -Wl,--disable-new-dtags)
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

install(TARGETS ${PROJECT_NAME}_hardware EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_hardware)
ament_export_dependencies(controller_manager hardware_interface pluginlib rclcpp)
ament_package()
