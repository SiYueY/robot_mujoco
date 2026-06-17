cmake_minimum_required(VERSION 3.16)
project(mujoco_simulation_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(mujoco_ros2_bridge_msgs REQUIRED)
find_package(mujoco_simulation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

get_filename_component(MUJOCO_ROS2_BRIDGE_MSGS_PREFIX
  "${mujoco_ros2_bridge_msgs_DIR}/../../.."
  ABSOLUTE
)
set(MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR "${MUJOCO_ROS2_BRIDGE_MSGS_PREFIX}/lib")

add_library(${PROJECT_NAME} SHARED
  src/simulation_ros_bridge.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR}"
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  mujoco_ros2_bridge_msgs
  rclcpp
  rosgraph_msgs
  sensor_msgs
  std_srvs
)
target_link_libraries(${PROJECT_NAME}
  mujoco_simulation::mujoco_simulation_headers
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  ament_add_gtest(test_simulation_ros_bridge
    tests/test_simulation_ros_bridge.cpp
  )
  if(TARGET test_simulation_ros_bridge)
    target_link_libraries(test_simulation_ros_bridge ${PROJECT_NAME})
    set_target_properties(test_simulation_ros_bridge PROPERTIES
      BUILD_RPATH "${MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR}"
    )
    ament_target_dependencies(test_simulation_ros_bridge
      mujoco_ros2_bridge_msgs
      rclcpp
      rosgraph_msgs
      sensor_msgs
      std_srvs
    )
    set_tests_properties(test_simulation_ros_bridge PROPERTIES
      ENVIRONMENT "LD_LIBRARY_PATH=${MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR}:$ENV{LD_LIBRARY_PATH}"
    )
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  mujoco_ros2_bridge_msgs
  mujoco_simulation
  rclcpp
  rosgraph_msgs
  sensor_msgs
  std_srvs
)
ament_package()
