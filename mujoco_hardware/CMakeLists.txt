cmake_minimum_required(VERSION 3.16)
project(mujoco_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mujoco_ros2_bridge_msgs REQUIRED)
find_package(mujoco_simulation REQUIRED)
find_package(mujoco_simulation_ros REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

get_filename_component(MUJOCO_ROS2_BRIDGE_MSGS_PREFIX
  "${mujoco_ros2_bridge_msgs_DIR}/../../.."
  ABSOLUTE
)
set(MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR "${MUJOCO_ROS2_BRIDGE_MSGS_PREFIX}/lib")

add_library(${PROJECT_NAME}_core STATIC
  src/config.cpp
  src/mujoco_hardware_interface.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  BUILD_RPATH "${MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR}"
)
add_library(${PROJECT_NAME} SHARED
  src/plugin_entry.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR}"
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(${PROJECT_NAME}_core
  hardware_interface
  mujoco_simulation
  mujoco_simulation_ros
  rclcpp
  rclcpp_lifecycle
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)

pluginlib_export_plugin_description_file(hardware_interface mujoco_hardware_plugin.xml)

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES mujoco_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(perf_read_write_loop
    tests/performance/perf_read_write_loop.cpp
  )
  target_link_libraries(perf_read_write_loop ${PROJECT_NAME}_core)
  set_target_properties(perf_read_write_loop PROPERTIES
    BUILD_RPATH "${MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR}"
  )
  ament_target_dependencies(perf_read_write_loop
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )

  ament_add_gtest(test_mujoco_hardware_interface
    tests/test_mujoco_hardware_interface.cpp
  )
  if(TARGET test_mujoco_hardware_interface)
    target_link_libraries(test_mujoco_hardware_interface ${PROJECT_NAME}_core)
    set_target_properties(test_mujoco_hardware_interface PROPERTIES
      BUILD_RPATH "${MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR}"
    )
    ament_target_dependencies(test_mujoco_hardware_interface
      hardware_interface
      mujoco_ros2_bridge_msgs
      mujoco_simulation_ros
      rclcpp
      rclcpp_lifecycle
      rosgraph_msgs
      sensor_msgs
      std_srvs
    )
    set_tests_properties(test_mujoco_hardware_interface PROPERTIES
      ENVIRONMENT "LD_LIBRARY_PATH=${MUJOCO_ROS2_BRIDGE_MSGS_RUNTIME_DIR}:$ENV{LD_LIBRARY_PATH}"
    )
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  mujoco_simulation
  mujoco_simulation_ros
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
