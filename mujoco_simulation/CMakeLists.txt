cmake_minimum_required(VERSION 3.16)
project(mujoco_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{MUJOCO_ROOT} AND NOT "$ENV{MUJOCO_ROOT}" STREQUAL "")
  set(MUJOCO_ROOT "$ENV{MUJOCO_ROOT}")
else()
  set(MUJOCO_ROOT "/opt/mujoco-3.9.0")
endif()

list(APPEND CMAKE_PREFIX_PATH "${MUJOCO_ROOT}")

find_package(ament_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(mujoco REQUIRED CONFIG)
find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
find_package(Threads REQUIRED)

set(MUJOCO_LIB mujoco::mujoco)
get_target_property(MUJOCO_INCLUDE_DIR mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(MUJOCO_RUNTIME_LIBRARY mujoco::mujoco IMPORTED_LOCATION)
if(NOT MUJOCO_RUNTIME_LIBRARY)
  get_target_property(MUJOCO_RUNTIME_LIBRARY mujoco::mujoco IMPORTED_LOCATION_RELEASE)
endif()
if(MUJOCO_RUNTIME_LIBRARY)
  get_filename_component(MUJOCO_RUNTIME_LIBRARY_DIR "${MUJOCO_RUNTIME_LIBRARY}" DIRECTORY)
endif()

add_library(lodepng STATIC
  src/viewer/lodepng/lodepng.cpp
)
target_include_directories(lodepng PRIVATE
  src/viewer/lodepng
)
set_target_properties(lodepng PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mujoco_platform_ui_adapter OBJECT
  src/viewer/simulate/glfw_adapter.cc
  src/viewer/simulate/glfw_dispatch.cc
  src/viewer/simulate/platform_ui_adapter.cc
)
target_include_directories(mujoco_platform_ui_adapter PRIVATE
  src/viewer/simulate
  ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(mujoco_platform_ui_adapter PRIVATE
  glfw
  ${MUJOCO_LIB}
)
set_target_properties(mujoco_platform_ui_adapter PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mujoco_official_simulate STATIC
  $<TARGET_OBJECTS:mujoco_platform_ui_adapter>
  src/viewer/simulate/simulate.cc
)
target_include_directories(mujoco_official_simulate PRIVATE
  src/viewer/simulate
  src/viewer/lodepng
  ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(mujoco_official_simulate PRIVATE
  glfw
  lodepng
  ${MUJOCO_LIB}
  OpenGL::OpenGL
)
set_target_properties(mujoco_official_simulate PROPERTIES POSITION_INDEPENDENT_CODE ON)

set(MUJOCO_SIMULATION_PUBLIC_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)

add_library(mujoco_simulation_headers INTERFACE)
target_include_directories(mujoco_simulation_headers INTERFACE
  ${MUJOCO_SIMULATION_PUBLIC_INCLUDE_DIRS}
)

add_library(mujoco_simulation_components STATIC
  src/component/component_manager.cpp
  src/component/component_registry.cpp
  src/component/camera/camera_component.cpp
  src/component/imu/imu_component.cpp
  src/component/joint/joint_component.cpp
  src/component/lidar/lidar_component.cpp
  src/component/mobile_base/mobile_base_component.cpp
  src/scheduler/sensor_scheduler.cpp
)
target_link_libraries(mujoco_simulation_components PUBLIC
  mujoco_simulation_headers
)
set_target_properties(mujoco_simulation_components PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mujoco_simulation_runtime STATIC
  src/runtime/model_runtime.cpp
  src/runtime/simulation_scheduler.cpp
)
target_link_libraries(mujoco_simulation_runtime PUBLIC
  mujoco_simulation_headers
)
set_target_properties(mujoco_simulation_runtime PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mujoco_simulation_camera STATIC
  src/component/camera/camera_renderer.cpp
)
target_link_libraries(mujoco_simulation_camera PUBLIC
  mujoco_simulation_headers
)
target_link_libraries(mujoco_simulation_camera PRIVATE
  ${MUJOCO_LIB}
  OpenGL::EGL
  glfw
)
set_target_properties(mujoco_simulation_camera PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mujoco_simulation_viewer_support STATIC
  src/viewer/viewer.cpp
)
target_link_libraries(mujoco_simulation_viewer_support PUBLIC
  mujoco_simulation_headers
)
target_include_directories(mujoco_simulation_viewer_support PRIVATE
  src/viewer/simulate
  src/viewer/lodepng
)
target_link_libraries(mujoco_simulation_viewer_support PRIVATE
  ${MUJOCO_LIB}
  mujoco_official_simulate
  glfw
)
set_target_properties(mujoco_simulation_viewer_support PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED
  src/simulation.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  mujoco_simulation_headers
  ${MUJOCO_LIB}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  Threads::Threads
  mujoco_simulation_components
  mujoco_simulation_runtime
  mujoco_simulation_camera
  mujoco_simulation_viewer_support
)
if(MUJOCO_RUNTIME_LIBRARY_DIR)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${MUJOCO_RUNTIME_LIBRARY_DIR}"
    INSTALL_RPATH "${MUJOCO_RUNTIME_LIBRARY_DIR}"
  )
endif()

install(
  TARGETS ${PROJECT_NAME} mujoco_simulation_headers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  add_executable(perf_runtime_scenarios
    test/performance/perf_runtime_scenarios.cpp
  )
  target_link_libraries(perf_runtime_scenarios ${PROJECT_NAME})

  ament_add_gtest(test_model_runtime
    test/unit/test_model_runtime.cpp
  )
  if(TARGET test_model_runtime)
    target_link_libraries(test_model_runtime ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_status
    test/unit/test_status.cpp
  )
  if(TARGET test_status)
    target_link_libraries(test_status mujoco_simulation_headers)
  endif()

  ament_add_gtest(test_simulation_scheduler
    test/unit/test_simulation_scheduler.cpp
  )
  if(TARGET test_simulation_scheduler)
    target_link_libraries(test_simulation_scheduler mujoco_simulation_runtime)
  endif()

  ament_add_gtest(test_joint_component
    test/unit/test_joint_component.cpp
  )
  if(TARGET test_joint_component)
    target_link_libraries(test_joint_component ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_component_registry
    test/unit/test_component_registry.cpp
  )
  if(TARGET test_component_registry)
    target_link_libraries(test_component_registry ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_component_config
    test/unit/test_component_config.cpp
  )
  if(TARGET test_component_config)
    target_link_libraries(test_component_config ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_mobile_base
    test/unit/test_mobile_base.cpp
  )
  if(TARGET test_mobile_base)
    target_link_libraries(test_mobile_base ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_buffered_simulation
    test/unit/test_buffered_simulation.cpp
  )
  if(TARGET test_buffered_simulation)
    target_link_libraries(test_buffered_simulation ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_command_buffer
    test/unit/test_command_buffer.cpp
  )
  if(TARGET test_command_buffer)
    target_link_libraries(test_command_buffer mujoco_simulation_headers)
  endif()

  ament_add_gtest(test_sensor_scheduler
    test/unit/test_sensor_scheduler.cpp
  )
  if(TARGET test_sensor_scheduler)
    target_link_libraries(test_sensor_scheduler mujoco_simulation_components)
  endif()

  ament_add_gtest(test_sensor_sampling
    test/unit/test_sensor_sampling.cpp
  )
  if(TARGET test_sensor_sampling)
    target_link_libraries(test_sensor_sampling ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_camera_buffer
    test/unit/test_camera_buffer.cpp
  )
  if(TARGET test_camera_buffer)
    target_link_libraries(test_camera_buffer ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_state_buffer
    test/unit/test_state_buffer.cpp
  )
  if(TARGET test_state_buffer)
    target_link_libraries(test_state_buffer ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_camera_sample
    test/unit/test_camera_sample.cpp
  )
  if(TARGET test_camera_sample)
    target_link_libraries(test_camera_sample ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_camera_rendering
    test/unit/test_camera_rendering.cpp
  )
  if(TARGET test_camera_rendering)
    target_link_libraries(test_camera_rendering ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_camera_runtime
    test/unit/test_camera_runtime.cpp
  )
  if(TARGET test_camera_runtime)
    target_link_libraries(test_camera_runtime ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_viewer
    test/unit/test_viewer.cpp
  )
  if(TARGET test_viewer)
    target_link_libraries(test_viewer ${PROJECT_NAME})
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mujoco)
ament_package()
