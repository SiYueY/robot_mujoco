cmake_minimum_required(VERSION 3.16)
project(mujoco_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mujoco_vendor QUIET)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

if(mujoco_vendor_FOUND)
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR "")
else()
  set(MUJOCO_LOCAL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco")
  set(MUJOCO_HOME_ROOT "$ENV{HOME}/.mujoco")
  file(GLOB MUJOCO_HOME_INSTALLS
    "${MUJOCO_HOME_ROOT}/mujoco-*"
    "${MUJOCO_HOME_ROOT}/mujoc-*"
  )
  set(MUJOCO_SEARCH_ROOTS
    "${MUJOCO_LOCAL_ROOT}"
    ${MUJOCO_HOME_INSTALLS}
  )
  find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    PATHS
      ${MUJOCO_SEARCH_ROOTS}
      /usr/local/include
      /usr/include
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
  )
  find_library(MUJOCO_LIB
    NAMES mujoco libmujoco.so libmujoco.so.3.9.0 libmujoco.so.3.8.0
    PATHS
      ${MUJOCO_SEARCH_ROOTS}
      /usr/local/lib
      /usr/lib
    PATH_SUFFIXES lib build/lib bin
    NO_DEFAULT_PATH
  )
  if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
    message(FATAL_ERROR "mujoco_vendor was not found and local MuJoCo could not be resolved.")
  endif()
  get_filename_component(MUJOCO_RUNTIME_LIBRARY_DIR "${MUJOCO_LIB}" DIRECTORY)
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

add_library(${PROJECT_NAME} SHARED
  src/hardware/camera.cpp
  src/hardware/hardware_manager.cpp
  src/hardware/imu.cpp
  src/hardware/joint.cpp
  src/hardware/lidar.cpp
  src/hardware/mobile_base.cpp
  src/mujoco_simulation.cpp
  src/viewer/viewer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  src/viewer/simulate
  src/viewer/lodepng
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  rclcpp
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}
  ${MUJOCO_LIB}
  Threads::Threads
)
target_link_libraries(${PROJECT_NAME}
  mujoco_official_simulate
  glfw
)
if(MUJOCO_RUNTIME_LIBRARY_DIR)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${MUJOCO_RUNTIME_LIBRARY_DIR}"
    INSTALL_RPATH "${MUJOCO_RUNTIME_LIBRARY_DIR}"
  )
endif()

install(
  TARGETS ${PROJECT_NAME} mujoco_official_simulate lodepng
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  rclcpp
  sensor_msgs
)
ament_package()
