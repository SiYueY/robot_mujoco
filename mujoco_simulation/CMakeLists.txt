cmake_minimum_required(VERSION 3.16)

project(
  mujoco_simulation
  VERSION 0.1.0
  DESCRIPTION "MuJoCo simulation"
  LANGUAGES CXX
)

if(NOT UNIX OR APPLE)
  message(FATAL_ERROR "mujoco_simulation currently supports Linux only.")
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(MUJOCO_SIMULATION_BUILD_SHARED
       "Build mujoco_simulation as a shared library" ON)
option(MUJOCO_SIMULATION_BUILD_TESTS "Build mujoco_simulation tests" OFF)
option(MUJOCO_SIMULATION_BUILD_EXAMPLES "Build mujoco_simulation examples" OFF)
option(MUJOCO_SIMULATION_INSTALL "Generate installation rules" ON)
option(MUJOCO_SIMULATION_ENABLE_ASAN "Enable AddressSanitizer" OFF)

set(MUJOCO_ROOT_PATH "")
if(DEFINED ENV{MUJOCO_ROOT} AND NOT "$ENV{MUJOCO_ROOT}" STREQUAL "")
  set(MUJOCO_ROOT_PATH "$ENV{MUJOCO_ROOT}")
else()
  set(MUJOCO_ROOT_PATH "/opt/mujoco-3.9.0")
endif()

file(TO_CMAKE_PATH "${MUJOCO_ROOT_PATH}" MUJOCO_ROOT_PATH)
get_filename_component(MUJOCO_ROOT_PATH "${MUJOCO_ROOT_PATH}" ABSOLUTE)

if(NOT IS_DIRECTORY "${MUJOCO_ROOT_PATH}")
  message(FATAL_ERROR
    "MuJoCo installation directory was not found.\n"
    "Checked path: ${MUJOCO_ROOT_PATH}\n"
    "Set MUJOCO_ROOT or install MuJoCo under /opt/mujoco-3.9.0.")
endif()

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  PATHS "${MUJOCO_ROOT_PATH}/include"
  NO_DEFAULT_PATH)
find_library(MUJOCO_LIBRARY
  NAMES mujoco
  PATHS
    "${MUJOCO_ROOT_PATH}/lib"
    "${MUJOCO_ROOT_PATH}/lib64"
  NO_DEFAULT_PATH)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "MuJoCo headers or library were not found under '${MUJOCO_ROOT_PATH}'.")
endif()

message(STATUS "MuJoCo root: ${MUJOCO_ROOT_PATH}")
message(STATUS "MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo library: ${MUJOCO_LIBRARY}")

if(NOT TARGET mujoco::mujoco)
  add_library(mujoco::mujoco UNKNOWN IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
endif()

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
find_package(glfw3 CONFIG REQUIRED)

if(NOT TARGET OpenGL::GL OR NOT TARGET OpenGL::EGL)
  message(FATAL_ERROR "OpenGL and EGL CMake targets are required.")
endif()

if(TARGET glfw)
  set(MUJOCO_SIMULATION_GLFW_TARGET glfw)
elseif(TARGET glfw3::glfw)
  set(MUJOCO_SIMULATION_GLFW_TARGET glfw3::glfw)
else()
  message(FATAL_ERROR
    "glfw3 was found, but neither target 'glfw' nor 'glfw3::glfw' is available.")
endif()

set(MUJOCO_SIMULATION_SOURCES
  src/simulation.cpp
  src/buffer/command_buffer.cpp
  src/buffer/state_buffer.cpp
  src/component/simulation_component.cpp
  src/component/component_manager.cpp
  src/component/camera/camera_component.cpp
  src/component/imu/imu_component.cpp
  src/component/joint/joint_component.cpp
  src/component/lidar/lidar_component.cpp
  src/component/mobile_base/mobile_base_component.cpp
  src/config/simulation_config.cpp
  src/mujoco/camera_renderer.cpp
  src/runtime/simulation_runtime.cpp
  src/runtime/simulation_scheduler.cpp
  src/viewer/simulation_viewer.cpp
  src/viewer/lodepng/lodepng.cpp
  src/viewer/simulate/glfw_adapter.cc
  src/viewer/simulate/glfw_dispatch.cc
  src/viewer/simulate/platform_ui_adapter.cc
  src/viewer/simulate/simulate.cc
  third_party/easyloggingpp/easylogging++.cc
  third_party/tinyxml2/tinyxml2.cpp
)

if(MUJOCO_SIMULATION_BUILD_SHARED)
  set(MUJOCO_SIMULATION_LIBRARY_TYPE SHARED)
else()
  set(MUJOCO_SIMULATION_LIBRARY_TYPE STATIC)
endif()

add_library(mujoco_simulation ${MUJOCO_SIMULATION_LIBRARY_TYPE}
  ${MUJOCO_SIMULATION_SOURCES})
add_library(mujoco_simulation::mujoco_simulation ALIAS mujoco_simulation)

set_source_files_properties(
  third_party/easyloggingpp/easylogging++.cc
  PROPERTIES COMPILE_DEFINITIONS AUTO_INITIALIZE_EASYLOGGINGPP)

set(MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/generated")
configure_file(
  include/mujoco_simulation/version.hpp.in
  "${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}/mujoco_simulation/version.hpp"
  @ONLY)

target_compile_features(mujoco_simulation PUBLIC cxx_std_17)
target_include_directories(mujoco_simulation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/viewer/simulate
    ${CMAKE_CURRENT_SOURCE_DIR}/src/viewer/lodepng
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/easyloggingpp
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2)
target_link_libraries(mujoco_simulation
  PUBLIC
    mujoco::mujoco
    Threads::Threads
  PRIVATE
    OpenGL::GL
    OpenGL::EGL
    ${MUJOCO_SIMULATION_GLFW_TARGET}
    ${CMAKE_DL_LIBS})
target_compile_definitions(mujoco_simulation PRIVATE
  MUJOCO_SIMULATION_BUILDING_LIBRARY)

if(NOT MUJOCO_SIMULATION_BUILD_SHARED)
  target_compile_definitions(mujoco_simulation PUBLIC
    MUJOCO_SIMULATION_STATIC_DEFINE)
endif()

set_target_properties(mujoco_simulation PROPERTIES
  OUTPUT_NAME mujoco_simulation
  EXPORT_NAME mujoco_simulation
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  CXX_EXTENSIONS OFF
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  POSITION_INDEPENDENT_CODE ON)

if(MUJOCO_SIMULATION_ENABLE_ASAN)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR
      "AddressSanitizer is only supported for GCC and Clang.")
  endif()
  target_compile_options(mujoco_simulation PRIVATE
    -fsanitize=address
    -fno-omit-frame-pointer)
  target_link_options(mujoco_simulation PRIVATE -fsanitize=address)
endif()

if(MUJOCO_SIMULATION_BUILD_TESTS)
  include(CTest)
  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()

if(MUJOCO_SIMULATION_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(MUJOCO_SIMULATION_INSTALL)
  set(MUJOCO_SIMULATION_CMAKE_INSTALL_DIR
    "${CMAKE_INSTALL_LIBDIR}/cmake/mujoco_simulation")

  install(TARGETS mujoco_simulation
    EXPORT mujoco_simulationTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/mujoco_simulation
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING
      PATTERN "*.hpp"
      PATTERN "*.h"
      PATTERN "*.in" EXCLUDE)
  install(FILES
    "${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}/mujoco_simulation/version.hpp"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation)

  configure_package_config_file(
    cmake/mujoco_simulationConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simulationConfig.cmake"
    INSTALL_DESTINATION ${MUJOCO_SIMULATION_CMAKE_INSTALL_DIR})
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simulationConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
  install(EXPORT mujoco_simulationTargets
    FILE mujoco_simulationTargets.cmake
    NAMESPACE mujoco_simulation::
    DESTINATION ${MUJOCO_SIMULATION_CMAKE_INSTALL_DIR})
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simulationConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simulationConfigVersion.cmake"
    DESTINATION ${MUJOCO_SIMULATION_CMAKE_INSTALL_DIR})
endif()
