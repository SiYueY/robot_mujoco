cmake_minimum_required(VERSION 3.16)

project(
  romujoco
  VERSION 0.1.0
  DESCRIPTION "RoMuJoCo robotics simulation runtime"
  LANGUAGES CXX
)

if(NOT UNIX OR APPLE)
  message(FATAL_ERROR "romujoco supports Linux only.")
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(ROMUJOCO_IS_TOP_LEVEL ON)
else()
  set(ROMUJOCO_IS_TOP_LEVEL OFF)
endif()

option(ROMUJOCO_BUILD_SHARED
       "Build romujoco as shared library" ON)
option(ROMUJOCO_BUILD_TESTS "Build romujoco tests"
       ${ROMUJOCO_IS_TOP_LEVEL})
option(ROMUJOCO_INSTALL "Generate installation rules"
       ${ROMUJOCO_IS_TOP_LEVEL})
option(ROMUJOCO_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ROMUJOCO_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(ROMUJOCO_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(ROMUJOCO_TSAN_RUNTIME_TESTS
  "Run TSAN-labelled tests when ThreadSanitizer is enabled" ON)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake")

set(ROMUJOCO_RUNTIME_SOURCES
  src/buffer/command_buffer.cpp
  src/buffer/state_buffer.cpp
  src/component/component.cpp
  src/component/component_manager.cpp
  src/component/camera/camera_component.cpp
  src/component/imu/imu_component.cpp
  src/component/joint/joint_component.cpp
  src/component/lidar/lidar_component.cpp
  src/component/mobile_base/mobile_base_component.cpp
  src/component/mobile_base/mobile_base_factory.cpp
  src/component/mobile_base/mecanum/kinematic_mecanum_mobile_base.cpp
  src/component/mobile_base/swerve/dynamic_swerve_mobile_base.cpp
  src/config/simulation_config_parser.cpp
  src/config/simulation_config_validator.cpp
  src/runtime/simulation_runtime.cpp
  src/runtime/simulation_scheduler.cpp)

set(ROMUJOCO_RENDER_SOURCES
  src/render/camera_renderer.cpp
  src/render/camera_render_service_impl.cpp)

set(ROMUJOCO_VIEWER_SOURCES
  src/viewer/simulation_viewer.cpp
  src/viewer/simulate/glfw_adapter.cc
  src/viewer/simulate/glfw_dispatch.cc
  src/viewer/simulate/platform_ui_adapter.cc
  src/viewer/simulate/simulate.cc)

set(ROMUJOCO_VIEWER_BACKEND_SOURCES
  src/viewer/lodepng/lodepng.cpp
)

if(ROMUJOCO_BUILD_SHARED)
  set(ROMUJOCO_LIBRARY_TYPE SHARED)
else()
  set(ROMUJOCO_LIBRARY_TYPE STATIC)
endif()

add_library(romujoco_runtime OBJECT
  ${ROMUJOCO_RUNTIME_SOURCES})
add_library(romujoco_render OBJECT
  ${ROMUJOCO_RENDER_SOURCES})
add_library(romujoco_viewer OBJECT
  ${ROMUJOCO_VIEWER_SOURCES})
add_library(romujoco_tinyxml2 OBJECT third_party/tinyxml2/tinyxml2.cpp)
add_library(romujoco_logging OBJECT
  src/log/impl/easylogging_adapter.cpp
  src/log/impl/logger.cpp
  third_party/easyloggingpp/easylogging++.cc)
add_library(romujoco_viewer_backend OBJECT
  ${ROMUJOCO_VIEWER_BACKEND_SOURCES})

set(ROMUJOCO_GENERATED_INCLUDE_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/generated")
configure_file(
  include/romujoco/version.hpp.in
  "${ROMUJOCO_GENERATED_INCLUDE_DIR}/romujoco/version.hpp"
  @ONLY)

set(ROMUJOCO_PRIVATE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/romujoco
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/viewer/simulate
    ${CMAKE_CURRENT_SOURCE_DIR}/src/viewer/lodepng
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/easyloggingpp
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2)

set(ROMUJOCO_INTERNAL_TARGETS
  romujoco_runtime
  romujoco_render
  romujoco_viewer
  romujoco_tinyxml2
  romujoco_logging
  romujoco_viewer_backend)

foreach(target IN LISTS ROMUJOCO_INTERNAL_TARGETS)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_include_directories(${target} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ROMUJOCO_GENERATED_INCLUDE_DIR}
    ${ROMUJOCO_PRIVATE_INCLUDE_DIRS})
  target_compile_definitions(${target} PRIVATE ROMUJOCO_BUILDING_LIBRARY)
  set_target_properties(${target} PROPERTIES
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES)
endforeach()

target_link_libraries(romujoco_runtime PRIVATE mujoco::mujoco Threads::Threads)
target_link_libraries(romujoco_render PRIVATE mujoco::mujoco Threads::Threads OpenGL::GL OpenGL::EGL ${ROMUJOCO_GLFW_TARGET} ${CMAKE_DL_LIBS})
target_link_libraries(romujoco_viewer PRIVATE mujoco::mujoco Threads::Threads OpenGL::GL OpenGL::EGL ${ROMUJOCO_GLFW_TARGET} ${CMAKE_DL_LIBS})

add_library(romujoco ${ROMUJOCO_LIBRARY_TYPE}
  src/simulation/simulation.cpp
  src/simulation/simulation_lifecycle.cpp
  src/simulation/simulation_runtime.cpp
  src/simulation/simulation_viewer.cpp
  src/simulation/simulation_commands.cpp
  src/simulation/simulation_state.cpp
  src/log/logging.cpp
  $<TARGET_OBJECTS:romujoco_runtime>
  $<TARGET_OBJECTS:romujoco_render>
  $<TARGET_OBJECTS:romujoco_viewer>
  $<TARGET_OBJECTS:romujoco_tinyxml2>
  $<TARGET_OBJECTS:romujoco_logging>
  $<TARGET_OBJECTS:romujoco_viewer_backend>)
add_library(romujoco::romujoco ALIAS romujoco)

generate_export_header(romujoco
  EXPORT_FILE_NAME
    "${ROMUJOCO_GENERATED_INCLUDE_DIR}/romujoco/export.hpp"
  EXPORT_MACRO_NAME ROMUJOCO_PUBLIC
  NO_EXPORT_MACRO_NAME ROMUJOCO_LOCAL
  STATIC_DEFINE ROMUJOCO_STATIC_DEFINE)

target_compile_features(romujoco PUBLIC cxx_std_17)
target_include_directories(romujoco
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${ROMUJOCO_GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${ROMUJOCO_PRIVATE_INCLUDE_DIRS})
target_link_libraries(romujoco
  PUBLIC mujoco::mujoco Threads::Threads OpenGL::GL OpenGL::EGL ${ROMUJOCO_GLFW_TARGET}
  PRIVATE ${CMAKE_DL_LIBS})
target_compile_definitions(romujoco PRIVATE ROMUJOCO_BUILDING_LIBRARY)
target_compile_definitions(romujoco_logging PRIVATE
  ELPP_THREAD_SAFE
  ELPP_DISABLE_DEFAULT_CRASH_HANDLING
  ELPP_NO_DEFAULT_LOG_FILE)

if(NOT ROMUJOCO_BUILD_SHARED)
  target_compile_definitions(romujoco PUBLIC
    ROMUJOCO_STATIC_DEFINE)
endif()

set_target_properties(romujoco PROPERTIES
  OUTPUT_NAME romujoco
  EXPORT_NAME romujoco
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  BUILD_RPATH_USE_ORIGIN ON
  INSTALL_RPATH "$ORIGIN"
  CXX_EXTENSIONS OFF
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  POSITION_INDEPENDENT_CODE ON)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Sanitizers.cmake")

if(ROMUJOCO_BUILD_TESTS)
  include(CTest)
  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()

if(ROMUJOCO_INSTALL)
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Install.cmake")
endif()
