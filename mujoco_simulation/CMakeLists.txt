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
include(GenerateExportHeader)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(MUJOCO_SIMULATION_IS_TOP_LEVEL ON)
else()
  set(MUJOCO_SIMULATION_IS_TOP_LEVEL OFF)
endif()

option(MUJOCO_SIMULATION_BUILD_SHARED
       "Build mujoco_simulation as a shared library" ON)
option(MUJOCO_SIMULATION_BUILD_TESTS "Build mujoco_simulation tests"
       ${MUJOCO_SIMULATION_IS_TOP_LEVEL})
option(MUJOCO_SIMULATION_BUILD_EXAMPLES "Build mujoco_simulation examples"
       ${MUJOCO_SIMULATION_IS_TOP_LEVEL})
option(MUJOCO_SIMULATION_INSTALL "Generate installation rules"
       ${MUJOCO_SIMULATION_IS_TOP_LEVEL})
option(MUJOCO_SIMULATION_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(MUJOCO_SIMULATION_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(MUJOCO_SIMULATION_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(MUJOCO_SIMULATION_TSAN_RUNTIME_TESTS
  "Run TSAN-labelled tests when ThreadSanitizer is enabled" ON)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake")

set(MUJOCO_SIMULATION_RUNTIME_SOURCES
  src/buffer/command_buffer.cpp
  src/buffer/state_buffer.cpp
  src/component/component.cpp
  src/component/component_manager.cpp
  src/component/camera/camera_component.cpp
  src/component/imu/imu_component.cpp
  src/component/joint/joint_component.cpp
  src/component/lidar/lidar_component.cpp
  src/component/mobile_base/mobile_base_component.cpp
  src/config/simulation_config_parser.cpp
  src/config/simulation_config_validator.cpp
  src/runtime/simulation_runtime.cpp
  src/runtime/simulation_scheduler.cpp)

set(MUJOCO_SIMULATION_RENDER_SOURCES
  src/render/camera_renderer.cpp
  src/render/camera_render_service_impl.cpp)

set(MUJOCO_SIMULATION_VIEWER_SOURCES
  src/viewer/simulation_viewer.cpp
  src/viewer/simulate/glfw_adapter.cc
  src/viewer/simulate/glfw_dispatch.cc
  src/viewer/simulate/platform_ui_adapter.cc
  src/viewer/simulate/simulate.cc)

set(MUJOCO_SIMULATION_VIEWER_BACKEND_SOURCES
  src/viewer/lodepng/lodepng.cpp
)

if(MUJOCO_SIMULATION_BUILD_SHARED)
  set(MUJOCO_SIMULATION_LIBRARY_TYPE SHARED)
else()
  set(MUJOCO_SIMULATION_LIBRARY_TYPE STATIC)
endif()

set_source_files_properties(
  third_party/easyloggingpp/easylogging++.cc
  PROPERTIES COMPILE_DEFINITIONS AUTO_INITIALIZE_EASYLOGGINGPP)

add_library(mujoco_simulation_runtime OBJECT
  ${MUJOCO_SIMULATION_RUNTIME_SOURCES})
add_library(mujoco_simulation_render OBJECT
  ${MUJOCO_SIMULATION_RENDER_SOURCES})
add_library(mujoco_simulation_viewer OBJECT
  ${MUJOCO_SIMULATION_VIEWER_SOURCES})
add_library(mujoco_simulation_tinyxml2 OBJECT third_party/tinyxml2/tinyxml2.cpp)
add_library(mujoco_simulation_logging OBJECT third_party/easyloggingpp/easylogging++.cc)
add_library(mujoco_simulation_viewer_backend OBJECT
  ${MUJOCO_SIMULATION_VIEWER_BACKEND_SOURCES})

set(MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/generated")
configure_file(
  include/mujoco_simulation/version.hpp.in
  "${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}/mujoco_simulation/version.hpp"
  @ONLY)

set(MUJOCO_SIMULATION_PRIVATE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/viewer/simulate
    ${CMAKE_CURRENT_SOURCE_DIR}/src/viewer/lodepng
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/easyloggingpp
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2)

set(MUJOCO_SIMULATION_INTERNAL_TARGETS
  mujoco_simulation_runtime
  mujoco_simulation_render
  mujoco_simulation_viewer
  mujoco_simulation_tinyxml2
  mujoco_simulation_logging
  mujoco_simulation_viewer_backend)

foreach(target IN LISTS MUJOCO_SIMULATION_INTERNAL_TARGETS)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_include_directories(${target} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}
    ${MUJOCO_SIMULATION_PRIVATE_INCLUDE_DIRS})
  target_compile_definitions(${target} PRIVATE MUJOCO_SIMULATION_BUILDING_LIBRARY)
  set_target_properties(${target} PROPERTIES
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES)
endforeach()

target_link_libraries(mujoco_simulation_runtime PRIVATE mujoco::mujoco Threads::Threads)
target_link_libraries(mujoco_simulation_render PRIVATE mujoco::mujoco Threads::Threads OpenGL::GL OpenGL::EGL ${MUJOCO_SIMULATION_GLFW_TARGET} ${CMAKE_DL_LIBS})
target_link_libraries(mujoco_simulation_viewer PRIVATE mujoco::mujoco Threads::Threads OpenGL::GL OpenGL::EGL ${MUJOCO_SIMULATION_GLFW_TARGET} ${CMAKE_DL_LIBS})

add_library(mujoco_simulation ${MUJOCO_SIMULATION_LIBRARY_TYPE}
  src/facade/simulation.cpp
  $<TARGET_OBJECTS:mujoco_simulation_runtime>
  $<TARGET_OBJECTS:mujoco_simulation_render>
  $<TARGET_OBJECTS:mujoco_simulation_viewer>
  $<TARGET_OBJECTS:mujoco_simulation_tinyxml2>
  $<TARGET_OBJECTS:mujoco_simulation_logging>
  $<TARGET_OBJECTS:mujoco_simulation_viewer_backend>)
add_library(mujoco_simulation::mujoco_simulation ALIAS mujoco_simulation)

generate_export_header(mujoco_simulation
  EXPORT_FILE_NAME
    "${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}/mujoco_simulation/export.hpp"
  EXPORT_MACRO_NAME MUJOCO_SIMULATION_PUBLIC
  NO_EXPORT_MACRO_NAME MUJOCO_SIMULATION_LOCAL
  STATIC_DEFINE MUJOCO_SIMULATION_STATIC_DEFINE)

target_compile_features(mujoco_simulation PUBLIC cxx_std_17)
target_include_directories(mujoco_simulation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${MUJOCO_SIMULATION_PRIVATE_INCLUDE_DIRS})
target_link_libraries(mujoco_simulation
  PUBLIC mujoco::mujoco Threads::Threads OpenGL::GL OpenGL::EGL ${MUJOCO_SIMULATION_GLFW_TARGET}
  PRIVATE ${CMAKE_DL_LIBS})
target_compile_definitions(mujoco_simulation PRIVATE MUJOCO_SIMULATION_BUILDING_LIBRARY)

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

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Sanitizers.cmake")

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
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Install.cmake")
endif()
