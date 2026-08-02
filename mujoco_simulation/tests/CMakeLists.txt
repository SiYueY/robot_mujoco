add_executable(mujoco_simulation_camera_renderer_async_test
  unit/camera_renderer_async_test.cpp)
target_include_directories(mujoco_simulation_camera_renderer_async_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(mujoco_simulation_camera_renderer_async_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_render>)
target_link_libraries(mujoco_simulation_camera_renderer_async_test PRIVATE
  mujoco_simulation::mujoco_simulation)

add_test(
  NAME mujoco_simulation.camera_renderer_async
  COMMAND mujoco_simulation_camera_renderer_async_test)

add_executable(mujoco_simulation_camera_snapshot_tsan_test
  unit/camera_snapshot_tsan_test.cpp)
target_include_directories(mujoco_simulation_camera_snapshot_tsan_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(mujoco_simulation_camera_snapshot_tsan_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_render>)
target_link_libraries(mujoco_simulation_camera_snapshot_tsan_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.camera_snapshot_tsan
  COMMAND mujoco_simulation_camera_snapshot_tsan_test)

add_executable(mujoco_simulation_scheduler_realtime_test
  unit/simulation_scheduler_realtime_test.cpp)
target_include_directories(mujoco_simulation_scheduler_realtime_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(mujoco_simulation_scheduler_realtime_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_runtime>)
target_link_libraries(mujoco_simulation_scheduler_realtime_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.scheduler_realtime
  COMMAND mujoco_simulation_scheduler_realtime_test)

add_executable(mujoco_simulation_config_timing_test
  unit/simulation_config_timing_test.cpp)
target_include_directories(mujoco_simulation_config_timing_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src)
target_sources(mujoco_simulation_config_timing_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_runtime>)
target_link_libraries(mujoco_simulation_config_timing_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.config_timing
  COMMAND mujoco_simulation_config_timing_test)

add_executable(mujoco_simulation_component_period_test
  unit/simulation_component_period_test.cpp)
target_include_directories(mujoco_simulation_component_period_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src)
target_sources(mujoco_simulation_component_period_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_runtime>)
target_link_libraries(mujoco_simulation_component_period_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.component_period
  COMMAND mujoco_simulation_component_period_test)

add_executable(mujoco_simulation_component_manager_camera_reset_test
  unit/component_manager_camera_reset_test.cpp)
target_include_directories(mujoco_simulation_component_manager_camera_reset_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(mujoco_simulation_component_manager_camera_reset_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_runtime>)
target_link_libraries(mujoco_simulation_component_manager_camera_reset_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.component_manager_camera_reset
  COMMAND mujoco_simulation_component_manager_camera_reset_test)

add_executable(mujoco_simulation_command_buffer_test
  unit/command_buffer_test.cpp)
target_include_directories(mujoco_simulation_command_buffer_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src)
target_sources(mujoco_simulation_command_buffer_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_runtime>)
target_link_libraries(mujoco_simulation_command_buffer_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.command_buffer
  COMMAND mujoco_simulation_command_buffer_test)

add_executable(mujoco_simulation_lifecycle_test
  public/simulation_lifecycle_test.cpp)
target_include_directories(mujoco_simulation_lifecycle_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_link_libraries(mujoco_simulation_lifecycle_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.lifecycle
  COMMAND mujoco_simulation_lifecycle_test)

add_executable(mujoco_simulation_component_header_contract_test
  public/component_header_contract_test.cpp)
target_link_libraries(mujoco_simulation_component_header_contract_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.component_header_contract
  COMMAND mujoco_simulation_component_header_contract_test)

add_executable(mujoco_simulation_viewer_headless_test
  unit/simulation_viewer_headless_test.cpp)
target_include_directories(mujoco_simulation_viewer_headless_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(mujoco_simulation_viewer_headless_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_viewer>
  $<TARGET_OBJECTS:mujoco_simulation_viewer_backend>)
target_link_libraries(mujoco_simulation_viewer_headless_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.viewer_headless
  COMMAND mujoco_simulation_viewer_headless_test)

add_executable(mujoco_simulation_state_buffer_concurrency_test
  unit/state_buffer_concurrency_test.cpp)
target_include_directories(mujoco_simulation_state_buffer_concurrency_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src)
target_sources(mujoco_simulation_state_buffer_concurrency_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_runtime>)
target_link_libraries(mujoco_simulation_state_buffer_concurrency_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.state_buffer_concurrency
  COMMAND mujoco_simulation_state_buffer_concurrency_test)

set_tests_properties(
  mujoco_simulation.camera_snapshot_tsan
  mujoco_simulation.command_buffer
  mujoco_simulation.state_buffer_concurrency
  PROPERTIES LABELS tsan)

if(MUJOCO_SIMULATION_ENABLE_TSAN AND
   NOT MUJOCO_SIMULATION_TSAN_RUNTIME_TESTS)
  set_tests_properties(
    mujoco_simulation.command_buffer
    mujoco_simulation.state_buffer_concurrency
    PROPERTIES DISABLED TRUE)
endif()

# Internal tests intentionally exercise hidden implementation classes.  Supply
# their support objects explicitly instead of widening the installed ABI.
foreach(test_target
    mujoco_simulation_camera_renderer_async_test
    mujoco_simulation_camera_snapshot_tsan_test
    mujoco_simulation_scheduler_realtime_test
    mujoco_simulation_config_timing_test
    mujoco_simulation_component_period_test
    mujoco_simulation_component_manager_camera_reset_test
    mujoco_simulation_command_buffer_test
    mujoco_simulation_lifecycle_test
    mujoco_simulation_component_header_contract_test
    mujoco_simulation_viewer_headless_test
    mujoco_simulation_state_buffer_concurrency_test)
  target_sources(${test_target} PRIVATE
    $<TARGET_OBJECTS:mujoco_simulation_logging>)
  target_link_libraries(${test_target} PRIVATE mujoco_simulation_sanitizers)
endforeach()

target_sources(mujoco_simulation_config_timing_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_tinyxml2>)

if(MUJOCO_SIMULATION_INSTALL)
  if(MUJOCO_SIMULATION_BUILD_SHARED)
    set(MUJOCO_SIMULATION_PACKAGE_CONSUMER_KIND shared)
  else()
    set(MUJOCO_SIMULATION_PACKAGE_CONSUMER_KIND static)
  endif()
  set(MUJOCO_SIMULATION_PACKAGE_CONSUMER_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/package/${MUJOCO_SIMULATION_PACKAGE_CONSUMER_KIND}_consumer")
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/package_consumer.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/package_${MUJOCO_SIMULATION_PACKAGE_CONSUMER_KIND}_consumer.cmake
    @ONLY)
  add_test(
    NAME mujoco_simulation.package_${MUJOCO_SIMULATION_PACKAGE_CONSUMER_KIND}_consumer
    COMMAND ${CMAKE_COMMAND} -P
      ${CMAKE_CURRENT_BINARY_DIR}/package_${MUJOCO_SIMULATION_PACKAGE_CONSUMER_KIND}_consumer.cmake)
endif()
