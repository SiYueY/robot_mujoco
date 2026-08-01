add_executable(mujoco_simulation_camera_renderer_async_test
  camera_renderer_async_test.cpp)
target_link_libraries(mujoco_simulation_camera_renderer_async_test PRIVATE
  mujoco_simulation::mujoco_simulation)

add_test(
  NAME mujoco_simulation.camera_renderer_async
  COMMAND mujoco_simulation_camera_renderer_async_test)

add_executable(mujoco_simulation_scheduler_realtime_test
  simulation_scheduler_realtime_test.cpp)
target_link_libraries(mujoco_simulation_scheduler_realtime_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.scheduler_realtime
  COMMAND mujoco_simulation_scheduler_realtime_test)

add_executable(mujoco_simulation_config_timing_test
  simulation_config_timing_test.cpp)
target_link_libraries(mujoco_simulation_config_timing_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.config_timing
  COMMAND mujoco_simulation_config_timing_test)

add_executable(mujoco_simulation_component_period_test
  simulation_component_period_test.cpp)
target_link_libraries(mujoco_simulation_component_period_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.component_period
  COMMAND mujoco_simulation_component_period_test)

add_executable(mujoco_simulation_command_bus_test
  command_buffer_test.cpp)
target_link_libraries(mujoco_simulation_command_bus_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.command_bus
  COMMAND mujoco_simulation_command_bus_test)

add_executable(mujoco_simulation_lifecycle_test
  simulation_lifecycle_test.cpp)
target_link_libraries(mujoco_simulation_lifecycle_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.lifecycle
  COMMAND mujoco_simulation_lifecycle_test)

add_executable(mujoco_simulation_viewer_headless_test
  simulation_viewer_headless_test.cpp)
target_link_libraries(mujoco_simulation_viewer_headless_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.viewer_headless
  COMMAND mujoco_simulation_viewer_headless_test)

add_executable(mujoco_simulation_state_buffer_concurrency_test
  state_buffer_concurrency_test.cpp)
target_link_libraries(mujoco_simulation_state_buffer_concurrency_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.state_buffer_concurrency
  COMMAND mujoco_simulation_state_buffer_concurrency_test)

set_tests_properties(
  mujoco_simulation.command_bus
  mujoco_simulation.state_buffer_concurrency
  PROPERTIES LABELS tsan)

if(MUJOCO_SIMULATION_ENABLE_TSAN AND
   NOT MUJOCO_SIMULATION_TSAN_RUNTIME_TESTS)
  set_tests_properties(
    mujoco_simulation.command_bus
    mujoco_simulation.state_buffer_concurrency
    PROPERTIES DISABLED TRUE)
endif()
