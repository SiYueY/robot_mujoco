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

add_executable(mujoco_simulation_mobile_base_component_test
  unit/mobile_base_component_test.cpp)
target_include_directories(mujoco_simulation_mobile_base_component_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(mujoco_simulation_mobile_base_component_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_runtime>)
target_link_libraries(mujoco_simulation_mobile_base_component_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.mobile_base_component
  COMMAND mujoco_simulation_mobile_base_component_test)

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

add_executable(mujoco_simulation_component_manager_sparse_command_test
  unit/component_manager_sparse_command_test.cpp)
target_include_directories(mujoco_simulation_component_manager_sparse_command_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(mujoco_simulation_component_manager_sparse_command_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_runtime>)
target_link_libraries(mujoco_simulation_component_manager_sparse_command_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.component_manager_sparse_command
  COMMAND mujoco_simulation_component_manager_sparse_command_test)

add_executable(mujoco_simulation_joint_gravity_compensation_test
  unit/joint_gravity_compensation_test.cpp)
target_include_directories(mujoco_simulation_joint_gravity_compensation_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(mujoco_simulation_joint_gravity_compensation_test PRIVATE
  $<TARGET_OBJECTS:mujoco_simulation_runtime>)
target_link_libraries(mujoco_simulation_joint_gravity_compensation_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.joint_gravity_compensation
  COMMAND mujoco_simulation_joint_gravity_compensation_test)

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

add_executable(mujoco_simulation_bitmask_test unit/bitmask_test.cpp)
target_link_libraries(mujoco_simulation_bitmask_test PRIVATE mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.bitmask
  COMMAND mujoco_simulation_bitmask_test)

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

add_executable(mujoco_simulation_version_header_contract_test
  public/version_header_contract_test.cpp)
target_link_libraries(mujoco_simulation_version_header_contract_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.version_header_contract
  COMMAND mujoco_simulation_version_header_contract_test)

add_executable(mujoco_simulation_logging_header_contract_test
  public/logging_header_contract_test.cpp)
target_link_libraries(mujoco_simulation_logging_header_contract_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_header_contract
  COMMAND mujoco_simulation_logging_header_contract_test)

add_executable(mujoco_simulation_logging_record_contract_test
  public/logging_record_contract_test.cpp)
target_link_libraries(mujoco_simulation_logging_record_contract_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_record_contract
  COMMAND mujoco_simulation_logging_record_contract_test)

add_executable(mujoco_simulation_logging_frontend_contract_test
  public/logging_frontend_contract_test.cpp)
target_link_libraries(mujoco_simulation_logging_frontend_contract_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_frontend_contract
  COMMAND mujoco_simulation_logging_frontend_contract_test)

add_executable(mujoco_simulation_logging_console_contract_test
  public/logging_console_contract_test.cpp)
target_link_libraries(mujoco_simulation_logging_console_contract_test PRIVATE
  mujoco_simulation::mujoco_simulation util)
add_test(
  NAME mujoco_simulation.logging_console_contract
  COMMAND mujoco_simulation_logging_console_contract_test)

add_executable(mujoco_simulation_logging_record_atomicity_test
  public/logging_record_atomicity_test.cpp)
target_link_libraries(mujoco_simulation_logging_record_atomicity_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_record_atomicity
  COMMAND mujoco_simulation_logging_record_atomicity_test)

add_executable(mujoco_simulation_logging_integration_test
  public/logging_integration_test.cpp)
target_link_libraries(mujoco_simulation_logging_integration_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_integration
  COMMAND mujoco_simulation_logging_integration_test)

add_executable(mujoco_simulation_logging_compile_gate_test
  public/logging_compile_gate_test.cpp)
target_link_libraries(mujoco_simulation_logging_compile_gate_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_compile_gate
  COMMAND mujoco_simulation_logging_compile_gate_test)

add_executable(mujoco_simulation_logging_invalid_abi_test
  public/logging_invalid_abi_test.cpp)
target_link_libraries(mujoco_simulation_logging_invalid_abi_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_invalid_abi
  COMMAND mujoco_simulation_logging_invalid_abi_test)

add_test(
  NAME mujoco_simulation.logging_compile_failures
  COMMAND ${CMAKE_COMMAND}
    -DCXX=${CMAKE_CXX_COMPILER}
    -DINCLUDE_DIR=${PROJECT_SOURCE_DIR}/include
    -DGENERATED_INCLUDE_DIR=${PROJECT_BINARY_DIR}/generated
    -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/public/logging_compile_fail_source.cpp
    -DINVALID_TOKEN_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/public/logging_invalid_token_source.cpp
    -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/compile_failure_objects
    -P ${CMAKE_CURRENT_SOURCE_DIR}/expect_logging_compile_failure.cmake)

add_executable(mujoco_simulation_logging_default_bootstrap_test
  public/logging_default_bootstrap_test.cpp)
target_link_libraries(mujoco_simulation_logging_default_bootstrap_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_default_bootstrap
  COMMAND mujoco_simulation_logging_default_bootstrap_test)

add_executable(mujoco_simulation_logging_file_contract_test
  public/logging_file_contract_test.cpp)
target_link_libraries(mujoco_simulation_logging_file_contract_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_file_contract
  COMMAND mujoco_simulation_logging_file_contract_test)

add_executable(mujoco_simulation_easylogging_adapter_test
  unit/easylogging_adapter_test.cpp
  ${PROJECT_SOURCE_DIR}/src/log/impl/easylogging_adapter.cpp
  ${PROJECT_SOURCE_DIR}/third_party/easyloggingpp/easylogging++.cc)
target_include_directories(mujoco_simulation_easylogging_adapter_test PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_BINARY_DIR}/generated
  ${PROJECT_SOURCE_DIR}/include/mujoco_simulation
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/third_party/easyloggingpp)
target_compile_features(mujoco_simulation_easylogging_adapter_test PRIVATE cxx_std_17)
target_compile_definitions(mujoco_simulation_easylogging_adapter_test PRIVATE
  ELPP_THREAD_SAFE
  ELPP_DISABLE_DEFAULT_CRASH_HANDLING
  ELPP_NO_DEFAULT_LOG_FILE)
target_link_libraries(mujoco_simulation_easylogging_adapter_test PRIVATE Threads::Threads)
add_test(
  NAME mujoco_simulation.easylogging_adapter
  COMMAND mujoco_simulation_easylogging_adapter_test)

add_executable(mujoco_simulation_logging_concurrency_test
  public/logging_concurrency_test.cpp)
target_link_libraries(mujoco_simulation_logging_concurrency_test PRIVATE
  mujoco_simulation::mujoco_simulation)
add_test(
  NAME mujoco_simulation.logging_concurrency
  COMMAND mujoco_simulation_logging_concurrency_test)

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
  mujoco_simulation.logging_header_contract
  mujoco_simulation.logging_record_contract
  mujoco_simulation.logging_frontend_contract
  mujoco_simulation.logging_console_contract
  mujoco_simulation.logging_record_atomicity
  mujoco_simulation.logging_integration
  mujoco_simulation.logging_compile_gate
  mujoco_simulation.logging_invalid_abi
  mujoco_simulation.logging_default_bootstrap
  mujoco_simulation.logging_file_contract
  mujoco_simulation.easylogging_adapter
  mujoco_simulation.logging_concurrency
  mujoco_simulation.state_buffer_concurrency
  PROPERTIES LABELS tsan)

if(MUJOCO_SIMULATION_ENABLE_TSAN AND
   NOT MUJOCO_SIMULATION_TSAN_RUNTIME_TESTS)
  set_tests_properties(
    mujoco_simulation.command_buffer
    mujoco_simulation.state_buffer_concurrency
    PROPERTIES DISABLED TRUE)
endif()

# Link sanitizer instrumentation into every test executable so it is loaded
# before the shared library in sanitizer builds.
foreach(test_target
    mujoco_simulation_camera_renderer_async_test
    mujoco_simulation_camera_snapshot_tsan_test
    mujoco_simulation_scheduler_realtime_test
    mujoco_simulation_config_timing_test
    mujoco_simulation_component_period_test
    mujoco_simulation_component_manager_camera_reset_test
    mujoco_simulation_component_manager_sparse_command_test
    mujoco_simulation_joint_gravity_compensation_test
    mujoco_simulation_command_buffer_test
    mujoco_simulation_bitmask_test
    mujoco_simulation_lifecycle_test
    mujoco_simulation_component_header_contract_test
    mujoco_simulation_logging_header_contract_test
    mujoco_simulation_logging_record_contract_test
    mujoco_simulation_logging_frontend_contract_test
    mujoco_simulation_logging_console_contract_test
    mujoco_simulation_logging_record_atomicity_test
    mujoco_simulation_logging_integration_test
    mujoco_simulation_logging_compile_gate_test
    mujoco_simulation_logging_invalid_abi_test
    mujoco_simulation_logging_default_bootstrap_test
    mujoco_simulation_logging_file_contract_test
    mujoco_simulation_easylogging_adapter_test
    mujoco_simulation_logging_concurrency_test
    mujoco_simulation_viewer_headless_test
    mujoco_simulation_state_buffer_concurrency_test)
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
