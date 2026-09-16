add_executable(romujoco_camera_renderer_async_test
  unit/camera_renderer_async_test.cpp)
target_include_directories(romujoco_camera_renderer_async_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(romujoco_camera_renderer_async_test PRIVATE
  $<TARGET_OBJECTS:romujoco_render>)
target_link_libraries(romujoco_camera_renderer_async_test PRIVATE
  romujoco::romujoco)

add_test(
  NAME romujoco.camera_renderer_async
  COMMAND romujoco_camera_renderer_async_test)

add_executable(romujoco_camera_snapshot_tsan_test
  unit/camera_snapshot_tsan_test.cpp)
target_include_directories(romujoco_camera_snapshot_tsan_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(romujoco_camera_snapshot_tsan_test PRIVATE
  $<TARGET_OBJECTS:romujoco_render>)
target_link_libraries(romujoco_camera_snapshot_tsan_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.camera_snapshot_tsan
  COMMAND romujoco_camera_snapshot_tsan_test)

add_executable(romujoco_scheduler_realtime_test
  unit/simulation_scheduler_realtime_test.cpp)
target_include_directories(romujoco_scheduler_realtime_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(romujoco_scheduler_realtime_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_scheduler_realtime_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.scheduler_realtime
  COMMAND romujoco_scheduler_realtime_test)

add_executable(romujoco_config_timing_test
  unit/simulation_config_timing_test.cpp)
target_include_directories(romujoco_config_timing_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src)
target_sources(romujoco_config_timing_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_config_timing_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.config_timing
  COMMAND romujoco_config_timing_test)

add_executable(romujoco_component_period_test
  unit/simulation_component_period_test.cpp)
target_include_directories(romujoco_component_period_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src)
target_sources(romujoco_component_period_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_component_period_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.component_period
  COMMAND romujoco_component_period_test)

add_executable(romujoco_mobile_base_component_test
  unit/mobile_base_component_test.cpp)
target_include_directories(romujoco_mobile_base_component_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(romujoco_mobile_base_component_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_mobile_base_component_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.mobile_base_component
  COMMAND romujoco_mobile_base_component_test)

add_executable(romujoco_dynamic_swerve_mobile_base_test
  unit/dynamic_swerve_mobile_base_test.cpp)
target_include_directories(romujoco_dynamic_swerve_mobile_base_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(romujoco_dynamic_swerve_mobile_base_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_dynamic_swerve_mobile_base_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.dynamic_swerve_mobile_base
  COMMAND romujoco_dynamic_swerve_mobile_base_test)

add_executable(romujoco_component_manager_camera_reset_test
  unit/component_manager_camera_reset_test.cpp)
target_include_directories(romujoco_component_manager_camera_reset_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(romujoco_component_manager_camera_reset_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_component_manager_camera_reset_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.component_manager_camera_reset
  COMMAND romujoco_component_manager_camera_reset_test)

add_executable(romujoco_component_manager_sparse_command_test
  unit/component_manager_sparse_command_test.cpp)
target_include_directories(romujoco_component_manager_sparse_command_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(romujoco_component_manager_sparse_command_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_component_manager_sparse_command_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.component_manager_sparse_command
  COMMAND romujoco_component_manager_sparse_command_test)

add_executable(romujoco_joint_gravity_compensation_test
  unit/joint_gravity_compensation_test.cpp)
target_include_directories(romujoco_joint_gravity_compensation_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(romujoco_joint_gravity_compensation_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_joint_gravity_compensation_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.joint_gravity_compensation
  COMMAND romujoco_joint_gravity_compensation_test)

add_executable(romujoco_command_buffer_test
  unit/command_buffer_test.cpp)
target_include_directories(romujoco_command_buffer_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src)
target_sources(romujoco_command_buffer_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_command_buffer_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.command_buffer
  COMMAND romujoco_command_buffer_test)

add_executable(romujoco_bitmask_test unit/bitmask_test.cpp)
target_link_libraries(romujoco_bitmask_test PRIVATE romujoco::romujoco)
add_test(
  NAME romujoco.bitmask
  COMMAND romujoco_bitmask_test)

add_executable(romujoco_lifecycle_test
  public/simulation_lifecycle_test.cpp)
target_include_directories(romujoco_lifecycle_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_link_libraries(romujoco_lifecycle_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.lifecycle
  COMMAND romujoco_lifecycle_test)

add_executable(romujoco_component_header_contract_test
  public/component_header_contract_test.cpp)
target_link_libraries(romujoco_component_header_contract_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.component_header_contract
  COMMAND romujoco_component_header_contract_test)

add_executable(romujoco_version_header_contract_test
  public/version_header_contract_test.cpp)
target_link_libraries(romujoco_version_header_contract_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.version_header_contract
  COMMAND romujoco_version_header_contract_test)

add_executable(romujoco_logging_header_contract_test
  public/logging_header_contract_test.cpp)
target_link_libraries(romujoco_logging_header_contract_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_header_contract
  COMMAND romujoco_logging_header_contract_test)

add_executable(romujoco_logging_record_contract_test
  public/logging_record_contract_test.cpp)
target_link_libraries(romujoco_logging_record_contract_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_record_contract
  COMMAND romujoco_logging_record_contract_test)

add_executable(romujoco_logging_frontend_contract_test
  public/logging_frontend_contract_test.cpp)
target_link_libraries(romujoco_logging_frontend_contract_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_frontend_contract
  COMMAND romujoco_logging_frontend_contract_test)

add_executable(romujoco_logging_console_contract_test
  public/logging_console_contract_test.cpp)
target_link_libraries(romujoco_logging_console_contract_test PRIVATE
  romujoco::romujoco util)
add_test(
  NAME romujoco.logging_console_contract
  COMMAND romujoco_logging_console_contract_test)

add_executable(romujoco_logging_record_atomicity_test
  public/logging_record_atomicity_test.cpp)
target_link_libraries(romujoco_logging_record_atomicity_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_record_atomicity
  COMMAND romujoco_logging_record_atomicity_test)

add_executable(romujoco_logging_integration_test
  public/logging_integration_test.cpp)
target_link_libraries(romujoco_logging_integration_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_integration
  COMMAND romujoco_logging_integration_test)

add_executable(romujoco_logging_compile_gate_test
  public/logging_compile_gate_test.cpp)
target_link_libraries(romujoco_logging_compile_gate_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_compile_gate
  COMMAND romujoco_logging_compile_gate_test)

add_executable(romujoco_logging_invalid_abi_test
  public/logging_invalid_abi_test.cpp)
target_link_libraries(romujoco_logging_invalid_abi_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_invalid_abi
  COMMAND romujoco_logging_invalid_abi_test)

add_test(
  NAME romujoco.logging_compile_failures
  COMMAND ${CMAKE_COMMAND}
    -DCXX=${CMAKE_CXX_COMPILER}
    -DINCLUDE_DIR=${PROJECT_SOURCE_DIR}/include
    -DGENERATED_INCLUDE_DIR=${PROJECT_BINARY_DIR}/generated
    -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/public/logging_compile_fail_source.cpp
    -DINVALID_TOKEN_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/public/logging_invalid_token_source.cpp
    -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/compile_failure_objects
    -P ${CMAKE_CURRENT_SOURCE_DIR}/expect_logging_compile_failure.cmake)

add_executable(romujoco_logging_default_bootstrap_test
  public/logging_default_bootstrap_test.cpp)
target_link_libraries(romujoco_logging_default_bootstrap_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_default_bootstrap
  COMMAND romujoco_logging_default_bootstrap_test)

add_executable(romujoco_logging_file_contract_test
  public/logging_file_contract_test.cpp)
target_link_libraries(romujoco_logging_file_contract_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_file_contract
  COMMAND romujoco_logging_file_contract_test)

add_executable(romujoco_easylogging_adapter_test
  unit/easylogging_adapter_test.cpp
  ${PROJECT_SOURCE_DIR}/src/log/impl/easylogging_adapter.cpp
  ${PROJECT_SOURCE_DIR}/third_party/easyloggingpp/easylogging++.cc)
target_include_directories(romujoco_easylogging_adapter_test PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_BINARY_DIR}/generated
  ${PROJECT_SOURCE_DIR}/include/romujoco
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/third_party/easyloggingpp)
target_compile_features(romujoco_easylogging_adapter_test PRIVATE cxx_std_17)
target_compile_definitions(romujoco_easylogging_adapter_test PRIVATE
  ELPP_THREAD_SAFE
  ELPP_DISABLE_DEFAULT_CRASH_HANDLING
  ELPP_NO_DEFAULT_LOG_FILE)
target_link_libraries(romujoco_easylogging_adapter_test PRIVATE Threads::Threads)
add_test(
  NAME romujoco.easylogging_adapter
  COMMAND romujoco_easylogging_adapter_test)

add_executable(romujoco_logging_concurrency_test
  public/logging_concurrency_test.cpp)
target_link_libraries(romujoco_logging_concurrency_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.logging_concurrency
  COMMAND romujoco_logging_concurrency_test)

add_executable(romujoco_viewer_headless_test
  unit/simulation_viewer_headless_test.cpp)
target_include_directories(romujoco_viewer_headless_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_sources(romujoco_viewer_headless_test PRIVATE
  $<TARGET_OBJECTS:romujoco_viewer>
  $<TARGET_OBJECTS:romujoco_viewer_backend>)
target_link_libraries(romujoco_viewer_headless_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.viewer_headless
  COMMAND romujoco_viewer_headless_test)

add_executable(romujoco_state_buffer_concurrency_test
  unit/state_buffer_concurrency_test.cpp)
target_include_directories(romujoco_state_buffer_concurrency_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src)
target_sources(romujoco_state_buffer_concurrency_test PRIVATE
  $<TARGET_OBJECTS:romujoco_runtime>)
target_link_libraries(romujoco_state_buffer_concurrency_test PRIVATE
  romujoco::romujoco)
add_test(
  NAME romujoco.state_buffer_concurrency
  COMMAND romujoco_state_buffer_concurrency_test)

set_tests_properties(
  romujoco.camera_snapshot_tsan
  romujoco.command_buffer
  romujoco.logging_header_contract
  romujoco.logging_record_contract
  romujoco.logging_frontend_contract
  romujoco.logging_console_contract
  romujoco.logging_record_atomicity
  romujoco.logging_integration
  romujoco.logging_compile_gate
  romujoco.logging_invalid_abi
  romujoco.logging_default_bootstrap
  romujoco.logging_file_contract
  romujoco.easylogging_adapter
  romujoco.logging_concurrency
  romujoco.state_buffer_concurrency
  PROPERTIES LABELS tsan)

if(ROMUJOCO_ENABLE_TSAN AND
   NOT ROMUJOCO_TSAN_RUNTIME_TESTS)
  set_tests_properties(
    romujoco.command_buffer
    romujoco.state_buffer_concurrency
    PROPERTIES DISABLED TRUE)
endif()

# Link sanitizer instrumentation into every test executable so it is loaded
# before the shared library in sanitizer builds.
foreach(test_target
    romujoco_camera_renderer_async_test
    romujoco_camera_snapshot_tsan_test
    romujoco_scheduler_realtime_test
    romujoco_config_timing_test
    romujoco_component_period_test
    romujoco_component_manager_camera_reset_test
    romujoco_component_manager_sparse_command_test
    romujoco_joint_gravity_compensation_test
    romujoco_command_buffer_test
    romujoco_bitmask_test
    romujoco_lifecycle_test
    romujoco_component_header_contract_test
    romujoco_logging_header_contract_test
    romujoco_logging_record_contract_test
    romujoco_logging_frontend_contract_test
    romujoco_logging_console_contract_test
    romujoco_logging_record_atomicity_test
    romujoco_logging_integration_test
    romujoco_logging_compile_gate_test
    romujoco_logging_invalid_abi_test
    romujoco_logging_default_bootstrap_test
    romujoco_logging_file_contract_test
    romujoco_easylogging_adapter_test
    romujoco_logging_concurrency_test
    romujoco_viewer_headless_test
    romujoco_state_buffer_concurrency_test)
  target_link_libraries(${test_target} PRIVATE romujoco_sanitizers)
endforeach()

target_sources(romujoco_config_timing_test PRIVATE
  $<TARGET_OBJECTS:romujoco_tinyxml2>)

if(ROMUJOCO_INSTALL)
  if(ROMUJOCO_BUILD_SHARED)
    set(ROMUJOCO_PACKAGE_CONSUMER_KIND shared)
  else()
    set(ROMUJOCO_PACKAGE_CONSUMER_KIND static)
  endif()
  set(ROMUJOCO_PACKAGE_CONSUMER_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/package/${ROMUJOCO_PACKAGE_CONSUMER_KIND}_consumer")
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/package_consumer.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/package_${ROMUJOCO_PACKAGE_CONSUMER_KIND}_consumer.cmake
    @ONLY)
  add_test(
    NAME romujoco.package_${ROMUJOCO_PACKAGE_CONSUMER_KIND}_consumer
    COMMAND ${CMAKE_COMMAND} -P
      ${CMAKE_CURRENT_BINARY_DIR}/package_${ROMUJOCO_PACKAGE_CONSUMER_KIND}_consumer.cmake)
endif()
