if(NOT DEFINED CXX OR NOT DEFINED INCLUDE_DIR OR NOT DEFINED GENERATED_INCLUDE_DIR OR
   NOT DEFINED SOURCE OR NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "compile-failure test is missing required parameters")
endif()

file(MAKE_DIRECTORY "${OUTPUT_DIR}")

function(expect_failure definition)
  string(MAKE_C_IDENTIFIER "${definition}" output_name)
  execute_process(
    COMMAND "${CXX}" -std=c++17 "-I${INCLUDE_DIR}" "-I${GENERATED_INCLUDE_DIR}"
            "-D${definition}" -c "${SOURCE}" -o "${OUTPUT_DIR}/${output_name}.o"
    RESULT_VARIABLE result)
  if(result EQUAL 0)
    message(FATAL_ERROR "expected compilation to fail for ${definition}")
  endif()
endfunction()

foreach(definition IN ITEMS
    SIM_LOG_COMPILED_LEVEL=-1
    SIM_LOG_COMPILED_LEVEL=99
    SIM_LOG_COMPILED_LEVEL=INVALID_TOKEN
    SIM_LOG
    SIM_DEBUG
    SIM_INFO
    SIM_WARN
    SIM_ERROR
    SIM_FATAL
    SIM_LOG_LEVEL_DEBUG
    SIM_LOG_LEVEL_INFO
    SIM_LOG_LEVEL_WARN
    SIM_LOG_LEVEL_ERROR
    SIM_LOG_LEVEL_FATAL
    SIM_LOG_LEVEL_NONE
    SIMULATE_LOG
    SIMULATE_LOG_COMPILE_DISABLED
    SIMULATE_LOG_DEBUG
    SIMULATE_LOG_INFO
    SIMULATE_LOG_WARN
    SIMULATE_LOG_ERROR
    SIMULATE_LOG_FATAL)
  expect_failure("${definition}")
endforeach()

execute_process(
  COMMAND "${CXX}" -std=c++17 "-I${INCLUDE_DIR}" "-I${GENERATED_INCLUDE_DIR}"
          -c "${INVALID_TOKEN_SOURCE}" -o "${OUTPUT_DIR}/invalid_severity_token.o"
  RESULT_VARIABLE invalid_token_result)
if(invalid_token_result EQUAL 0)
  message(FATAL_ERROR "expected SIM_LOG(OFF) to fail compilation")
endif()
