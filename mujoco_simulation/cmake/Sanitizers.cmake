if(MUJOCO_SIMULATION_ENABLE_ASAN AND MUJOCO_SIMULATION_ENABLE_TSAN)
  message(FATAL_ERROR "AddressSanitizer and ThreadSanitizer cannot be enabled together.")
endif()

if(MUJOCO_SIMULATION_ENABLE_ASAN OR MUJOCO_SIMULATION_ENABLE_UBSAN OR
   MUJOCO_SIMULATION_ENABLE_TSAN)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "Sanitizers are only supported for GCC and Clang.")
  endif()
endif()

add_library(mujoco_simulation_sanitizers INTERFACE)

if(MUJOCO_SIMULATION_ENABLE_ASAN)
  target_compile_options(mujoco_simulation_sanitizers INTERFACE
    -fsanitize=address
    -fno-omit-frame-pointer)
  # MuJoCo 3.9's mjsan.h emits a GCC-incompatible attribute placement when
  # __SANITIZE_ADDRESS__ is visible in C++ mode. The library remains fully
  # ASan-instrumented; this only disables MuJoCo's optional stack hooks.
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(mujoco_simulation_sanitizers INTERFACE
      -U__SANITIZE_ADDRESS__)
  endif()
  target_link_options(mujoco_simulation_sanitizers INTERFACE -fsanitize=address)
endif()

if(MUJOCO_SIMULATION_ENABLE_UBSAN)
  target_compile_options(mujoco_simulation_sanitizers INTERFACE
    -fsanitize=undefined
    -fno-omit-frame-pointer)
  target_link_options(mujoco_simulation_sanitizers INTERFACE
    -fsanitize=undefined)
endif()

if(MUJOCO_SIMULATION_ENABLE_TSAN)
  target_compile_options(mujoco_simulation_sanitizers INTERFACE
    -fsanitize=thread
    -fno-omit-frame-pointer)
  target_link_options(mujoco_simulation_sanitizers INTERFACE -fsanitize=thread)
endif()

foreach(target IN LISTS MUJOCO_SIMULATION_INTERNAL_TARGETS)
  target_link_libraries(${target} PRIVATE mujoco_simulation_sanitizers)
endforeach()
target_link_libraries(mujoco_simulation PRIVATE
  $<BUILD_INTERFACE:mujoco_simulation_sanitizers>)
