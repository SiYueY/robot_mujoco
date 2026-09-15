if(ROMUJOCO_ENABLE_ASAN AND ROMUJOCO_ENABLE_TSAN)
  message(FATAL_ERROR "AddressSanitizer and ThreadSanitizer cannot be enabled together.")
endif()

if(ROMUJOCO_ENABLE_ASAN OR ROMUJOCO_ENABLE_UBSAN OR
   ROMUJOCO_ENABLE_TSAN)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "Sanitizers are only supported for GCC and Clang.")
  endif()
endif()

add_library(romujoco_sanitizers INTERFACE)

if(ROMUJOCO_ENABLE_ASAN)
  target_compile_options(romujoco_sanitizers INTERFACE
    -fsanitize=address
    -fno-omit-frame-pointer)
  # MuJoCo 3.9's mjsan.h emits a GCC-incompatible attribute placement when
  # __SANITIZE_ADDRESS__ is visible in C++ mode. The library remains fully
  # ASan-instrumented; this only disables MuJoCo's optional stack hooks.
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(romujoco_sanitizers INTERFACE
      -U__SANITIZE_ADDRESS__)
  endif()
  target_link_options(romujoco_sanitizers INTERFACE -fsanitize=address)
endif()

if(ROMUJOCO_ENABLE_UBSAN)
  target_compile_options(romujoco_sanitizers INTERFACE
    -fsanitize=undefined
    -fno-omit-frame-pointer)
  target_link_options(romujoco_sanitizers INTERFACE
    -fsanitize=undefined)
endif()

if(ROMUJOCO_ENABLE_TSAN)
  target_compile_options(romujoco_sanitizers INTERFACE
    -fsanitize=thread
    -fno-omit-frame-pointer)
  target_link_options(romujoco_sanitizers INTERFACE -fsanitize=thread)
endif()

foreach(target IN LISTS ROMUJOCO_INTERNAL_TARGETS)
  target_link_libraries(${target} PRIVATE romujoco_sanitizers)
endforeach()
target_link_libraries(romujoco PRIVATE
  $<BUILD_INTERFACE:romujoco_sanitizers>)
