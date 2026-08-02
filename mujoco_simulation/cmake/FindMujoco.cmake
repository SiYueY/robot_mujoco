# Resolve MuJoCo consistently in the build tree and from an installed package.
# The function first accepts a config-package supplied target, then falls back
# to an SDK prefix selected by MUJOCO_ROOT, the environment, or known paths.
function(mujoco_simulation_find_mujoco result_variable)
  if(TARGET mujoco::mujoco)
    set(${result_variable} TRUE PARENT_SCOPE)
    return()
  endif()

  find_package(mujoco CONFIG QUIET)
  if(TARGET mujoco::mujoco)
    set(${result_variable} TRUE PARENT_SCOPE)
    return()
  endif()

  set(MUJOCO_ROOT "" CACHE PATH "MuJoCo installation prefix")
  set(_mujoco_simulation_roots "")
  if(MUJOCO_ROOT)
    list(APPEND _mujoco_simulation_roots "${MUJOCO_ROOT}")
  endif()
  if(DEFINED ENV{MUJOCO_ROOT} AND NOT "$ENV{MUJOCO_ROOT}" STREQUAL "")
    list(APPEND _mujoco_simulation_roots "$ENV{MUJOCO_ROOT}")
  endif()
  list(APPEND _mujoco_simulation_roots /opt/mujoco /opt/mujoco-3.9.0)
  list(REMOVE_DUPLICATES _mujoco_simulation_roots)

  foreach(_mujoco_simulation_root IN LISTS _mujoco_simulation_roots)
    file(TO_CMAKE_PATH "${_mujoco_simulation_root}" _mujoco_simulation_root)
    get_filename_component(_mujoco_simulation_root
      "${_mujoco_simulation_root}" ABSOLUTE)
    if(NOT IS_DIRECTORY "${_mujoco_simulation_root}")
      continue()
    endif()

    find_path(_mujoco_simulation_include_dir
      NAMES mujoco/mujoco.h
      PATHS "${_mujoco_simulation_root}/include"
      NO_DEFAULT_PATH)
    find_library(_mujoco_simulation_library
      NAMES mujoco
      PATHS
        "${_mujoco_simulation_root}/lib"
        "${_mujoco_simulation_root}/lib64"
      NO_DEFAULT_PATH)
    if(_mujoco_simulation_include_dir AND _mujoco_simulation_library)
      add_library(mujoco::mujoco UNKNOWN IMPORTED)
      set_target_properties(mujoco::mujoco PROPERTIES
        IMPORTED_LOCATION "${_mujoco_simulation_library}"
        INTERFACE_INCLUDE_DIRECTORIES "${_mujoco_simulation_include_dir}")
      set(MUJOCO_ROOT "${_mujoco_simulation_root}" CACHE PATH
          "MuJoCo installation prefix" FORCE)
      message(STATUS "MuJoCo root: ${_mujoco_simulation_root}")
      message(STATUS "MuJoCo include: ${_mujoco_simulation_include_dir}")
      message(STATUS "MuJoCo library: ${_mujoco_simulation_library}")
      set(${result_variable} TRUE PARENT_SCOPE)
      return()
    endif()
  endforeach()

  set(${result_variable} FALSE PARENT_SCOPE)
endfunction()
