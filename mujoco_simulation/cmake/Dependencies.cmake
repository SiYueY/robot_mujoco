# All three internal layers are always built, so these dependencies are
# required even when a particular run does not open a viewer.
include("${CMAKE_CURRENT_LIST_DIR}/FindMujoco.cmake")
mujoco_simulation_find_mujoco(MUJOCO_SIMULATION_MUJOCO_FOUND)
if(NOT MUJOCO_SIMULATION_MUJOCO_FOUND)
  message(FATAL_ERROR
    "MuJoCo was not found. Set MUJOCO_ROOT, set the MUJOCO_ROOT environment "
    "variable, install a mujoco config package, or install it under /opt/mujoco "
    "or /opt/mujoco-3.9.0.")
endif()
get_target_property(MUJOCO_SIMULATION_MUJOCO_LIBRARY mujoco::mujoco
  IMPORTED_LOCATION)
if(NOT MUJOCO_SIMULATION_MUJOCO_LIBRARY)
  get_target_property(MUJOCO_SIMULATION_MUJOCO_LIBRARY mujoco::mujoco
    IMPORTED_LOCATION_RELEASE)
endif()
get_filename_component(MUJOCO_SIMULATION_MUJOCO_LIBRARY_DIR
  "${MUJOCO_SIMULATION_MUJOCO_LIBRARY}" DIRECTORY)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
find_package(glfw3 CONFIG REQUIRED)

if(NOT TARGET OpenGL::GL OR NOT TARGET OpenGL::EGL)
  message(FATAL_ERROR "OpenGL and EGL CMake targets are required.")
endif()

if(TARGET glfw)
  set(MUJOCO_SIMULATION_GLFW_TARGET glfw)
elseif(TARGET glfw3::glfw)
  set(MUJOCO_SIMULATION_GLFW_TARGET glfw3::glfw)
else()
  message(FATAL_ERROR
    "glfw3 was found, but neither target 'glfw' nor 'glfw3::glfw' is available.")
endif()
