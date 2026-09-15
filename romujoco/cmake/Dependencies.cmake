# All runtime, rendering, and viewer layers are built unconditionally.
# MuJoCo is an exact, vendored dependency prepared explicitly by mujoco.sh.
set(
  ROMUJOCO_MUJOCO_PREFIX
  "${PROJECT_SOURCE_DIR}/third_party/mujoco/install"
)

if(NOT EXISTS "${ROMUJOCO_MUJOCO_PREFIX}")
  message(
    FATAL_ERROR
    "Bundled MuJoCo is not installed.\n"
    "\n"
    "Run:\n"
    "  ./scripts/mujoco.sh build"
  )
endif()

find_package(
  mujoco CONFIG REQUIRED
  PATHS "${ROMUJOCO_MUJOCO_PREFIX}"
  NO_DEFAULT_PATH
)

if(NOT TARGET mujoco::mujoco)
  message(
    FATAL_ERROR
    "Bundled MuJoCo package does not provide target mujoco::mujoco."
  )
endif()

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
find_package(glfw3 CONFIG REQUIRED)

if(NOT TARGET OpenGL::GL OR NOT TARGET OpenGL::EGL)
  message(FATAL_ERROR "OpenGL and EGL CMake targets are required.")
endif()

if(TARGET glfw)
  set(ROMUJOCO_GLFW_TARGET glfw)
elseif(TARGET glfw3::glfw)
  set(ROMUJOCO_GLFW_TARGET glfw3::glfw)
else()
  message(FATAL_ERROR "glfw3 was found, but neither target 'glfw' nor 'glfw3::glfw' is available.")
endif()
