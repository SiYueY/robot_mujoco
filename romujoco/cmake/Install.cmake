set(
  ROMUJOCO_CMAKE_INSTALL_DIR
  "${CMAKE_INSTALL_LIBDIR}/cmake/romujoco"
)

install(
  TARGETS romujoco
  EXPORT romujocoTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# The staging prefix is a complete MuJoCo SDK. Install its public artifacts
# into the same final prefix rather than exposing a nested third_party layout.
install(
  DIRECTORY "${ROMUJOCO_MUJOCO_PREFIX}/include/"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
  DIRECTORY "${ROMUJOCO_MUJOCO_PREFIX}/lib/"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
if(EXISTS "${ROMUJOCO_MUJOCO_PREFIX}/share")
  install(
    DIRECTORY "${ROMUJOCO_MUJOCO_PREFIX}/share/"
    DESTINATION ${CMAKE_INSTALL_DATADIR}
  )
endif()

install(
  DIRECTORY include/romujoco/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/romujoco
  FILES_MATCHING PATTERN "*.hpp"
)
install(
  FILES
    "${ROMUJOCO_GENERATED_INCLUDE_DIR}/romujoco/version.hpp"
    "${ROMUJOCO_GENERATED_INCLUDE_DIR}/romujoco/export.hpp"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/romujoco
)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/romujocoConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/romujocoConfig.cmake"
  INSTALL_DESTINATION ${ROMUJOCO_CMAKE_INSTALL_DIR}
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/romujocoConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(
  EXPORT romujocoTargets
  FILE romujocoTargets.cmake
  NAMESPACE romujoco::
  DESTINATION ${ROMUJOCO_CMAKE_INSTALL_DIR}
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/romujocoConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/romujocoConfigVersion.cmake"
  DESTINATION ${ROMUJOCO_CMAKE_INSTALL_DIR}
)
