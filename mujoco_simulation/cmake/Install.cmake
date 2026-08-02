set(MUJOCO_SIMULATION_CMAKE_INSTALL_DIR
  "${CMAKE_INSTALL_LIBDIR}/cmake/mujoco_simulation")

install(TARGETS mujoco_simulation
  EXPORT mujoco_simulationTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install only the supported facade and value-type contracts. Runtime,
# renderer, viewer, buffer and parser headers deliberately remain private.
install(FILES
  include/mujoco_simulation/simulation.hpp
  include/mujoco_simulation/simulation_status.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation)
install(FILES
  include/mujoco_simulation/component/joint.hpp
  include/mujoco_simulation/component/imu.hpp
  include/mujoco_simulation/component/camera.hpp
  include/mujoco_simulation/component/lidar.hpp
  include/mujoco_simulation/component/mobile_base.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation/component)
install(FILES
  include/mujoco_simulation/config/simulation_config.hpp
  include/mujoco_simulation/config/config_limits.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation/config)
install(FILES include/mujoco_simulation/data/robot_state.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation/data)
install(FILES include/mujoco_simulation/component/component_id.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation/component)
install(FILES
  include/mujoco_simulation/common/math.hpp
  include/mujoco_simulation/common/enum.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation/common)
install(FILES
  "${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}/mujoco_simulation/version.hpp"
  "${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}/mujoco_simulation/export.hpp"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/mujoco_simulationConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simulationConfig.cmake"
  INSTALL_DESTINATION ${MUJOCO_SIMULATION_CMAKE_INSTALL_DIR})
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simulationConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(EXPORT mujoco_simulationTargets
  FILE mujoco_simulationTargets.cmake
  NAMESPACE mujoco_simulation::
  DESTINATION ${MUJOCO_SIMULATION_CMAKE_INSTALL_DIR})
install(FILES
  "${PROJECT_SOURCE_DIR}/cmake/FindMujoco.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simulationConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simulationConfigVersion.cmake"
  DESTINATION ${MUJOCO_SIMULATION_CMAKE_INSTALL_DIR})
