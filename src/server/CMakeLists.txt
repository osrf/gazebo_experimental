set(sources
  ComponentManager.cc
  Entity.cc
  EntityManager.cc
  EntityQuery.cc
  Manager.cc
  QueryRegistrar.cc
  System.cc
)

set_property(
  SOURCE Manager.cc
  PROPERTY COMPILE_DEFINITIONS
  GAZEBO_SYSTEM_PLUGIN_PATH="${GAZEBO_SYSTEM_PLUGIN_PATH}"
)

add_subdirectory(systems)
# add_subdirectory(componetizers)


add_library(GazeboExperimentalServer SHARED ${sources})
target_link_libraries(GazeboExperimentalServer
  GazeboExperimentalUtil
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  ${SDFormat_LIBRARIES}
  )

gz_install_library(GazeboExperimentalServer)
