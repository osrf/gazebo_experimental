set(sources
  Entity.cc
  EntityComponentDatabase.cc
  EntityQuery.cc
  ComponentFactory.cc
  Manager.cc
  QueryRegistrar.cc
  System.cc
)

add_library(GazeboECS SHARED ${sources})
target_link_libraries(GazeboECS GazeboUtil ${IGNITION-COMMON_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

gz_install_library(GazeboECS)
