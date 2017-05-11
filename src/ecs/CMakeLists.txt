set(sources
  Entity.cc
  EntityComponentDatabase.cc
  EntityQuery.cc
  ComponentFactory.cc
  Componentizer.cc
  Manager.cc
  QueryRegistrar.cc
  System.cc
)

include_directories(${SDFormat_INCLUDE_DIRS})

add_library(GazeboECS SHARED ${sources})
target_link_libraries(GazeboECS ${IGNITION-COMMON_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

gz_install_library(GazeboECS)
