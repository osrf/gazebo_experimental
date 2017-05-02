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
target_link_libraries(GazeboECS GazeboUtil)

gz_install_library(GazeboECS)
