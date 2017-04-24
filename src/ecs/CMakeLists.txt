set(sources
  Entity.cc
  EntityComponentDatabase.cc
  EntityQuery.cc
  ComponentFactory.cc
  Manager.cc
  System.cc
)

add_library(GazeboECS SHARED ${sources})

gz_install_library(GazeboECS)
