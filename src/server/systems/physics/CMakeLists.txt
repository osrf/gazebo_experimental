
set(sources
  Physics.cc
)

include_directories(
  .
)

add_library(physics SHARED ${sources})
target_link_libraries(physics GazeboExperimentalServer)

gz_install_system_plugin(physics)
