
set(sources
  PhysicsSystem.cc
)

include_directories(
  .
)

add_library(gazeboPhysicsSystem SHARED ${sources})
target_link_libraries(gazeboPhysicsSystem GazeboECS)

install(TARGETS gazeboPhysicsSystem LIBRARY DESTINATION lib)
