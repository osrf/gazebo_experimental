
set(sources
  PhysicsSystem.cc
)

include_directories(
  .
)

add_library(gazeboPhysicsSystem SHARED ${sources})

install(TARGETS gazeboPhysicsSystem LIBRARY DESTINATION lib)
