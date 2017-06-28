
set(sources
  PhysicsSystem.cc
)


add_library(gazeboPhysicsSystem SHARED ${sources})
target_link_libraries(gazeboPhysicsSystem
  GazeboECS
  ${BULLET_LIBRARIES}
)
target_include_directories(gazeboPhysicsSystem
  PRIVATE
    ${BULLET_INCLUDE_DIRS}
    .
)

gz_install_library(gazeboPhysicsSystem)
