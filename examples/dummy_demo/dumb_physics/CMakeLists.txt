set(sources
  Body.cc
  World.cc
)

find_package(ignition-math3 REQUIRED)
include_directories(${IGNITION-MATH_INCLUDE_DIRS})
add_library(DumbPhysics SHARED ${sources})
target_link_libraries(DumbPhysics GazeboECS ${IGNITION-MATH_LIBRARIES})

