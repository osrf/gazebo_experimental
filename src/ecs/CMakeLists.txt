set(sources
  Componentizer.cc
  Entity.cc
  EntityComponentDatabase.cc
  EntityQuery.cc
  ComponentFactory.cc
  Manager.cc
  QueryRegistrar.cc
  System.cc
)

include_directories(${SDFormat_INCLUDE_DIRS})

add_library(GazeboECS SHARED ${sources})
target_link_libraries(GazeboECS
  GazeboUtil
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  ${SDFormat_LIBRARIES}
  )

gz_install_library(GazeboECS)
