find_package(ignition-math3 REQUIRED)
include_directories(${IGNITION-MATH_INCLUDE_DIRS} ${IGNITION-TRANSPORT_INCLUDE_DIRS} .)

add_subdirectory(dumb_physics)
add_subdirectory(systems)

add_executable(dummy_demo dummy_demo.cc)
target_link_libraries(dummy_demo
  GazeboECS
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  gazeboComponentInertial
  gazeboComponentGeometry
  gazeboComponentMaterial
  gazeboComponentPose
  gazeboComponentWorldVelocity
)

