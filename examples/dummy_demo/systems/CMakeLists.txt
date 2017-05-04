include_directories(.)
add_library(DumbPhysicsPlugin SHARED DumbPhysics.cc)
target_link_libraries(DumbPhysicsPlugin DumbPhysics)

add_library(DummyRenderingPlugin SHARED DummyRendering.cc)
target_link_libraries(DummyRenderingPlugin GazeboECS ${IGNITION-COMMON_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})
