add_library(DivideAndPrintResult SHARED DivideAndPrintResult.cc)
add_library(AddAndPrintResult SHARED AddAndPrintResult.cc)
add_library(DumbPhysicsPlugin SHARED DumbPhysics.cc)
target_link_libraries(DumbPhysicsPlugin DumbPhysics)


include_directories(${IGNITION-MATH_INCLUDE_DIRS})
add_library(DummyRenderingPlugin SHARED DummyRendering.cc)
target_link_libraries(DummyRenderingPlugin ${IGNITION-MATH_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})
