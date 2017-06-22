
include_directories(.)


#Include current binary dir for generated headers
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(GAZEBO_GENERATE_COMPONENT_PIMPL-CPP ${CMAKE_SOURCE_DIR}/src/components/protoc-gen-PIMPL-CPP)
set(GAZEBO_GENERATE_COMPONENT_PROTO_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/src/components)

GAZEBO_GENERATE_COMPONENT(components/Fraction.proto
  ${GAZEBO_COMPONENT_GENERATOR_FILES})
set(component_libs ${component_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

GAZEBO_GENERATE_COMPONENT(components/Triplet.proto
  ${GAZEBO_COMPONENT_GENERATOR_FILES})
set(component_libs ${component_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

add_subdirectory(systems)

add_executable(add_divide add_divide.cc)
target_link_libraries(add_divide
  GazeboECS
  DivideAndPrintResult
  ${IGNITION-COMMON_LIBRARIES}
  ${component_libs}
)
