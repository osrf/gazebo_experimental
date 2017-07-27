set(protoc_tests
  DefaultValues.proto
  EmptyNested.proto
  EnumType.proto
  NestedMessage.proto
  NestedOneof.proto
  OneofMessage.proto
  RepeatedMessage.proto
  SimpleTypes.proto
  SubstitutedTypes.proto
)

include(${CMAKE_SOURCE_DIR}/cmake/GazeboGenerateComponent.cmake)

include_directories(${CMAKE_CURRENT_BINARY_DIR})


GAZEBO_GENERATE_COMPONENT(DefaultValues.proto
  DEPENDENCIES
    ${GAZEBO_COMPONENT_GENERATOR_FILES}
)
set(protoc_test_libs ${protoc_test_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

GAZEBO_GENERATE_COMPONENT(EmptyNested.proto
  DEPENDENCIES
    ${GAZEBO_COMPONENT_GENERATOR_FILES}
)
set(protoc_test_libs ${protoc_test_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

GAZEBO_GENERATE_COMPONENT(EnumType.proto
  DEPENDENCIES
    ${GAZEBO_COMPONENT_GENERATOR_FILES}
)
set(protoc_test_libs ${protoc_test_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

GAZEBO_GENERATE_COMPONENT(NestedMessage.proto
  DEPENDENCIES
    ${GAZEBO_COMPONENT_GENERATOR_FILES}
  LIBRARIES
    gazeboComponentDefaultValues
)
set(protoc_test_libs ${protoc_test_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

GAZEBO_GENERATE_COMPONENT(NestedOneof.proto
  DEPENDENCIES
    ${GAZEBO_COMPONENT_GENERATOR_FILES}
)
set(protoc_test_libs ${protoc_test_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

GAZEBO_GENERATE_COMPONENT(OneofMessage.proto
  DEPENDENCIES
    ${GAZEBO_COMPONENT_GENERATOR_FILES}
  LIBRARIES
    gazeboComponentDefaultValues
)
set(protoc_test_libs ${protoc_test_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

GAZEBO_GENERATE_COMPONENT(RepeatedMessage.proto
  DEPENDENCIES
    ${GAZEBO_COMPONENT_GENERATOR_FILES}
  LIBRARIES
    gazeboComponentDefaultValues
)
set(protoc_test_libs ${protoc_test_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

GAZEBO_GENERATE_COMPONENT(SimpleTypes.proto
  DEPENDENCIES
    ${GAZEBO_COMPONENT_GENERATOR_FILES}
)
set(protoc_test_libs ${protoc_test_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})

GAZEBO_GENERATE_COMPONENT(SubstitutedTypes.proto
  DEPENDENCIES
    ${GAZEBO_COMPONENT_GENERATOR_FILES}
)
set(protoc_test_libs ${protoc_test_libs} ${GAZEBO_GENERATE_COMPONENT_LIBRARY})


set (src_file pimpl-cpp_TEST.cc)
string(REGEX REPLACE "\\.cc" "" BINARY_NAME ${src_file})
set(BINARY_NAME UNIT_${BINARY_NAME})
add_executable(${BINARY_NAME} ${src_file})
target_link_libraries(${BINARY_NAME}
  gtest
  gtest_main
  GazeboECS
  GazeboUtil
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${protoc_test_libs}
  )
if (UNIX)
  # gtest uses pthread on UNIX
  target_link_libraries(${BINARY_NAME} pthread)
endif()
add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
  --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
