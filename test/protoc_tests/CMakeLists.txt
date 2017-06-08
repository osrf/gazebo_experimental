set(protoc_tests
  SimpleTypes.proto
  SubstitutedTypes.proto
)

include(${CMAKE_SOURCE_DIR}/cmake/GazeboGenerateComponent.cmake)
set(GAZEBO_GENERATE_COMPONENT_PIMPL-CPP ${CMAKE_SOURCE_DIR}/src/components/protoc-gen-PIMPL-CPP)
set(GAZEBO_GENERATE_COMPONENT_PROTO_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/src/components)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include")
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")


foreach(protobuf_file ${protoc_tests})
  GAZEBO_GENERATE_COMPONENT(${protobuf_file}
    "${CMAKE_CURRENT_BINARY_DIR}/include")
  get_filename_component(no_ext ${protobuf_file} NAME_WE)
  set(protoc_test_libs ${protoc_test_libs} gazeboComponent${no_ext})
endforeach()

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
