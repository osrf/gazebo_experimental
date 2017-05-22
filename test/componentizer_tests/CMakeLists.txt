# These are componentizer tests. They run fairly quickly. They're something
# between a unit test and an integration test, testing one componentizer each.


include_directories(
  ../../src
  ${SDFormat_INCLUDE_DIRS}
)


set(unit_tests
  CZCollidable_TEST.cc
  CZGeometry_TEST.cc
  CZInertial_TEST.cc
  CZMaterial_TEST.cc
  CZName_TEST.cc
  CZPhysicsConfig_TEST.cc
  CZPose_TEST.cc
  CZWorldVelocity_TEST.cc
)


# Loop to take care of linking and test macros
# This makes targets like CZ_CZName_TEST
foreach (src_file ${unit_tests})
  string(REGEX REPLACE "\\.cc" "" BINARY_NAME ${src_file})
  string(REGEX REPLACE "_TEST" "" CZNAME ${BINARY_NAME})
  set(BINARY_NAME CZ_${BINARY_NAME})
  set(CZ_LIB_NAME gazebo${CZNAME})
  add_executable(${BINARY_NAME} ${src_file})
  target_link_libraries(${BINARY_NAME}
    gtest
    gtest_main
    GazeboECS
    GazeboUtil
    ${CZ_LIB_NAME}
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
    )
  if (UNIX)
    # gtest uses pthread on UNIX
    target_link_libraries(${BINARY_NAME} pthread)
  endif()
  add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
    --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
endforeach()
