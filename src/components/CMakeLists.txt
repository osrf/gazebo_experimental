include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  )

set(component_files
  gazebo/components/NewPose.proto
)

set(generator_files
  protoc-gen-PIMPL-CPP
  templates/ApiDeclaration.hh.in
  templates/ApiDefinition.cc.in
  templates/Component.cc.in
  templates/Component.hh.in
  templates/Include.hh.in
  templates/StorageField.cc.in
)


# Use a cmake macro that turns protobuf into generated code for components
include(${PROJECT_CMAKE_DIR}/GazeboGenerateComponent.cmake)

# The protoc plugin isn't on the path yet, so set a sneaky variable to make the macro work
set(GAZEBO_GENERATE_COMPONENT_PIMPL-CPP ${CMAKE_CURRENT_SOURCE_DIR}/protoc-gen-PIMPL-CPP)

# Generate code for all of the protobuf files
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/gazebo/components)
foreach(comp_file ${component_files})
  GAZEBO_GENERATE_COMPONENT(${comp_file}
    "${CMAKE_BINARY_DIR}/include/gazebo/components/")
endforeach()

# install macro and protoc plugin so others can use it
install(FILES ${PROJECT_CMAKE_DIR}/GazeboGenerateComponent.cmake
  DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake
  )
install(FILES ${generator_files} DESTINATION ${BIN_INSTALL_DIR})
