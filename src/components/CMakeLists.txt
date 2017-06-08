include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  )

set(component_files
  gazebo/components/NewPose.proto
  gazebo/components/NewGeometry.proto
)

set(generator_files
  protoc-gen-PIMPL-CPP
  templates/ApiDeclaration.hh.in
  templates/ApiDefinition.cc.in
  templates/Component.cc.in
  templates/Component.hh.in
  templates/Enum.hh.in
  templates/EnumField.hh.in
  templates/Include.hh.in
  templates/StorageField.cc.in
  templates/StorageFieldWithDefault.cc.in
)


# Use a cmake macro that turns protobuf into generated code for components
include(${PROJECT_CMAKE_DIR}/GazeboGenerateComponent.cmake)

# The protoc plugin isn't on the path yet, so set a sneaky variable to make the macro work
set(GAZEBO_GENERATE_COMPONENT_PIMPL-CPP ${CMAKE_CURRENT_SOURCE_DIR}/protoc-gen-PIMPL-CPP)

# Make a target containing the generator files
add_custom_command(OUTPUT ${generator_files})
add_custom_target(generator_files_target ALL DEPENDS ${generator_files} VERBATIM)

# Generate code for all of the protobuf files
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/gazebo/components)
foreach(comp_file ${component_files})
  GAZEBO_GENERATE_COMPONENT(${comp_file}
    "${CMAKE_BINARY_DIR}/include/gazebo/components/")

  # make libs get regenerated when generator files change
  get_filename_component(no_ext_name ${comp_file} NAME_WE)
  add_dependencies(gazeboComponent${no_ext_name} generator_files_target)
endforeach()

# install macro and protoc plugin so others can use it
install(FILES ${PROJECT_CMAKE_DIR}/GazeboGenerateComponent.cmake
  DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake
  )
install(FILES ${generator_files} DESTINATION ${BIN_INSTALL_DIR})
