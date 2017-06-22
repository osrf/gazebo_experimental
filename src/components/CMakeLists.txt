include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  )

set(component_files
  # gazebo/components/Pose.proto
  # gazebo/components/Geometry.proto
)

set(generator_templates
  templates/Api.cc.in
  templates/ApiComplexDecl.hh.in
  templates/ApiComplexDef.cc.in
  templates/ApiComplexOneofDecl.hh.in
  templates/ApiComplexOneofDef.cc.in
  templates/ApiComplexRepeatedDecl.hh.in
  templates/ApiComplexRepeatedDef.cc.in
  templates/ApiDecl.hh.in
  templates/ApiDef.cc.in
  templates/Api.hh.in
  templates/ApiNestedClassDecl.hh.in
  templates/ApiNestedClassDef.cc.in
  templates/ApiOneofDecl.hh.in
  templates/ApiOneofDef.cc.in
  templates/ApiRepeatedDecl.hh.in
  templates/ApiRepeatedDef.cc.in
  templates/EnumField.hh.in
  templates/Enum.hh.in
  templates/Factory.cc.in
  templates/Factory.hh.in
  templates/Include.hh.in
  templates/StorageClassDecl.hh.in
  templates/StorageClassUnion.hh.in
  templates/StorageField.cc.in
  templates/StorageFieldRepeated.cc.in
  templates/StorageFieldUnion.hh.in
  templates/StorageFieldWithDefault.cc.in
  templates/Storage.hh.in
  templates/UnionFieldCopy.hh.in
  templates/UnionFieldDestruct.hh.in
)

set(generator_scripts
  protoc-gen-PIMPL-CPP
)

set(generator_files
  ${generator_scripts}
  ${generator_templates}
)

# Use a cmake macro that turns protobuf into generated code for components
include(${PROJECT_CMAKE_DIR}/GazeboGenerateComponent.cmake)


# Make a target containing the generator files
add_custom_command(OUTPUT ${generator_files})
add_custom_target(generator_files_target ALL DEPENDS ${generator_files} VERBATIM)


# Generate code for all of the protobuf files
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/gazebo/components)
foreach(comp_file ${component_files})
  GAZEBO_GENERATE_COMPONENT(${comp_file})

  add_dependencies(${GAZEBO_GENERATE_COMPONENT_LIBRARY} generator_files_target)

  foreach(generated_header ${GAZEBO_GENERATE_COMPONENT_HEADERS})
    # Copy headers to build folder
    get_filename_component(hh_name ${generated_header} NAME)
    set (hh_renamed ${CMAKE_BINARY_DIR}/include/gazebo/components/${hh_name})
    add_custom_command(OUTPUT ${hh_renamed}
      COMMAND ${CMAKE_COMMAND} -E copy ${generated_header} ${hh_renamed}
      DEPENDS ${generated_header} generator_files_target
      COMMENT Copying ${generated_header} to ${hh_renamed}
      )
    add_custom_target(hdr_${comp_name}_${hh_name} ALL DEPENDS ${hh_renamed})
    # Install header
    install(TARGETS ${hdr_${comp_name}_${hh_name}} DESTINATION ${INCLUDE_INSTALL_DIR}/gazebo/components/)
  endforeach()

  # Install library
  gz_install_library(${GAZEBO_GENERATE_COMPONENT_LIBRARY})
endforeach()

# install macro, protobuf files, and protoc plugin so others can use it
install(FILES ${PROJECT_CMAKE_DIR}/GazeboGenerateComponent.cmake
  DESTINATION ${SHARE_INSTALL_DIR}/cmake
  )
install(DIRECTORY gazebo DESTINATION ${SHARE_INSTALL_DIR}/component_generator)
install(FILES ${generator_scripts} DESTINATION ${SHARE_INSTALL_DIR}/component_generator)
install(FILES ${generator_templates} DESTINATION ${SHARE_INSTALL_DIR}/component_generator/templates)
