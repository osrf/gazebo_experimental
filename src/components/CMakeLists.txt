include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  )

set(component_files
  gazebo/components/Collidable.proto
  gazebo/components/Geometry.proto
  gazebo/components/Inertial.proto
  gazebo/components/Material.proto
  gazebo/components/Name.proto
  gazebo/components/PhysicsConfig.proto
  gazebo/components/Pose.proto
  gazebo/components/WorldVelocity.proto
)

set(generator_files
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
  protoc-gen-PIMPL-CPP
)

foreach(generator_file ${generator_files})
  get_filename_component(abs_path ${generator_file} ABSOLUTE)
  set(abs_path_generator_files ${abs_path} ${abs_path_generator_files})
endforeach()

# Set this globally so protoc unit tests are rebuilt when generator changes
set(GAZEBO_COMPONENT_GENERATOR_FILES ${abs_path_generator_files} CACHE INTERNAL "")

# Use a cmake macro that turns protobuf into generated code for components
include(${PROJECT_CMAKE_DIR}/GazeboGenerateComponent.cmake)

# Generate code for all of the protobuf files
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/gazebo/components)
foreach(comp_file ${component_files})
  GAZEBO_GENERATE_COMPONENT(${comp_file} ${GAZEBO_COMPONENT_GENERATOR_FILES})

  foreach(generated_header ${GAZEBO_GENERATE_COMPONENT_HEADERS})
    # Copy headers to build folder
    get_filename_component(hh_name ${generated_header} NAME)
    set (hh_renamed ${CMAKE_BINARY_DIR}/include/gazebo/components/${hh_name})
    add_custom_command(OUTPUT ${hh_renamed}
      COMMAND ${CMAKE_COMMAND} -E copy ${generated_header} ${hh_renamed}
      DEPENDS ${generated_header}
      COMMENT Copying ${generated_header} to ${hh_renamed}
      )
    add_custom_target(hdr_${comp_name}_${hh_name} ALL DEPENDS ${hh_renamed})
    # Install header
    install(TARGETS ${hdr_${comp_name}_${hh_name}} DESTINATION ${INCLUDE_INSTALL_DIR}/gazebo/components/)
  endforeach()

  foreach(generated_header ${GAZEBO_GENERATE_COMPONENT_PRIVATE_HEADERS})
    # Copy private headers to build folder for use by componentizer tests
    get_filename_component(hh_name ${generated_header} NAME)
    set (hh_renamed ${CMAKE_BINARY_DIR}/include/gazebo/components/${hh_name})
    add_custom_command(OUTPUT ${hh_renamed}
      COMMAND ${CMAKE_COMMAND} -E copy ${generated_header} ${hh_renamed}
      DEPENDS ${generated_header}
      COMMENT Copying ${generated_header} to ${hh_renamed}
      )
    add_custom_target(hdr_${comp_name}_${hh_name} ALL DEPENDS ${hh_renamed})
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

# Make helper libraries
add_library(gazeboPoseHelper PoseHelper.cc)
target_link_libraries(gazeboPoseHelper
  GazeboECS
  gazeboComponentPose)
gz_install_library(gazeboPoseHelper)
