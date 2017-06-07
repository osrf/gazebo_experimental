include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  )

set(component_files
  NewPose.proto
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

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/gazebo/components)

# Generate code for all of the protobuf files
foreach(comp_file ${component_files})
  get_filename_component(abs_path ${comp_file} ABSOLUTE)
  get_filename_component(comp_name ${comp_file} NAME_WE)

  set(gen_pimpl_hh "${CMAKE_CURRENT_BINARY_DIR}/${comp_name}.PIMPL-CPP.hh")
  set(gen_pimpl_cc "${CMAKE_CURRENT_BINARY_DIR}/${comp_name}.PIMPL-CPP.cc")

  # Generate code using a protobuf plugin
  add_custom_command(
    OUTPUT
      ${gen_pimpl_hh}
      ${gen_pimpl_cc}

    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --plugin=${CMAKE_CURRENT_SOURCE_DIR}/protoc-gen-PIMPL-CPP
    --PIMPL-CPP_out=${CMAKE_CURRENT_BINARY_DIR}
    --proto_path=${CMAKE_CURRENT_SOURCE_DIR} ${abs_path}
    DEPENDS ${abs_path} ${generator_files}
    COMMENT "Running protubuf to component code generator for ${comp_file}"
    VERBATIM)

  # Make a library out of the generated code
  set(lib_target gazeboComponent${comp_name})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  add_library(${lib_target}
      "${CMAKE_CURRENT_BINARY_DIR}/${comp_name}.PIMPL-CPP.cc"
    )
  target_link_libraries(${lib_target}
    GazeboECS
    ${IGNITION-MATH_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
    )
  gz_install_library(${lib_target})


  # Rename and install generated headerfiles
  set(hh_renamed "${CMAKE_BINARY_DIR}/include/gazebo/components/${comp_name}.hh")
  add_custom_command(OUTPUT ${hh_renamed}
    COMMAND ${CMAKE_COMMAND} -E copy ${gen_pimpl_hh} ${hh_renamed}
    DEPENDS ${gen_pimpl_hh}
    COMMENT Copying ${gen_pimpl_hh} to ${hh_renamed}
    )
  add_custom_target(hdr_${comp_name} ALL DEPENDS ${hh_renamed})
  install(TARGETS ${hdr_${comp_name}} DESTINATION ${INCLUDE_INSTALL_DIR}/gazebo/components/)
endforeach()


