################################################################################
# GAZEBO_GENERATE_COMPONENT(_protobuf _outputDir)
# Generates a component plugin from a protobuf file
# _protobuf is a relative path to a protobuf file
#   It must be a relative path, matching protoc's idea of "cannonical path"
# _outputDir is where the header file for the component will be saved
# creates a library called gazeboComponentX where X is protobuf top level message name
MACRO (GAZEBO_GENERATE_COMPONENT _protobuf _outputDir)
  file(TO_CMAKE_PATH _outputDir _outputDir)
  get_filename_component(rel_dir ${_protobuf} DIRECTORY)
  get_filename_component(abs_path ${_protobuf} ABSOLUTE)
  get_filename_component(abs_dir ${abs_path} DIRECTORY)
  get_filename_component(comp_name ${_protobuf} NAME_WE)

  set(gen_pimpl_hh "${CMAKE_CURRENT_BINARY_DIR}/${rel_dir}/${comp_name}.PIMPL-CPP.hh")
  set(gen_pimpl_cc "${CMAKE_CURRENT_BINARY_DIR}/${rel_dir}/${comp_name}.PIMPL-CPP.cc")

  if(DEFINED GAZEBO_GENERATE_COMPONENT_PIMPL-CPP)
    set(extra_protobuf_args --plugin=${GAZEBO_GENERATE_COMPONENT_PIMPL-CPP})
  endif()

  if(DEFINED GAZEBO_GENERATE_COMPONENT_PROTO_INCLUDE_DIRS)
    set(extra_protobuf_args ${extra_protobuf_args} --proto_path=${GAZEBO_GENERATE_COMPONENT_PROTO_INCLUDE_DIRS})
  endif()

  # Generate code using a protobuf plugin
  add_custom_command(
    OUTPUT
      ${gen_pimpl_hh}
      ${gen_pimpl_cc}

    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS
      --PIMPL-CPP_out=${CMAKE_CURRENT_BINARY_DIR}
      --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
      ${extra_protobuf_args}
      ${abs_path}
    DEPENDS ${abs_path}
    COMMENT "Running protubuf to component code generator for ${_protobuf} in ${CMAKE_CURRENT_BINARY_DIR} with ${extra_protobuf_args}"
    VERBATIM)

  # Make a library out of the generated code
  set(lib_target gazeboComponent${comp_name})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  add_library(${lib_target} SHARED
    ${gen_pimpl_cc}
    )
  target_link_libraries(${lib_target}
    GazeboECS
    ${IGNITION-MATH_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
    )
  gz_install_library(${lib_target})


  # Rename and install generated headerfiles
  set(hh_renamed "${_outputDir}/${comp_name}.hh")
  add_custom_command(OUTPUT ${hh_renamed}
    COMMAND ${CMAKE_COMMAND} -E copy ${gen_pimpl_hh} ${hh_renamed}
    DEPENDS ${gen_pimpl_hh}
    COMMENT Copying ${gen_pimpl_hh} to ${hh_renamed}
    )
  add_custom_target(hdr_${comp_name} ALL DEPENDS ${hh_renamed})
  install(TARGETS ${hdr_${comp_name}} DESTINATION ${INCLUDE_INSTALL_DIR}/gazebo/components/)

ENDMACRO(GAZEBO_GENERATE_COMPONENT)

