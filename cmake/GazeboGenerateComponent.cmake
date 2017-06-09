################################################################################
# GAZEBO_GENERATE_COMPONENT(_protobuf _outputDir)
# Generates a component plugin from a protobuf file
# _protobuf is a relative path to a protobuf file
#   It must be a relative path, matching protoc's idea of "cannonical path"
# Optionally import proto files from GAZEBO_GENERATE_COMPONENT_PROTO_INCLUDE_DIRS
# Optionally use GAZEBO_GENERATE_COMPONENT_PIMPL-CPP for generator location
# creates a library called gazeboComponentX where X is protobuf top level message name
# creates variable GAZEBO_GENERATE_COMPONENT_HEADERS
# creates variable GAZEBO_GENERATE_COMPONENT_LIBRARY
MACRO (GAZEBO_GENERATE_COMPONENT _protobuf)
  get_filename_component(rel_dir ${_protobuf} DIRECTORY)
  get_filename_component(abs_path ${_protobuf} ABSOLUTE)
  get_filename_component(abs_dir ${abs_path} DIRECTORY)
  get_filename_component(comp_name ${_protobuf} NAME_WE)

  set(GAZEBO_GENERATE_COMPONENT_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/${rel_dir}/${comp_name}.api.hh
    ${CMAKE_CURRENT_BINARY_DIR}/${rel_dir}/${comp_name}.storage.hh
    )
  set(gen_cc
    ${CMAKE_CURRENT_BINARY_DIR}/${rel_dir}/${comp_name}.api.cc
    ${CMAKE_CURRENT_BINARY_DIR}/${rel_dir}/${comp_name}.factory.cc
    )

  if(DEFINED GAZEBO_GENERATE_COMPONENT_PIMPL-CPP)
    set(extra_protobuf_args --plugin=${GAZEBO_GENERATE_COMPONENT_PIMPL-CPP})
  endif()

  if(DEFINED GAZEBO_GENERATE_COMPONENT_PROTO_INCLUDE_DIRS)
    set(extra_protobuf_args ${extra_protobuf_args} --proto_path=${GAZEBO_GENERATE_COMPONENT_PROTO_INCLUDE_DIRS})
  endif()

  # Generate code using a protobuf plugin
  add_custom_command(
    OUTPUT
      ${GAZEBO_GENERATE_COMPONENT_HEADERS}
      ${gen_cc}

    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS
      --PIMPL-CPP_out=${CMAKE_CURRENT_BINARY_DIR}
      --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
      ${extra_protobuf_args}
      ${abs_path}
    DEPENDS ${abs_path}
    COMMENT "Running protubuf to component code generator for ${_protobuf}"
    VERBATIM)

  # Make a library out of the generated code
  set(GAZEBO_GENERATE_COMPONENT_LIBRARY gazeboComponent${comp_name})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  add_library(${GAZEBO_GENERATE_COMPONENT_LIBRARY} SHARED
    ${gen_cc}
    )
  target_link_libraries(${GAZEBO_GENERATE_COMPONENT_LIBRARY}
    GazeboECS
    ${IGNITION-MATH_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
    )
ENDMACRO(GAZEBO_GENERATE_COMPONENT)

