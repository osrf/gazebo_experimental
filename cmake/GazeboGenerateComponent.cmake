################################################################################
# GAZEBO_GENERATE_COMPONENT(_protobuf)
# Generates a component plugin from a protobuf file
# _protobuf is a relative path to a protobuf file
#   It must be a relative path, matching protoc's idea of "canonical path"
# Uses PROTOBUF_PROTOC_EXECUTABLE as path to protoc
# Uses CMAKE_CURRENT_SOURCE_DIR as a protobuf import path
# Uses GAZEBO_COMPONENT_GENERATOR_DIR as a protobuf import path
# Uses Extra arguments as dependencies to the generated files
# sets GAZEBO_GENERATE_COMPONENT_LIBRARY
#   A library that should be installed
#   library is called gazeboComponentX where X is protobuf file name
# sets GAZEBO_GENERATE_COMPONENT_HEADERS
#   a header that should be installed
#   X.api.hh where x is protobuf file name
# sets GAZEBO_GENERATE_COMPONENT_PRIVATE_HEADERS
#   Private headers that should not be installed or used
#   Useful for test code that doesn't want to do a dynamic library load
#   X.storage.hh
#   X.factory.hh

MACRO (GAZEBO_GENERATE_COMPONENT _protobuf)
  get_filename_component(rel_dir ${_protobuf} DIRECTORY)
  get_filename_component(abs_path ${_protobuf} ABSOLUTE)
  get_filename_component(abs_dir ${abs_path} DIRECTORY)
  get_filename_component(comp_name ${_protobuf} NAME_WE)

  # Headers that should be installed
  set(GAZEBO_GENERATE_COMPONENT_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/${rel_dir}/${comp_name}.api.hh
  )
  # Headers that shouldn't be installed
  set(GAZEBO_GENERATE_COMPONENT_PRIVATE_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/${rel_dir}/${comp_name}.storage.hh
  )
  set(gen_hh
    ${GAZEBO_GENERATE_COMPONENT_HEADERS}
    ${GAZEBO_GENERATE_COMPONENT_PRIVATE_HEADERS}
  )
  # Source files to get linked into a shared library
  set(gen_cc
    ${CMAKE_CURRENT_BINARY_DIR}/${rel_dir}/${comp_name}.api.cc
  )

  set_source_files_properties(${gen_hh} ${gen_cc} PROPERTIES GENERATED TRUE)

  # This variable should be set by FindGazeboConfig.cmake
  # If it's not set, then this is the gazebo_experimental repo.
  # Set it to the source directory
  if(NOT DEFINED GAZEBO_COMPONENT_GENERATOR_DIR)
    set(GAZEBO_COMPONENT_GENERATOR_DIR ${CMAKE_SOURCE_DIR}/src/components)
  endif()

  # Generate code using a protobuf plugin
  add_custom_command(
    OUTPUT
      ${gen_hh}
      ${gen_cc}
    DEPENDS
      ${abs_path}
      ${ARGN}
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS
      --PIMPL-CPP_out=${CMAKE_CURRENT_BINARY_DIR}
      --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
      --proto_path=${GAZEBO_COMPONENT_GENERATOR_DIR}
      --plugin=${GAZEBO_COMPONENT_GENERATOR_DIR}/protoc-gen-PIMPL-CPP
      ${abs_path}
    COMMENT "Running protubuf to component code generator for ${_protobuf}"
    VERBATIM
  )

  # Make a library out of the generated code
  set(GAZEBO_GENERATE_COMPONENT_LIBRARY gazeboComponent${comp_name})
  add_library(${GAZEBO_GENERATE_COMPONENT_LIBRARY} SHARED
    ${gen_cc}
  )

  target_compile_definitions(${GAZEBO_GENERATE_COMPONENT_LIBRARY}
    PRIVATE
      BUILDING_COMPONENT_${comp_name}_DLL=1
  )

  target_include_directories(${GAZEBO_GENERATE_COMPONENT_LIBRARY}
    PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}
  )
  target_link_libraries(${GAZEBO_GENERATE_COMPONENT_LIBRARY}
    GazeboECS
    ${IGNITION-MATH_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
  )
ENDMACRO(GAZEBO_GENERATE_COMPONENT)

