cmake_minimum_required(VERSION 3.5)

project(gazebo-experimental)

set (PROJECT_MAJOR_VERSION 0)
set (PROJECT_MINOR_VERSION 0)
set (PROJECT_PATCH_VERSION 1)

set (PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION})
set (PROJECT_VERSION_FULL
  ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION})

string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
set(PROJECT_LIBRARY_TARGET_NAME ${PROJECT_NAME_LOWER})
set(PKG_NAME ${PROJECT_NAME_UPPER})

set (project_cmake_dir ${PROJECT_SOURCE_DIR}/cmake
    CACHE PATH "Location of CMake scripts")

set(CMAKE_CXX_FLAGS "-std=c++11")

set (PROJECT_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake
  CACHE PATH "Location of CMake scripts")
include_directories(include)
enable_testing()

################################################################################

set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)

################################################################################
message (STATUS "====== Finding 3rd Party Packages ======")

include (${PROJECT_CMAKE_DIR}/FindDependencies.cmake)

message (STATUS "========================================")
################################################################################

# Use GNUInstallDirst to get canonical paths
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)


# with -fPIC
if(UNIX AND NOT WIN32)
  set (CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "Install Prefix")
 find_program(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
 if(CMAKE_UNAME)
   exec_program(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
   set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL
     "processor type (i386 and x86_64)")
   if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
     ADD_DEFINITIONS(-fPIC)
   endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
 endif(CMAKE_UNAME)
endif()

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

# developer's option to cache PKG_CONFIG_PATH and
# LD_LIBRARY_PATH for local installs
if(PKG_CONFIG_PATH)
  set (ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
endif()
if(LD_LIBRARY_PATH)
  set (ENV{LD_LIBRARY_PATH} ${LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH})
endif()

set (INCLUDE_INSTALL_DIR
  "${CMAKE_INSTALL_INCLUDEDIR}/gazebo-experimental")
set (LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set (BIN_INSTALL_DIR ${CMAKE_INSTALL_BINDIR})
set (WORLD_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/share/gazebo-${PROJECT_MAJOR_VERSION}/worlds/)

if (build_errors)
  message(STATUS "BUILD ERRORS: These must be resolved before compiling.")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD ERRORS\n")
  message (FATAL_ERROR "Errors encountered in build. "
      "Please see the BUILD ERRORS above.")
else (build_errors)
  configure_file (${PROJECT_CMAKE_DIR}/Config.hh.in
      "${PROJECT_BINARY_DIR}/include/gazebo/Config.hh")

  if(WIN32 AND NOT CYGWIN)
    set(CMAKE_CONFIG_INSTALL_DIR CMake)
  else()
    set(CMAKE_CONFIG_INSTALL_DIR ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME_LOWER}/)
  endif()

  set(cmake_conf_file "${PROJECT_NAME_LOWER}-config.cmake")
  set(cmake_conf_version_file "${PROJECT_NAME_LOWER}-config-version.cmake")
  set(cmake_targets_file "${PROJECT_NAME_LOWER}-config-version.cmake")

  configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME_LOWER}-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}"
    INSTALL_DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
    PATH_VARS LIB_INSTALL_DIR INCLUDE_INSTALL_DIR
    )

  # Use write_basic_package_version_file to generate a ConfigVersion file that
  # allow users of gazebo to specify the API or version to depend on
  # TODO: keep this instruction until deprecate Ubuntu/Precise and update with
  # https://github.com/Kitware/CMake/blob/v2.8.8/Modules/CMakePackageConfigHelpers.cmake
  include(WriteBasicConfigVersionFile)
  write_basic_config_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
    VERSION "${PROJECT_VERSION_FULL}"
    COMPATIBILITY SameMajorVersion)
    
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}
    ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
    DESTINATION ${CMAKE_CONFIG_INSTALL_DIR} COMPONENT cmake)
    
  # Create *-targets.cmake file for build directory
  # TODO: keep this instruction until we depend on CMake 3
  #       then change the first line from TARGETS ...  
  #       to EXPORT ... to match the format used in install(EXPORT
  # export(EXPORT ${PROJECT_LIBRARY_TARGET_NAME}
  #        FILE  ${CMAKE_BINARY_DIR}/${cmake_targets_file})
    
  # Install *-targets.cmake file
  #install(EXPORT ${PROJECT_EXPORT_NAME}
  #        DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
  #        FILE ${cmake_targets_file})

  include_directories(${PROJECT_BINARY_DIR}/include)
  add_subdirectory(include)
  add_subdirectory(src)
  add_subdirectory(test)
  add_subdirectory(worlds)
endif (build_errors)

