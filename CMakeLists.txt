cmake_minimum_required(VERSION 2.6)
project(gz_ecs)

set(CMAKE_CXX_FLAGS "-std=c++11")

set (PROJECT_MAJOR_VERSION 0)
set (PROJECT_MINOR_VERSION 0)
set (PROJECT_PATCH_VERSION 0)
set (PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION})
set (PROJECT_VERSION_FULL
  ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION})

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
  include_directories(${PROJECT_BINARY_DIR}/include)
  add_subdirectory(src)
  add_subdirectory(test)
  add_subdirectory(examples)
endif (build_errors)