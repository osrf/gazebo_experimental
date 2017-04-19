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
message (STATUS "\n\n====== Finding 3rd Party Packages ======")

include (${PROJECT_CMAKE_DIR}/FindDependencies.cmake)

message (STATUS "\n\n========================================")
################################################################################
message (STATUS "\n\n====== Building Configuration File =====")

configure_file (${PROJECT_CMAKE_DIR}/Config.hh.in
    "${PROJECT_BINARY_DIR}/include/gazebo/Config.hh")
include_directories(${PROJECT_BINARY_DIR}/include)

message (STATUS "\n\n========================================")
################################################################################

find_package(ignition-common0 REQUIRED)

include_directories(${IGNITION-COMMON_INCLUDE_DIRS})

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(examples)
