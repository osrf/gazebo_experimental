cmake_minimum_required(VERSION 2.6)
project(gz_ecs)

set(CMAKE_CXX_FLAGS "-std=c++11")

set (PROJECT_MAJOR_VERSION 0)
set (PROJECT_MINOR_VERSION 0)
set (PROJECT_PATCH_VERSION 0)

set (PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION})
set (PROJECT_VERSION_FULL
  ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION})

include_directories(include)

enable_testing()

########################################
# Write the config file
set (PROJECT_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake
  CACHE PATH "Location of CMake scripts")
configure_file (${PROJECT_CMAKE_DIR}/Config.hh.in
    "${PROJECT_BINARY_DIR}/include/gazebo/Config.hh")

find_package(ignition-common0 REQUIRED)

include_directories(
    ${IGNITION-COMMON_INCLUDE_DIRS}
    ${gflags_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}/include)

add_subdirectory(ecs)
add_subdirectory(systems)
add_subdirectory(dumb_physics)
add_subdirectory(test)

add_executable(gz_ecs gz_ecs.cc)
target_link_libraries(gz_ecs DivideAndPrintResult GazeboECS ${IGNITION-COMMON_LIBRARIES})

find_package(ignition-math3 REQUIRED)
include_directories(${IGNITION-MATH_INCLUDE_DIRS})
add_executable(dummy_demo dummy_demo.cc)
target_link_libraries(dummy_demo GazeboECS ${IGNITION-COMMON_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

add_executable(gazebo main.cc)
target_link_libraries(gazebo gflags ${IGNITION-COMMON_LIBRARIES})

# TODO install rules that don't install gazebo/system/*.hh
