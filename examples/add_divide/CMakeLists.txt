cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo-experimental REQUIRED)
include_directories(./ ${GAZEBO-EXPERIMENTAL_INCLUDE_DIRS})
link_directories(${GAZEBO-EXPERIMENTAL_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO-EXPERIMENTAL_CXX_FLAGS}")

add_subdirectory(systems)

add_executable(add_divide add_divide.cc)
target_link_libraries(add_divide GazeboECS DivideAndPrintResult ${IGNITION-COMMON_LIBRARIES})

