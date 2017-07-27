cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

find_package(gazebo-experimental REQUIRED)
include_directories(./ ${GAZEBO-EXPERIMENTAL_INCLUDE_DIRS})
link_directories(${GAZEBO-EXPERIMENTAL_LIBRARY_DIRS})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO-EXPERIMENTAL_CXX_FLAGS}")

add_subdirectory(dumb_physics)
add_subdirectory(systems)

add_executable(dummy_demo dummy_demo.cc)
target_link_libraries(dummy_demo ${GAZEBO-EXPERIMENTAL_LIBRARIES})

