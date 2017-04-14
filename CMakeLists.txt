cmake_minimum_required(VERSION 2.6)
project(gz_ecs)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(include)

enable_testing()


find_package(ignition-common0 REQUIRED)

include_directories(${IGNITION-COMMON_INCLUDE_DIRS})

add_subdirectory(ecs)
add_subdirectory(systems)
add_subdirectory(dumb_physics)
add_subdirectory(test)

add_executable(gz_ecs main.cc)
target_link_libraries(gz_ecs DivideAndPrintResult GazeboECS ${IGNITION-COMMON_LIBRARIES})

find_package(ignition-math3 REQUIRED)
include_directories(${IGNITION-MATH_INCLUDE_DIRS})
add_executable(dummy_demo dummy_demo.cc)
target_link_libraries(dummy_demo GazeboECS ${IGNITION-COMMON_LIBRARIES} ${IGNITION-MATH_LIBRARIES})


# TODO install rules that don't install gazebo/system/*.hh
