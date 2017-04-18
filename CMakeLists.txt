cmake_minimum_required(VERSION 2.6)
project(gz_ecs)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(include)

enable_testing()

find_package(ignition-common0 REQUIRED)
include_directories(${IGNITION-COMMON_INCLUDE_DIRS})

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(examples)
