add_subdirectory(ecs)
add_subdirectory(systems)

add_executable(gazebo gazebo_main.cc)
target_link_libraries(gazebo ${gflags_LIBRARIES})