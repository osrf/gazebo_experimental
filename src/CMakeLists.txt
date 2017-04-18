add_subdirectory(ecs)
add_subdirectory(systems)

add_executable(gz_ecs main.cc)
target_link_libraries(gz_ecs DivideAndPrintResult GazeboECS ${IGNITION-COMMON_LIBRARIES})

