
include_directories(.)
add_subdirectory(systems)

add_executable(add_divide add_divide.cc)
target_link_libraries(add_divide GazeboECS DivideAndPrintResult ${IGNITION-COMMON_LIBRARIES})

