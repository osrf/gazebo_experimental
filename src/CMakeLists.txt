add_subdirectory(ecs)
add_subdirectory(systems)
add_subdirectory(rendering)

add_executable(gazebo gazebo_main.cc)
target_link_libraries(gazebo
  ${gflags_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-GUI_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

gz_install_executable(gazebo)
