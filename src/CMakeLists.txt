add_subdirectory(util)
add_subdirectory(server)
add_subdirectory(gui)

add_executable(gazebo-experimental main.cc)
target_link_libraries(gazebo-experimental
  GazeboExperimentalServer
  ${gflags_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-GUI_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

if (UNIX)
  target_link_libraries(gazebo-experimental pthread)
endif()

gz_install_executable(gazebo-experimental)
