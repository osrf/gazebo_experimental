set_property(
  SOURCE gazebo_main.cc
  PROPERTY COMPILE_DEFINITIONS
  GAZEBO_PLUGIN_INSTALL_PATH="${GAZEBO_PLUGIN_INSTALL_PATH}"
)

add_subdirectory(util)
add_subdirectory(componentizers)
add_subdirectory(ecs)
add_subdirectory(systems)
add_subdirectory(gui)

include_directories(${SDFormat_INCLUDE_DIRS})

add_executable(gazebo gazebo_main.cc)
target_link_libraries(gazebo
  GazeboECS
  ${gflags_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-GUI_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

if (UNIX)
  target_link_libraries(gazebo
    pthread
    )
endif()

gz_install_executable(gazebo)
