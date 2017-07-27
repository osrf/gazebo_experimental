include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  .
  )

link_directories(
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-GUI_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
  )

set (plugins
  GuiDisplayImage
  GuiDiagnostics
)

foreach (plugin ${plugins})

  add_library(gazebo${plugin} SHARED ${plugin}.cc)

  target_link_libraries(gazebo${plugin}
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${IGNITION-GUI_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
  )

  install (TARGETS gazebo${plugin} DESTINATION ${GAZEBO_PLUGIN_INSTALL_PATH})

endforeach ()

