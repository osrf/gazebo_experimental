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

add_library(gazeboExperimentalGuiDisplayImage SHARED GuiDisplayImage.cc)

target_link_libraries(gazeboExperimentalGuiDisplayImage
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${IGNITION-GUI_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  )

add_library(gazeboExperimentalGuiDiagnostics SHARED GuiDiagnostics.cc)

target_link_libraries(gazeboExperimentalGuiDiagnostics
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${IGNITION-GUI_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  )

gz_install_library(gazeboExperimentalGuiDisplayImage)
gz_install_library(gazeboExperimentalGuiDiagnostics)
