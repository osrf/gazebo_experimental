set(sources
  DiagnosticsManager.cc
  )

add_library(GazeboUtil SHARED ${sources})
target_link_libraries(GazeboUtil
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  )

gz_install_library(GazeboUtil)
