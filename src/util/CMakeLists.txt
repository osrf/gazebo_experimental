set(sources
  DiagnosticsManager.cc
)

add_library(GazeboExperimentalUtil SHARED ${sources})
target_link_libraries(GazeboExperimentalUtil
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
)

gz_install_library(GazeboExperimentalUtil)
