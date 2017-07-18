include_directories(
  ${ignition-common_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  .
)

link_directories(
  ${IGNITION-MATH_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
)

set (plugins
  CZCollidable
  CZGeometry
  CZInertial
  CZMaterial
  CZName
  CZPhysicsConfig
  CZPose
  CZWorldVelocity
)

foreach (plugin ${plugins})

  add_library(gazebo${plugin} SHARED ${plugin}.cc)

  target_link_libraries(gazebo${plugin}
    GazeboECS
    ${IGNITION-MATH_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
    ${SDFormat_LIBRARIES}
  )

  install (TARGETS gazebo${plugin} DESTINATION ${GAZEBO_PLUGIN_INSTALL_PATH})

endforeach ()
