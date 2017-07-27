
set(plugins
  PhysicsSystem
)

include_directories(
  .
)

foreach (plugin ${plugins})

  add_library(gazebo${plugin} SHARED ${plugin}.cc)

  target_link_libraries(gazebo${plugin}
    GazeboECS
  )

  install (TARGETS gazebo${plugin} DESTINATION ${GAZEBO_PLUGIN_INSTALL_PATH})

endforeach ()

