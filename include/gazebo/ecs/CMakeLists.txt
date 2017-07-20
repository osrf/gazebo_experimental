include (${project_cmake_dir}/Utils.cmake)

set (headers
  ComponentFactory.hh
  Componentizer.hh
  Entity.hh
  EntityQuery.hh
  EntityComponentDatabase.hh
  Manager.hh
  QueryRegistrar.hh
  System.hh
)

gz_install_includes("ecs" ${headers})

