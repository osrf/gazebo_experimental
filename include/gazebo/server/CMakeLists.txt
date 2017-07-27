include (${project_cmake_dir}/Utils.cmake)

set (headers
  ComponentFactory.hh
  Entity.hh
  EntityQuery.hh
  Manager.hh
  QueryRegistrar.hh
  System.hh
  Types.hh
)

gz_install_includes("server" ${headers})

