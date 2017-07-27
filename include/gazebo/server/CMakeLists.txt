include (${project_cmake_dir}/Utils.cmake)

set (headers
  ComponentManager.hh
  Entity.hh
  EntityQuery.hh
  Manager.hh
  QueryRegistrar.hh
  System.hh
  Types.hh
)

set (private_headers
  EntityManager.hh
)

gz_install_includes("server" ${headers})

