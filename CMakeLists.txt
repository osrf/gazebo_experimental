cmake_minimum_required(VERSION 2.6)
project(gz_ecs)

set(CMAKE_CXX_FLAGS "-std=c++11")

set (PROJECT_MAJOR_VERSION 0)
set (PROJECT_MINOR_VERSION 0)
set (PROJECT_PATCH_VERSION 0)
set (PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION})
set (PROJECT_VERSION_FULL
  ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION})

set (PROJECT_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake
  CACHE PATH "Location of CMake scripts")
include_directories(include)
enable_testing()

################################################################################

set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)

################################################################################
message (STATUS "====== Finding 3rd Party Packages ======")

include (${PROJECT_CMAKE_DIR}/FindDependencies.cmake)

message (STATUS "========================================")
################################################################################

# Use GNUInstallDirst to get canonical paths
include(GNUInstallDirs)

# with -fPIC
if(UNIX AND NOT WIN32)
  set (CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "Install Prefix")
 find_program(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
 if(CMAKE_UNAME)
   exec_program(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
   set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL
     "processor type (i386 and x86_64)")
   if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
     ADD_DEFINITIONS(-fPIC)
   endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
 endif(CMAKE_UNAME)
endif()

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

# developer's option to cache PKG_CONFIG_PATH and
# LD_LIBRARY_PATH for local installs
if(PKG_CONFIG_PATH)
  set (ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
endif()
if(LD_LIBRARY_PATH)
  set (ENV{LD_LIBRARY_PATH} ${LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH})
endif()

set (INCLUDE_INSTALL_DIR
  "${CMAKE_INSTALL_INCLUDEDIR}/gazebo-${PROJECT_MAJOR_VERSION}/gazebo")
set (LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set (BIN_INSTALL_DIR ${CMAKE_INSTALL_BINDIR})
set (WORLD_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/share/gazebo-${PROJECT_MAJOR_VERSION}/worlds/)

if (build_errors)
  message(STATUS "BUILD ERRORS: These must be resolved before compiling.")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD ERRORS\n")
  message (FATAL_ERROR "Errors encountered in build. "
      "Please see the BUILD ERRORS above.")
else (build_errors)
  configure_file (${PROJECT_CMAKE_DIR}/Config.hh.in
      "${PROJECT_BINARY_DIR}/include/gazebo/Config.hh")
  include_directories(${PROJECT_BINARY_DIR}/include)
  add_subdirectory(src)
  add_subdirectory(test)
  add_subdirectory(examples)
  add_subdirectory(worlds)
endif (build_errors)
