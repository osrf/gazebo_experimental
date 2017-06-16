include (${PROJECT_CMAKE_DIR}/Utils.cmake)

################################################################################
# GFlags
find_library(gflags_LIBRARIES NAMES gflags)
find_path(gflags_INCLUDE_DIRS gflags/gflags.h ENV CPATH)
if (NOT gflags_LIBRARIES OR NOT gflags_INCLUDE_DIRS)
  BUILD_ERROR("Missing: GFlags (libgflags-dev)")
else()
  message (STATUS "Found GFlags")
  include_directories(${gflags_INCLUDE_DIRS})
  link_directories(${gflags_LIBRARY_DIRS})
endif()

################################################################################
# Ignition common
find_package(ignition-common0 QUIET)
if (NOT ignition-common0_FOUND)
  BUILD_ERROR ("Missing: Ignition Common (libignition-common0-dev)")
else()
  message (STATUS "Found Ignition Common")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-COMMON_CXX_FLAGS}")
  include_directories(${IGNITION-COMMON_INCLUDE_DIRS})
  link_directories(${IGNITION-COMMON_LIBRARY_DIRS})
endif()

################################################################################
# Ignition GUI
find_package(ignition-gui0 QUIET)
if (NOT ignition-gui0_FOUND)
  BUILD_ERROR ("Missing: Ignition GUI (libignition-gui0-dev)")
else()
  message (STATUS "Found Ignition GUI")
  set (CMAKE_AUTOMOC ON)
  find_package (Qt5Widgets REQUIRED)
  find_package (Qt5Core REQUIRED)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-GUI_CXX_FLAGS}")
  include_directories(
    ${IGNITION-GUI_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
  )
  link_directories(${IGNITION-GUI_LIBRARY_DIRS})
endif()

################################################################################
# Ignition msgs
find_package(ignition-msgs0 QUIET)
if (NOT ignition-msgs0_FOUND)
  BUILD_ERROR ("Missing: Ignition msgs (libignition-msgs0-dev)")
else()
  message (STATUS "Found Ignition msgs")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-MSGS_CXX_FLAGS}")
  include_directories(${IGNITION-MSGS_INCLUDE_DIRS})
  link_directories(${IGNITION-MSGS_LIBRARY_DIRS})
endif()

################################################################################
# Ignition transport
find_package(ignition-transport3 QUIET)
if (NOT ignition-transport3_FOUND)
  BUILD_ERROR ("Missing: Ignition Transport (libignition-transport3-dev)")
else()
  message (STATUS "Found Ignition Transport")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
  include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
  link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})
endif()

################################################################################
# Ignition rendering
find_package(ignition-rendering0 QUIET)
if (NOT ignition-rendering0_FOUND)
  BUILD_ERROR ("Missing: Ignition Rendering (libignition-rendering0-dev)")
else()
  message (STATUS "Found Ignition Rendering")
  include_directories(${IGNITION-RENDERING_INCLUDE_DIRS})
  link_directories(${IGNITION-RENDERING_LIBRARY_DIRS})
endif()

################################################################################
# Find SDFormat
set (SDFormat_MIN_VERSION 6.0.0)
find_package(SDFormat ${SDFormat_MIN_VERSION})

if (NOT SDFormat_FOUND)
  message (STATUS "Looking for SDFormat - not found")
  BUILD_ERROR ("Missing: SDF version >=${SDFormat_MIN_VERSION}. Required for reading and writing SDF files.")
else()
  message (STATUS "Looking for SDFormat - found")
endif()
