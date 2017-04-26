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
