include (${PROJECT_CMAKE_DIR}/Utils.cmake)

########################################
# GFlags
find_library(gflags_LIBRARIES NAMES gflags)
find_path(gflags_INCLUDE_DIRS gflags/gflags.h ENV CPATH)
if (NOT gflags_LIBRARIES OR NOT gflags_INCLUDE_DIRS)
  BUILD_ERROR("GFlags package is missing.")
endif()

########################################
# Ignition common
find_package(ignition-common0 REQUIRED)
include_directories(${IGNITION-COMMON_INCLUDE_DIRS})