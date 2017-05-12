include_directories(
  ${ignition-common_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  .
)

link_directories(
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  )

add_library(gazeboNamedElements SHARED NamedElements.cc)
target_link_libraries(gazeboNamedElements
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)

gz_install_library(gazeboNamedElements)
