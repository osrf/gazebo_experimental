include_directories(
  ${ignition-common_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  .
)

link_directories(
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  )

add_library(gazeboNamedElements SHARED NamedElements.cc)
target_link_libraries(gazeboNamedElements
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)

add_library(gazeboCZGeometry SHARED CZGeometry.cc)
target_link_libraries(gazeboCZGeometry
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)

gz_install_library(gazeboNamedElements)
gz_install_library(gazeboCZGeometry)
