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

add_library(gazeboCZName SHARED CZName.cc)
target_link_libraries(gazeboCZName
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)

add_library(gazeboCZGeometry SHARED CZGeometry.cc)
target_link_libraries(gazeboCZGeometry
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)

add_library(gazeboCZMaterial SHARED CZMaterial.cc)
target_link_libraries(gazeboCZMaterial
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)

add_library(gazeboCZPose SHARED CZPose.cc)
target_link_libraries(gazeboCZPose
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)

gz_install_library(gazeboCZName)
gz_install_library(gazeboCZGeometry)
gz_install_library(gazeboCZMaterial)
gz_install_library(gazeboCZPose)
