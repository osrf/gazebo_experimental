include_directories(
  ${ignition-common_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  .
)

link_directories(
  ${IGNITION-MATH_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
)

add_library(gazeboCZName SHARED CZName.cc)
target_link_libraries(gazeboCZName
  GazeboECS
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)
gz_install_library(gazeboCZName)

add_library(gazeboCZGeometry SHARED CZGeometry.cc)
target_link_libraries(gazeboCZGeometry
  GazeboECS
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)
gz_install_library(gazeboCZGeometry)

add_library(gazeboCZMaterial SHARED CZMaterial.cc)
target_link_libraries(gazeboCZMaterial
  GazeboECS
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)
gz_install_library(gazeboCZMaterial)

add_library(gazeboCZPose SHARED CZPose.cc)
target_link_libraries(gazeboCZPose
  GazeboECS
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)
gz_install_library(gazeboCZPose)

add_library(gazeboCZCollidable SHARED CZCollidable.cc)
target_link_libraries(gazeboCZCollidable
  GazeboECS
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)
gz_install_library(gazeboCZCollidable)

add_library(gazeboCZInertial SHARED CZInertial.cc)
target_link_libraries(gazeboCZInertial
  GazeboECS
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)
gz_install_library(gazeboCZInertial)

add_library(gazeboCZPhysicsConfig SHARED CZPhysicsConfig.cc)
target_link_libraries(gazeboCZPhysicsConfig
  GazeboECS
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)
gz_install_library(gazeboCZPhysicsConfig)

add_library(gazeboCZWorldVelocity SHARED CZWorldVelocity.cc)
target_link_libraries(gazeboCZWorldVelocity
  GazeboECS
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${SDFormat_LIBRARIES}
)
gz_install_library(gazeboCZWorldVelocity)
