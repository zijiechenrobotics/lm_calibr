add_executable(sim_calib_node
  sim_calib_node.cpp
)
target_link_libraries(sim_calib_node
  ROTATION_LIDAR_CALIB
)

add_executable(check_dh_node
  check_dh_node.cpp
)
target_link_libraries(check_dh_node
  ROTATION_LIDAR_CALIB
)

add_executable(calib_node
  calib_node.cpp
)
target_link_libraries(calib_node
  ROTATION_LIDAR_CALIB
)

add_executable(transform_cloud_node
  transform_cloud_node.cpp
)
target_link_libraries(transform_cloud_node
  ROTATION_LIDAR_CALIB
)
