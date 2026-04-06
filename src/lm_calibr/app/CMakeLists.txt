add_executable(sim_calib_node
  sim_calib_node.cpp
)
target_link_libraries(sim_calib_node
  ROTATION_LIDAR_CALIB
)
ament_target_dependencies(
  sim_calib_node
  rclcpp
  livox_ros_driver2
  development_tools
  pcl_conversions
  sensor_msgs
  rosbag2_cpp
)

install(TARGETS
  sim_calib_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(check_dh_node
  check_dh_node.cpp
)
target_link_libraries(check_dh_node
  ROTATION_LIDAR_CALIB
)
ament_target_dependencies(
  check_dh_node
  rclcpp
  livox_ros_driver2
  development_tools
  pcl_conversions
  sensor_msgs
  rosbag2_cpp
)

install(TARGETS
  check_dh_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(calib_node
  calib_node.cpp
)
target_link_libraries(calib_node
  ROTATION_LIDAR_CALIB
)
ament_target_dependencies(
  calib_node
  rclcpp
  livox_ros_driver2
  development_tools
  pcl_conversions
  sensor_msgs
  rosbag2_cpp
)

install(TARGETS
  calib_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(transform_cloud_node
  transform_cloud_node.cpp
)
target_link_libraries(transform_cloud_node
  ROTATION_LIDAR_CALIB
)
ament_target_dependencies(
  transform_cloud_node
  rclcpp
  livox_ros_driver2
  development_tools
  pcl_conversions
  sensor_msgs
  rosbag2_cpp
)
install(TARGETS
  transform_cloud_node
  DESTINATION lib/${PROJECT_NAME}
)
