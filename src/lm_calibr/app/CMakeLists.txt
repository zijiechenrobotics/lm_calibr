add_executable(sim_calib_node
  sim_calib_node.cpp
)
target_link_libraries(sim_calib_node
  ROTATION_LIDAR_CALIB
)
