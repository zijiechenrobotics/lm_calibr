cmake_minimum_required(VERSION 3.8)
project(lm_calibr)

cmake_policy(SET CMP0074 NEW)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(development_tools REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# depend
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
include(cmake/glog.cmake)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ---------------- library ----------------
add_library(ROTATION_LIDAR_CALIB SHARED
  src/rotation_lidar_calibration.cpp
  src/adaptive_voxel_map.cpp
)
ament_target_dependencies(
  ROTATION_LIDAR_CALIB
  rclcpp
  livox_ros_driver2
  development_tools
  pcl_conversions
  sensor_msgs
  rosbag2_cpp
)
target_link_libraries(ROTATION_LIDAR_CALIB
  ${PCL_LIBRARIES}
  glog
  yaml-cpp
  TBB::tbb
)

install(
  TARGETS ROTATION_LIDAR_CALIB
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_subdirectory(app)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ROTATION_LIDAR_CALIB)


ament_package()
