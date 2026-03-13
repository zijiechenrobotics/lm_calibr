cmake_minimum_required(VERSION 3.0.2)
project(lm_calibr)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  message_generation
  livox_ros_driver
  development_tools
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)

include(cmake/glog.cmake)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gicp_lio
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(ROTATION_LIDAR_CALIB SHARED
  src/rotation_lidar_calibration.cpp
  src/adaptive_voxel_map.cpp
)
target_link_libraries(ROTATION_LIDAR_CALIB
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  glog
  yaml-cpp
  TBB::tbb
)

# add_subdirectory(test)
add_subdirectory(app)


