cmake_minimum_required(VERSION 3.8)
project(development_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)


add_library(development_tools SHARED
  src/tools.cpp
)

target_include_directories(development_tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(development_tools
  rclcpp
  std_msgs
)

target_link_libraries(development_tools
  glog
)

ament_export_include_directories(include)
ament_export_libraries(development_tools)


install(
  TARGETS development_tools
  EXPORT export_development_tools
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()


