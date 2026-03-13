cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidar_normal_mode_switch)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        )
