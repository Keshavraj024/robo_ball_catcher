cmake_minimum_required(VERSION 3.8)
project(ball_chaser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ball_chaser_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

set(TARGET_LIBRARIES
    rclcpp
    ball_chaser_interface
    geometry_msgs)

add_executable(drive_bot src/drive_bot.cpp)
target_include_directories(drive_bot PRIVATE include)
ament_target_dependencies(drive_bot ${TARGET_LIBRARIES})

install(TARGETS
        drive_bot
        DESTINATION lib/${PROJECT_NAME})

ament_package()
