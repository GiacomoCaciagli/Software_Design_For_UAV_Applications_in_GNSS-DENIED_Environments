cmake_minimum_required(VERSION 3.8)
project(control_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_msgs REQUIRED)

add_executable(command_publisher src/command_publisher.cpp)
ament_target_dependencies(command_publisher rclcpp std_msgs drone_msgs)

add_executable(recorder src/recorder.cpp)
ament_target_dependencies(recorder rclcpp rosbag2_cpp std_msgs px4_msgs mocap4r2_msgs geometry_msgs)

add_executable(vicon_coordinate_publisher src/coordinate_publisher.cpp)
ament_target_dependencies(vicon_coordinate_publisher rclcpp std_msgs px4_msgs mocap4r2_msgs)

install(TARGETS
  command_publisher
  recorder
  vicon_coordinate_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
