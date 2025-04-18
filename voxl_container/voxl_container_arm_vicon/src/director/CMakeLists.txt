cmake_minimum_required(VERSION 3.8)
project(director)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_msgs REQUIRED)

add_executable(director_node src/director_node.cpp)
ament_target_dependencies(director_node rclcpp std_msgs px4_msgs geometry_msgs drone_msgs)

# to let ros2 run to find the executable 
install(TARGETS
  director_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
