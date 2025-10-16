cmake_minimum_required(VERSION 3.8)
project(franka_visualization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(pose_line_node src/pose_line_node.cpp)
ament_target_dependencies(pose_line_node
  rclcpp
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  pose_line_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
