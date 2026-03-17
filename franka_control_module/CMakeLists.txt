cmake_minimum_required(VERSION 3.8)
project(franka_control_module)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(franka_control_module src/franka_control_module.cpp)

ament_target_dependencies(franka_control_module
  rclcpp
  trajectory_msgs
  sensor_msgs
)

install(TARGETS
  franka_control_module
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(franka_cartesian_module src/franka_cartesian_module.cpp)

ament_target_dependencies(franka_cartesian_module
  rclcpp
  trajectory_msgs
  sensor_msgs
  pinocchio
  tf2_eigen
)

install(TARGETS
  franka_cartesian_module
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)


ament_package()
