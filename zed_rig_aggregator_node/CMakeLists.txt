cmake_minimum_required(VERSION 3.12)
project(zed_rig_aggregator_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# 2. Executable
add_executable(zed_rig_aggregator_node src/zed_rig_aggregator.cpp)
ament_target_dependencies(zed_rig_aggregator_node
  rclcpp
  sensor_msgs
  custom_interfaces
)

# 3. Install Executable
install(TARGETS zed_rig_aggregator_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Install Config (Sibling to src)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 5. Install Launch Files (Sibling to src)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()