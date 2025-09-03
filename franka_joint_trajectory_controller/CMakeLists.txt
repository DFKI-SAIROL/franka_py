cmake_minimum_required(VERSION 3.16)
project(franka_joint_trajectory_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    angles
    control_msgs
    control_toolbox
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    rsl
    tl_expected
    trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(franka_joint_trajectory_controller_parameters
  src/franka_joint_trajectory_controller_parameters.yaml
  include/franka_joint_trajectory_controller/validate_jtc_parameters.hpp
)

add_library(franka_joint_trajectory_controller SHARED
  src/franka_joint_trajectory_controller.cpp
  src/trajectory.cpp
)
target_compile_features(franka_joint_trajectory_controller PUBLIC cxx_std_17)
target_include_directories(franka_joint_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/franka_joint_trajectory_controller>
)
target_link_libraries(franka_joint_trajectory_controller PUBLIC
  franka_joint_trajectory_controller_parameters
)
ament_target_dependencies(franka_joint_trajectory_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(franka_joint_trajectory_controller PRIVATE "JOINT_TRAJECTORY_CONTROLLER_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface joint_trajectory_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/franka_joint_trajectory_controller
)
install(TARGETS
    franka_joint_trajectory_controller
    franka_joint_trajectory_controller_parameters
  EXPORT export_franka_joint_trajectory_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_franka_joint_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
