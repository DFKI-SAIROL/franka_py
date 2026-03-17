#!/bin/bash
# Move to the root of the frankapy project
cd "$(dirname "$0")/.."

if [ ! -d "crisp_controllers" ]; then
    echo "Cloning crisp_controllers..."
    git clone https://github.com/utiasDSL/crisp_controllers.git ./crisp_controllers
else
    echo "crisp_controllers folder already exists, skipping clone."
fi

if [ ! -d "franka_ros2" ]; then
    echo "Cloning franka_ros2 (humble)..."
    # git clone -b humble https://github.com/frankarobotics/franka_ros2.git ./franka_ros2
    git clone -b ${ROS_DISTRO} https://github.com/frankarobotics/franka_ros2 ./franka_ros2
    # Remove useless ros2 nodes from franka
    rm -rf ./franka_ros2/franka_gazebo ./franka_ros2/franka_fr3_moveit_config ./franka_ros2/franka_mobile_example_controllers ./franka_ros2/franka_mobile_sensors ./franka_ros2/franka_gazebo_bringup
    rm -rf ./franka_ros2/libfranka ./franka_ros2/franka_robot_state_broadcaster
    cd ./franka_ros2/ 
    git clone --recurse-submodules https://git.ias.informatik.tu-darmstadt.de/ros2/franka/libfranka.git
    git clone https://git.ias.informatik.tu-darmstadt.de/ros2/franka/franka_robot_state_broadcaster.git
    cd ..
    vcs import ./franka_ros2 < ./franka_ros2/dependency.repos --recursive --skip-existing
    rosdep install --from-paths ./franka_ros2 --ignore-src --rosdistro humble --skip-keys "ignition-plugin franka_ign_ros2_control" -y
else
    echo "franka_ros2 folder already exists, skipping clone."
fi

if [ ! -d "dynamixel_hardware_interface" ]; then
    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git ./dynamixel_hardware_interface
else
    echo "dynamixel_hardware_interface folder already exists, skipping clone."
fi

if [ ! -d "DynamixelSDK" ]; then
    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/DynamixelSDK.git ./DynamixelSDK
else
    echo "DynamixelSDK folder already exists, skipping clone."
fi

if [ ! -d "dynamixel_interfaces" ]; then
    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git ./dynamixel_interfaces
else
    echo "dynamixel_interfaces folder already exists, skipping clone."
fi
