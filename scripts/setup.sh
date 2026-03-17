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
    git clone https://github.com/frankarobotics/franka_ros2 ./franka_ros2
    # Remove useless ros2 nodes from franka
    rm -rf ./franka_ros2/franka_gazebo ./franka_ros2/franka_fr3_moveit_config ./franka_ros2/franka_mobile_example_controllers ./franka_ros2/franka_mobile_sensors
    vcs import ./franka_ros2 < ./franka_ros2/dependency.repos --recursive --skip-existing
    rosdep install --from-paths ./franka_ros2 --ignore-src --rosdistro humble --skip-keys "ignition-plugin franka_ign_ros2_control" -y
else
    echo "franka_ros2 folder already exists, skipping clone."
fi
