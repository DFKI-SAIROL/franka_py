#/usr/bin/env sh

# Ensure conda libs take precedence over system libs
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

ROS_ENV_FILE="scripts/personal_ros_env.sh"

if [ -f "$ROS_ENV_FILE" ]; then
    echo "$ROS_ENV_FILE already exists. Sourcing it..."
    . "$ROS_ENV_FILE"
else
    if [ "$ROS_DISTRO" = "jazzy" ] || [ "$ROS_DISTRO" = "humble" ]; then
        echo "$ROS_ENV_FILE not found. Using default environment variables..."
        export ROS_DOMAIN_ID=5
        export ROS_LOCALHOST_ONLY=0
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp #rmw_cyclonedds_cpp
    fi
fi

WORKSPACE_SETUP="${PIXI_PROJECT_ROOT:-$(pwd)}/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    . "$WORKSPACE_SETUP"
fi

ros2 daemon stop
ros2 daemon start
