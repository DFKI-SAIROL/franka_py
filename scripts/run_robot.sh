#!/bin/bash
if ! command -v tmux &> /dev/null
then
    echo "tmux is not installed. Running commands in background instead."
    ros2 launch franka_launch example.launch.py spawn_franka_left:=false spawn_franka_right:=true use_fake_hardware:=false &
    ssh jetson "cd Projects/ros2_ws && bash launch_zed.sh" &
    ros2 launch zed_rig_aggregator_node aggregator.launch.py &
    ros2 launch franka_safety_layer start_ijk.launch.py spawn_franka_left:=false spawn_franka_right:=true bypass_safety:=true &
    wait
    exit 0
fi

# Use tmux to open multiple terminals side-by-side
SESSION="robot_run"
tmux new-session -d -s $SESSION "pixi run -e humble ros2 launch franka_launch example.launch.py spawn_franka_left:=false spawn_franka_right:=true use_fake_hardware:=false ; exec bash"
tmux split-window -h -t $SESSION "env -i HOME=$HOME USER=$USER /usr/bin/ssh -t jetson 'cd Projects/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && bash launch_zed.sh csil'" # choose config csil|max|"custom"
tmux split-window -v -t $SESSION "pixi run -e humble ros2 launch zed_rig_aggregator_node aggregator.launch.py ; exec bash"
tmux split-window -v -t $SESSION "pixi run -e humble ros2 launch franka_safety_layer start_ijk.launch.py spawn_franka_left:=false spawn_franka_right:=true bypass_safety:=true ; exec bash"

echo "Started robot launch nodes in a tmux session."
tmux attach-session -t $SESSION
