#!/bin/bash
if ! command -v tmux &> /dev/null
then
    echo "tmux is not installed. Running commands in background instead."
    ros2 launch franka_launch example.launch.py spawn_franka_left:=false spawn_franka_right:=true use_fake_hardware:=false &
    ssh jetson "cd Projects/ros2_ws && bash launch_zed.sh" &
    ros2 launch zed_rig_aggregator_node aggregator.launch.py &
    wait
    exit 0
fi

# Use tmux to open multiple terminals side-by-side
SESSION="robot_run"
tmux new-session -d -s $SESSION "ros2 launch franka_launch example.launch.py spawn_franka_left:=false spawn_franka_right:=true use_fake_hardware:=false"
tmux split-window -h -t $SESSION "ssh jetson 'cd Projects/ros2_ws && bash launch_zed.sh'"
tmux split-window -v -t $SESSION "ros2 launch zed_rig_aggregator_node aggregator.launch.py"

echo "Started robot launch nodes in a tmux session."
if [ -t 1 ]; then
    tmux attach-session -t $SESSION
fi
