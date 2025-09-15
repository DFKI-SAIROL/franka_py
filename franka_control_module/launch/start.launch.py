from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import yaml

# log launch file with debug
import logging
logging.root.setLevel(logging.INFO)


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_robot_nodes(context):

    nodes = []
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    configs = load_yaml(config_file)

    spawn_robots = []
    if LaunchConfiguration('spawn_franka_main').perform(context).lower() == 'true':
        spawn_robots.append("franka_main")
    if LaunchConfiguration('spawn_franka_left').perform(context).lower() == 'true':
        spawn_robots.append("franka_left")
    if LaunchConfiguration('spawn_franka_right').perform(context).lower() == 'true':
        spawn_robots.append("franka_right")
    
    for item_name, config in configs.items():
        if item_name in spawn_robots: 
            print("Spawn", item_name)

            nodes.append(
                Node(
                    package="franka_control_module",
                    executable="franka_control_module",
                    name="franka_control_module",
                    namespace=config['namespace'],
                    parameters=[{
                        'arm_id': str(config['arm_id']),
                        'arm_prefix': str(config['arm_prefix']),
                        'init_joint_position': config['init_joint_position'],
                    }],
                    output="screen",
                )
            )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_launch'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        DeclareLaunchArgument(
            "spawn_franka_main",
            default_value="false",
            description="Spawn franka main",
        ),
        DeclareLaunchArgument(
            "spawn_franka_left",
            default_value="true",
            description="Spawn franka left",
        ),
        DeclareLaunchArgument(
            "spawn_franka_right",
            default_value="true",
            description="Spawn franka right",
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
