from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_robot_nodes(context):

    nodes = []
    
    spawn_robots = []
    if LaunchConfiguration('spawn_franka_main').perform(context).lower() == 'true':
        spawn_robots.append("franka_main")
    if LaunchConfiguration('spawn_franka_left').perform(context).lower() == 'true':
        spawn_robots.append("franka_left")
    if LaunchConfiguration('spawn_franka_right').perform(context).lower() == 'true':
        spawn_robots.append("franka_right")
    
    for item_name in spawn_robots: 
        print("Spawn", item_name)

        nodes.append(Node(
            package='franka_meta_quest',
            executable='oculus_action_main',
            name='oculus_action_'+item_name,
            namespace=item_name,
            output='screen'
        ))
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
            default_value="false",
            description="Spawn franka left",
        ),
        DeclareLaunchArgument(
            "spawn_franka_right",
            default_value="true",
            description="Spawn franka right",
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
