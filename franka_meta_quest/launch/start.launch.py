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
            output='screen',
            parameters=[
                {
                    'teleop_config': PathJoinSubstitution([
                        FindPackageShare('franka_meta_quest'), 'config', f'teleop_{"right" if "right" in item_name else "left"}.yaml'
                    ]),
                    'robot_config': PathJoinSubstitution([
                        FindPackageShare('franka_robot_description'), 'config', f'dfki_fr3_{"right" if "right" in item_name else "left"}.yaml'
                    ]),
                }
            ],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        ))
        
        nodes.append(Node(
            package='franka_meta_quest',
            executable='meta_quest_audio_publisher',
            name='oculus_audio_'+item_name,
            namespace=item_name,
            output='screen',
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
        DeclareLaunchArgument('teleop_config', 
                          default_value=PathJoinSubstitution([
                              FindPackageShare('franka_meta_quest'), 'config', 'teleop.yaml'
                          ]),
                          description='Path to the teleop configuration file'),
        OpaqueFunction(function=generate_robot_nodes),
    ])
