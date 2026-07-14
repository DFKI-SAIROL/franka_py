import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_share = get_package_share_directory('franka_launch')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_launch', 'utils')
sys.path.append(os.path.abspath(utils_path))
from launch_utils import merge_overrides


def generate_robot_nodes(context):

    nodes = []

    overrides_file = LaunchConfiguration('overrides_file').perform(context)

    spawn_robots = []
    if LaunchConfiguration('spawn_franka_main').perform(context).lower() == 'true':
        spawn_robots.append("franka_main")
    if LaunchConfiguration('spawn_franka_left').perform(context).lower() == 'true':
        spawn_robots.append("franka_left")
    if LaunchConfiguration('spawn_franka_right').perform(context).lower() == 'true':
        spawn_robots.append("franka_right")
    
    for item_name in spawn_robots:
        print("Spawn", item_name)

        arm_overrides = merge_overrides({}, overrides_file, item_name)

        node_parameters = {
            'teleop_config': PathJoinSubstitution([
                FindPackageShare('franka_meta_quest'), 'config', f'teleop_{"right" if "right" in item_name else "left"}.yaml'
            ]),
            'robot_config': PathJoinSubstitution([
                FindPackageShare('franka_robot_description'), 'config', f'dfki_fr3_{"right" if "right" in item_name else "left"}.yaml'
            ]),
        }
        if 'end_effector_frame' in arm_overrides:
            node_parameters['end_effector_frame'] = str(arm_overrides['end_effector_frame'])

        nodes.append(Node(
            package='franka_meta_quest',
            executable='oculus_action_main',
            name='oculus_action_'+item_name,
            namespace=item_name,
            output='screen',
            parameters=[node_parameters],
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
        DeclareLaunchArgument(
            'overrides_file',
            default_value='',
            description='Path to a robot_overrides.yaml that overrides per-arm end_effector_frame',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
