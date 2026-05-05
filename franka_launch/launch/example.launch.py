#  Copyright (c) 2025 Franka Robotics GmbH
#  Modified for Dynamic Configuration Architecture
############################################################################

import os
import sys
import logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_share = get_package_share_directory('franka_launch')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_launch', 'utils')
sys.path.append(os.path.abspath(utils_path))
from launch_utils import load_yaml  # noqa: E402

logging.root.setLevel(logging.INFO)


def generate_robot_nodes(context):
    nodes = []
    # loads dfki_bimanual.yaml
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    print(config_file)

    configs = load_yaml(config_file)

    spawn_robots = []
    if LaunchConfiguration('spawn_franka_left').perform(context).lower() == 'true':
        spawn_robots.append('franka_left')
    if LaunchConfiguration('spawn_franka_right').perform(context).lower() == 'true':
        spawn_robots.append('franka_right')

    for item_name, config in configs.items():
        if item_name in spawn_robots:
            print('Spawn', item_name)
            namespace = config['namespace']

            # check overwrite use_fake_hardware
            use_fake_hardware = config['use_fake_hardware']
            if LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'true':
                use_fake_hardware = 'true'
            if LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'false':
                use_fake_hardware = 'false'

            launch_kwargs = {
                'robot_config': str(config['robot_config']),
                'namespace': str(namespace),
                'robot_ip': str(config['robot_ip']),
                'use_fake_hardware': str(use_fake_hardware),
            }
            if 'end_effector_frame' in config:
                launch_kwargs['end_effector_frame'] = str(config['end_effector_frame'])

            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [FindPackageShare('franka_launch'), 'launch', 'franka.launch.py']
                        )
                    ),
                    launch_arguments=launch_kwargs.items(),
                )
            )
    rviz_file = os.path.join(
        get_package_share_directory('franka_launch'), 'rviz', 'visualize_franka.rviz'
    )
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_file, '-f', 'world'],
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('franka_launch'), 'config', 'dfki_bimanual.yaml']
                ),
                description='Path to the dynamic robot configuration file to load',
            ),
            DeclareLaunchArgument(
                'spawn_franka_left',
                default_value='true',
                description='Spawn franka left',
            ),
            DeclareLaunchArgument(
                'spawn_franka_right',
                default_value='true',
                description='Spawn franka right',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='false',
                description='Overwrite use_fake_hardware from config file',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
