#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import sys
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Add the path to the `utils` folder
package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
sys.path.append(os.path.abspath(utils_path))

from launch_utils import load_yaml  # noqa: E402


def generate_robot_nodes(context):
    nodes = []
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    configs = load_yaml(config_file)
    controller_name = LaunchConfiguration('controller_name').perform(context)

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
            namespace = config['namespace']

            # check overwrite use_fake_hardware
            use_fake_hardware = config['use_fake_hardware']
            if LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'true':
                use_fake_hardware = 'true'
            if LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'false':
                use_fake_hardware = 'false'

            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('franka_launch'), 'launch', 'franka.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'arm_id': str(config['arm_id']),
                        'arm_prefix': str(config['arm_prefix']),
                        'namespace': str(namespace),
                        'urdf_file': str(config['urdf_file']),
                        'srdf_file': str(config['srdf_file']),
                        'robot_ip': str(config['robot_ip']),
                        'load_gripper': str(config['load_gripper']),
                        'use_fake_hardware': str(use_fake_hardware),
                        'fake_sensor_commands': str(config['fake_sensor_commands']),
                        'joint_state_rate': str(config['joint_state_rate']),
                        'xyz': str(config['xyz']),
                        'rpy': str(config['rpy']),
                    }.items(),
                )
            )

               
            # Define the additional moveit nodes
            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            PathJoinSubstitution(
                                [
                                    FindPackageShare('franka_fr3_moveit_config'),
                                    'launch',
                                    'move_group.launch.py',
                                ]
                            )
                        ]
                    ),
                    launch_arguments={
                        'robot_ip': str(config['robot_ip']),
                        'namespace': str(config['namespace']),
                        'load_gripper': str(config['load_gripper']),
                        'use_fake_hardware': str(use_fake_hardware),
                        'fake_sensor_commands': str(config['fake_sensor_commands']),
                        'use_rviz': str('false'),
                    }.items(),
                ),
            )

            nodes.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    namespace=namespace,
                    arguments=[
                        controller_name, 
                        '-c', f'/{namespace}/controller_manager' if namespace else '/controller_manager',
                        '--controller-manager-timeout', '30'
                    ],
                    parameters=[
                        PathJoinSubstitution([
                            FindPackageShare('franka_launch'), 'config', "controllers.yaml",
                        ])],
                    output='screen',
                )
            )

    if any(str(config.get('use_rviz', 'false')).lower() == 'true' for config in configs.values()):
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['--display-config', PathJoinSubstitution([
                    FindPackageShare('franka_launch'), 'rviz', 'visualize_franka.rviz'
                ])],
                output='screen',
            )
        )
        
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_launch'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        DeclareLaunchArgument(
            'spawn_franka_main',
            default_value='false',
            description='Spawn franka main',
        ),
        DeclareLaunchArgument(
            'spawn_franka_left',
            default_value='false',
            description='Spawn franka left',
        ),
        DeclareLaunchArgument(
            'spawn_franka_right',
            default_value='true',
            description='Spawn franka right',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Overwrite use_fake_hardware from config file for all robots (true/false to overwrite, config_value else)',
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_impedance_with_ik_example_controller',
            description='Name of the controller to spawn',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
