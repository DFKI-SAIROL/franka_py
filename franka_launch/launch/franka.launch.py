#  Copyright (c) 2025 Franka Robotics GmbH
#  Modified for Dynamic Configuration Architecture
############################################################################

import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_robot_nodes(context):
    robot_config_name = LaunchConfiguration('robot_config').perform(context)
    if not robot_config_name.endswith('.yaml'):
        robot_config_name += '.yaml'
        
    yaml_path = os.path.join(
        get_package_share_directory('franka_robot_description'),
        'config',
        robot_config_name,
    )
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Robot config file not found: {yaml_path}")
        
    with open(yaml_path, 'r') as f:
        y_config = yaml.safe_load(f)
        
    robot_config = y_config.get('robot_config', {})
    urdf_file_name = robot_config.get('urdf_file', 'robots/fr3/fr3.urdf.xacro')
    gripper_type = robot_config.get('gripper_type', 'none')
    xyz = robot_config.get('xyz', '0 0 0')
    rpy = robot_config.get('rpy', '0 0 0')
    arm_prefix = robot_config.get('arm_prefix', '')
    arm_id = robot_config.get('arm_id', 'fr3')
    srdf_file_name = robot_config.get('srdf_file', 'fr3/fr3.srdf.xacro')

    arm_controller = robot_config.get('arm_controller', 'cartesian_impedance_controller')
    gripper_controller = robot_config.get('gripper_controller', None)

    load_franka_gripper = gripper_type == 'franka_default'
    use_fake_hardware_launch_configuration = LaunchConfiguration('use_fake_hardware').perform(context)
    use_fake_hardware = use_fake_hardware_launch_configuration.lower() == 'true'

    urdf_path = PathJoinSubstitution([
        FindPackageShare('franka_robot_description'), 'urdf', urdf_file_name
    ]).perform(context)
    
    # Base mappings shared across all URDF instantiations
    base_mappings = {
        'arm_id': arm_id,
        'arm_prefix': arm_prefix,
        'robot_ip': LaunchConfiguration('robot_ip').perform(context),
        'hand': 'true' if load_franka_gripper else 'false',
        'gripper_type': gripper_type,
        'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
        'xyz': xyz,
        'rpy': rpy,
    }

    # Unified URDF (Franka HW Active, Dynamixel HW Disabled for decoupled control)
    urdf_mappings = base_mappings.copy()
    urdf_mappings.update({'ros2_control': 'true', 'gripper_ros2_control': 'false'})
    robot_description = xacro.process_file(urdf_path, mappings=urdf_mappings).toprettyxml(indent='  ')

    namespace = LaunchConfiguration('namespace').perform(context)
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('franka_launch'), 'config', "controllers.yaml"
    ]).perform(context)

    joint_state_publisher_sources = ['franka/joint_states', 'franka_gripper/joint_states']

    cm_params = {
        'robot_description': robot_description,
        'load_gripper': load_franka_gripper,
        'use_fake_hardware': use_fake_hardware,
        'arm_id': arm_id,
        'arm_prefix': arm_prefix,
    }

    import tempfile
    broadcaster_arm_id = f'{arm_prefix}_{arm_id}' if arm_prefix else arm_id
    joints_list = [f'{arm_prefix}_{arm_id}_joint{i}' if arm_prefix else f'{arm_id}_joint{i}' for i in range(1, 8)]

    controller_params = {
        f'/**/{arm_controller}': {
            'ros__parameters': {
                'joints': joints_list
            }
        },
    }

    end_effector_frame = LaunchConfiguration('end_effector_frame').perform(context)
    if end_effector_frame:
        controller_params[f'/**/{arm_controller}']['ros__parameters']['end_effector_frame'] = end_effector_frame

    # Write to a temporary file because passing nested dicts directly to Node `parameters` 
    # doesn't work well for controller_manager uninitialized parameter checks
    param_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(controller_params, param_file)
    param_file.close()

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        # Arm Controller Manager (1000Hz)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                controllers_yaml,
                cm_params,
                param_file.name
            ],
            remappings=[
                ('~/robot_description', 'robot_description'),
                ('joint_states', joint_state_publisher_sources[0])
            ],
            output='screen',
            on_exit=Shutdown(),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{
                'robot_description': robot_description,
                'source_list': joint_state_publisher_sources,
                'rate': 1000,
            }],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=[
                'franka_robot_state_broadcaster',
                '-c', f'/{namespace}/controller_manager' if namespace else '/controller_manager',
                '-p', f'arm_id:={broadcaster_arm_id}'
            ],
            # parameters=[{'arm_id': arm_id}],
            condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')),
            output='screen',
        ),
    ]

    arm_spawner_args = [
        arm_controller,
        '-c', f'/{namespace}/controller_manager' if namespace else '/controller_manager',
        '--controller-manager-timeout', '30'
    ]

    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=arm_spawner_args,
        output='screen',
    ))

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_launch'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={
            'gripper_type': gripper_type,
            'urdf_file': urdf_file_name,
            'arm_id': arm_id,
            'arm_prefix': arm_prefix,
            'namespace': namespace,
            'robot_ip': LaunchConfiguration('robot_ip').perform(context),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
            'xyz': xyz,
            'rpy': rpy,
        }.items(),
        condition=UnlessCondition('true' if gripper_type == 'none' else 'false'),
    ))

    return nodes


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('robot_config',
                              default_value='fr3',
                              description='Name of the robot config yaml in franka_robot_description'),
        DeclareLaunchArgument('namespace',
                              default_value='',
                              description='Namespace for the robot'),
        DeclareLaunchArgument('robot_ip',
                              default_value='172.16.0.3',
                              description='Hostname or IP address of the robot'),
        DeclareLaunchArgument('use_fake_hardware',
                              default_value='false',
                              description='Use fake hardware'),
        DeclareLaunchArgument('end_effector_frame',
                              default_value='',
                              description='End effector frame for cartesian impedance controller'),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_robot_nodes)])
