import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_gripper_nodes(context):
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    
    # If no gripper is requested, return empty list
    if gripper_type == 'none':
        return []

    # Get arguments
    arm_id = LaunchConfiguration('arm_id').perform(context)
    arm_prefix = LaunchConfiguration('arm_prefix').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    use_fake_hardware_str = LaunchConfiguration('use_fake_hardware').perform(context)
    use_fake_hardware = use_fake_hardware_str.lower() == 'true'
    namespace = LaunchConfiguration('namespace').perform(context)
    
    # We load the main URDF but disable arm control and enable gripper control
    # For a standalone gripper, we might only want to process the gripper URDF,
    # but the custom gripper is currently a macro inside the fr3.urdf.xacro or similar.
    # To support standalone, we could have a specific gripper-only URDF, but for now 
    # we'll build the full URDF with arm HW disabled.
    urdf_file_name = LaunchConfiguration('urdf_file').perform(context)
    
    urdf_path = PathJoinSubstitution([
        FindPackageShare('franka_robot_description'), 'urdf', urdf_file_name
    ]).perform(context)
    
    # We configure URDF to only run ros2_control for the gripper
    gripper_mappings = {
        'arm_id': arm_id,
        'arm_prefix': arm_prefix,
        'robot_ip': robot_ip,
        'hand': 'false' if gripper_type != 'franka_default' else 'true',
        'gripper_type': gripper_type,
        'use_fake_hardware': use_fake_hardware_str,
        'ros2_control': 'false', 
        'gripper_ros2_control': 'true'
    }
    
    robot_description_gripper = xacro.process_file(urdf_path, mappings=gripper_mappings).toprettyxml(indent='  ')

    nodes = []

    # State publisher for the gripper
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='gripper_state_publisher',
        parameters=[{'robot_description': robot_description_gripper}],
        remappings=[
            ('robot_description', 'gripper_robot_description'),
        ],
        output='screen',
    ))

    # Franka specific loading
    if gripper_type == 'franka_default':
        # Include the standard franka_gripper launch file for default behavior
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={
                'namespace': namespace,
                'robot_ip': robot_ip,
                'use_fake_hardware': use_fake_hardware_str,
            }.items(),
        ))
        
    # Dynamixel specific loading
    elif gripper_type == 'rh_p12_rn_a' or gripper_type == 'dynamixel':
        controllers_yaml = PathJoinSubstitution([
            FindPackageShare('franka_launch'), 'config', "dynamixel_controllers.yaml"
        ]).perform(context)

        abs_manager = f'/{namespace}/dynamixel_controller_manager' if namespace else '/dynamixel_controller_manager'
        abs_joint_states = f'/{namespace}/franka_gripper/joint_states' if namespace else '/franka_gripper/joint_states'

        print(abs_manager)
        print(abs_joint_states)

        # Spawning generic parameters for Dynamixel
        nodes.append(Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='dynamixel_controller_manager',
            namespace=namespace,
            parameters=[
                controllers_yaml,
                {
                    'robot_description': robot_description_gripper,
                    'use_fake_hardware': use_fake_hardware,
                    'update_rate': 50,
                }
            ],
            remappings=[
                ('joint_states', abs_joint_states),
                ('robot_description', 'gripper_robot_description'),
                ('~/robot_description', 'gripper_robot_description')
            ],
            output='screen',
            on_exit=Shutdown()
        ))
        
        nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=[
                'gripper_joint_state_broadcaster',
                '-c', abs_manager,
                '--controller-manager-timeout', '30'
            ],
            output='screen',
        ))
        
        nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=[
                'gripper_controller', 
                '-c', abs_manager,
                '--controller-manager-timeout', '30'
            ],
            output='screen',
        ))

    return nodes

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('gripper_type', default_value='rh_p12_rn_a', description='Type of gripper to load'),
        DeclareLaunchArgument('urdf_file', default_value='robots/fr3/fr3.urdf.xacro', description='URDF file to load'),
        DeclareLaunchArgument('arm_id', default_value='fr3', description='ID of the arm'),
        DeclareLaunchArgument('arm_prefix', default_value='', description='Prefix of the arm'),
        DeclareLaunchArgument('namespace', default_value='franka_right', description='Namespace to run in'),
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.3', description='IP of the actual robot or gripper'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Whether to use mock hardware'),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_gripper_nodes)])
