import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    yaml_file = LaunchConfiguration('yaml_file').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    controller_name = LaunchConfiguration('controller_name').perform(context)
    
    # Check if yaml_file ends with `.yaml`, else add the suffix
    if not yaml_file.endswith(".yaml"):
        yaml_file += ".yaml"
    
    # If the provided path isn't absolute, assume it's in our config folder
    if not os.path.exists(yaml_file):
        yaml_file = PathJoinSubstitution([
            FindPackageShare('franka_robot_description'), 'config', yaml_file
        ]).perform(context)

    if not os.path.exists(yaml_file):
        raise FileNotFoundError(f"Config file not found: {yaml_file}")
    
    # Load yaml configuration
    with open(yaml_file, 'r') as f:
        config = yaml.safe_load(f)
        
    robot_config = config.get('robot_config', {})
    urdf_file_name = robot_config.get('urdf_file', 'robots/fr3/fr3.urdf.xacro')
    gripper_type = robot_config.get('gripper_type', 'none')
    xyz = robot_config.get('xyz', '0 0 0')
    rpy = robot_config.get('rpy', '0 0 0')
    arm_prefix = robot_config.get('arm_prefix', '')
    arm_id = robot_config.get('arm_id', 'fr3')
    robot_ip = robot_config.get('robot_ip', '')
    namespace = robot_config.get('namespace', '')
    
    # Is the gripper the default franka hand?
    load_gripper = 'true' if gripper_type == 'franka_default' else 'false'

    urdf_path = PathJoinSubstitution([
        FindPackageShare('franka_robot_description'),
        'urdf',
        urdf_file_name
    ]).perform(context)

    # Generate the robot description by calling xacro
    robot_description = ParameterValue(Command([
        'xacro ', urdf_path,
        ' gripper_type:=', gripper_type,
        ' xyz:="', xyz, '"',
        ' rpy:="', rpy, '"',
        ' arm_prefix:=', arm_prefix,
        ' arm_id:=', arm_id,
        ' use_sim:=', use_sim,
        ' use_fake_hardware:=', use_fake_hardware,
        ' robot_ip:=', robot_ip,
        ' ros2_control:=true'
    ]), value_type=str)

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('franka_launch'), 'config', "controllers.yaml"
    ]).perform(context)

    joint_state_publisher_sources = ['franka/joint_states', 'franka_gripper/joint_states']
    
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                controllers_yaml,
                {
                    'load_gripper': load_gripper == 'true',
                    'use_fake_hardware': use_fake_hardware == 'true',
                    'arm_id': arm_id,
                    'arm_prefix': arm_prefix
                }
            ],
            remappings=[('joint_states', joint_state_publisher_sources[0])],
            output='screen',
            on_exit=Shutdown(),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{
                'source_list': joint_state_publisher_sources,
                'rate': 100,
                'use_robot_description': False,
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
            arguments=['franka_robot_state_broadcaster'],
            parameters=[{'arm_id': arm_id}],
            condition=UnlessCondition(use_fake_hardware),
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=[controller_name, '--controller-manager-timeout', '30'],
            output='screen',
        )
    ]

    # Include franka_gripper launch if needed
    if load_gripper == 'true':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
                launch_arguments={
                    'namespace': namespace,
                    'robot_ip': robot_ip,
                    'use_fake_hardware': use_fake_hardware,
                }.items()
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'yaml_file',
            default_value='dfki_fr3_right',
            description='Name of the YAML config file in franka_robot_description/config (e.g., dfki_fr3_right)'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value="false",
            description='Whether to use the simulated env or the real hardware'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value="false",
            description='Whether to use fake hardware'
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='franka_joint_trajectory_controller',
            description='Name of the controller to spawn'
        ),
        OpaqueFunction(function=launch_setup)
    ])
