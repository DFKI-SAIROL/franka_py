import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    yaml_file = LaunchConfiguration('yaml_file').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)
    
    # Check if yaml_file ends with `.yaml`, else add the suffix
    if not yaml_file.endswith(".yaml"):
        yaml_file += ".yaml"
    yaml_file = PathJoinSubstitution([
        FindPackageShare('franka_robot_description'), 'config', yaml_file
    ]).perform(context)
    if not os.path.exists(yaml_file):
        raise FileNotFoundError(f"File not found: {yaml_file}")
    
    # Load yaml configuration
    with open(yaml_file, 'r') as f:
        config = yaml.safe_load(f)
        
    robot_config = config.get('robot_config', {})
    urdf_file_name = robot_config.get('urdf_file', 'robots/fr3/fr3.urdf.xacro')
    gripper_type = robot_config.get('gripper_type', 'none')
    xyz = robot_config.get('xyz', '0 0 0')
    rpy = robot_config.get('rpy', '0 0 0')
    arm_prefix = robot_config.get('arm_prefix', '')
    
    # Path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('franka_robot_description'),
        'urdf',
        urdf_file_name
    )

    # Node to publish the robot state (TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(Command([
            'xacro ', xacro_file, 
            ' gripper_type:=', gripper_type, 
            ' xyz:="', xyz, '"',
            ' rpy:="', rpy, '"',
            ' arm_prefix:=', arm_prefix,
            ' use_sim:=', use_sim
        ]), value_type=str)}]
    )

    # Node to provide a GUI to manually move the joints
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node to launch RViz2
    rviz_config_file = os.path.join(get_package_share_directory('franka_robot_description'), 'rviz', 'rh_p12_rn_a', 'rh_p12_rn_a.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    return [robot_state_publisher_node, joint_state_publisher_gui_node, rviz_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'yaml_file',
            default_value="dkfi_fr3_right",
            # default_value=PathJoinSubstitution([
            #     FindPackageShare('franka_robot_description'), 'config', 'dfki_fr3_right.yaml'
            # ]),
            description='Absolute path to the YAML configuration file for the robot'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value="true",
            description='Whether to use the simulated env or the real hardware',
        ),
        OpaqueFunction(function=launch_setup)
    ])
