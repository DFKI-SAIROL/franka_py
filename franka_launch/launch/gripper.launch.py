import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
    xyz = LaunchConfiguration('xyz').perform(context)
    rpy = LaunchConfiguration('rpy').perform(context)
    xyz_ee = LaunchConfiguration('xyz_ee').perform(context)
    rpy_ee = LaunchConfiguration('rpy_ee').perform(context)

    urdf_file_name = LaunchConfiguration('urdf_file').perform(context)

    urdf_path = PathJoinSubstitution(
        [FindPackageShare('franka_robot_description'), 'urdf', urdf_file_name]
    ).perform(context)

    # We configure URDF to only run ros2_control for the gripper
    gripper_mappings = {
        'arm_id': arm_id,
        'arm_prefix': arm_prefix,
        'robot_ip': robot_ip,
        'hand': 'false' if gripper_type != 'franka_default' else 'true',
        'gripper_type': gripper_type,
        'use_fake_hardware': use_fake_hardware_str,
        'ros2_control': 'false',
        'gripper_ros2_control': 'true',
        'xyz': xyz,
        'rpy': rpy,
        'xyz_ee': xyz_ee,
        'rpy_ee': rpy_ee,
    }

    robot_description_gripper = xacro.process_file(
        urdf_path, mappings=gripper_mappings
    ).toprettyxml(indent='  ')

    nodes = []

    # State publisher for the gripper
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='gripper_state_publisher',
            parameters=[{'robot_description': robot_description_gripper}],
            remappings=[
                ('robot_description', 'gripper_robot_description'),
            ],
            output='screen',
        )
    )

    # Franka specific loading
    if gripper_type == 'franka_default':
        if not use_fake_hardware:
            # Pass the correct robot_type so that franka_gripper constructs the correct joint names
            robot_type = f'{arm_prefix}_{arm_id}' if arm_prefix else arm_id

            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            PathJoinSubstitution(
                                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py']
                            )
                        ]
                    ),
                    launch_arguments={
                        'namespace': namespace,
                        'robot_ip': robot_ip,
                        'use_fake_hardware': use_fake_hardware_str,
                        'robot_type': robot_type,
                    }.items(),
                )
            )
        else:
            # In fake mode, the controller parameters are already loaded by the arm's ros2_control_node
            # (which was started in franka.launch.py). We just need to spawn the controller.
            nodes.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    namespace=namespace,
                    arguments=[
                        'franka_gripper',
                        '-c',
                        f'/{namespace}/controller_manager' if namespace else '/controller_manager',
                    ],
                    output='screen',
                )
            )

    # Dynamixel specific loading
    elif gripper_type == 'rh_p12_rn_a' or gripper_type == 'dynamixel':
        controllers_yaml = PathJoinSubstitution(
            [FindPackageShare('franka_launch'), 'config', 'dynamixel_controllers.yaml']
        ).perform(context)

        gripper_ns = f'{namespace}/gripper' if namespace else '/gripper'
        abs_joint_states = (
            f'/{namespace}/franka_gripper/joint_states'
            if namespace
            else '/franka_gripper/joint_states'
        )

        # Spawning generic parameters for Dynamixel
        nodes.append(
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                namespace=gripper_ns,
                parameters=[
                    controllers_yaml,
                    {
                        'robot_description': robot_description_gripper,
                        'use_fake_hardware': use_fake_hardware,
                        'use_local_parameters': True,
                    },
                ],
                remappings=[
                    ('joint_states', abs_joint_states),
                    ('robot_description', 'gripper_robot_description'),
                    ('~/robot_description', 'gripper_robot_description'),
                ],
                output='screen',
                on_exit=Shutdown(),
            )
        )

        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=gripper_ns,
                arguments=[
                    'gripper_joint_state_broadcaster',
                    '-c',
                    'controller_manager',
                    '--controller-manager-timeout',
                    '30',
                ],
                output='screen',
            )
        )

        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=gripper_ns,
                arguments=[
                    'gripper_controller',
                    '-c',
                    'controller_manager',
                    '--controller-manager-timeout',
                    '30',
                ],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'gripper_type', default_value='rh_p12_rn_a', description='Type of gripper to load'
        ),
        DeclareLaunchArgument(
            'urdf_file', default_value='robots/fr3/fr3.urdf.xacro', description='URDF file to load'
        ),
        DeclareLaunchArgument('arm_id', default_value='fr3', description='ID of the arm'),
        DeclareLaunchArgument('arm_prefix', default_value='', description='Prefix of the arm'),
        DeclareLaunchArgument(
            'namespace', default_value='franka_right', description='Namespace to run in'
        ),
        DeclareLaunchArgument(
            'robot_ip', default_value='172.16.0.3', description='IP of the actual robot or gripper'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false', description='Whether to use mock hardware'
        ),
        DeclareLaunchArgument('xyz', default_value='0 0 0', description='Link 0 offset XYZ'),
        DeclareLaunchArgument('rpy', default_value='0 0 0', description='Link 0 offset RPY'),
        DeclareLaunchArgument(
            'xyz_ee', default_value='0 0 0', description='End effector offset XYZ'
        ),
        DeclareLaunchArgument(
            'rpy_ee', default_value='0 0 0', description='End effector offset RPY'
        ),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_gripper_nodes)])
