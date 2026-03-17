import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'zed_rig_aggregator_node'
    
    # Automatically find the installed config file
    # This works because of the install(DIRECTORY config ...) in CMakeLists.txt
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'zed_rig_aggregator_node_config.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='zed_rig_aggregator_node',
            name='zed_rig_aggregator',
            output='screen',
            # Load params directly from the YAML file
            parameters=[config_path]
        )
    ])
