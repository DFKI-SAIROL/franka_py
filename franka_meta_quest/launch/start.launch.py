from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='franka_meta_quest',
            executable='oculus_action_main',
            namespace='franka_right',
            output='screen'
        )
    ])
