from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='franka_visualization',
            executable='pose_line_node',
            name='pose_line_node',
            namespace='franka_right',
            output='screen',
            parameters=[{
                'line_length': 500,
                'min_distance': 0.001,
            }]
        ),
        Node(
            package='franka_visualization',
            executable='pose_line_node',
            name='pose_line_node',
            namespace='franka_left',
            output='screen',
            parameters=[{
                'line_length': 500,
                'min_distance': 0.001,
            }]
        )
    ])
