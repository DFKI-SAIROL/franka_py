from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='franka_visualization',
            executable='pose_line_node',
            name='pose_line_node',
            output='screen',
            parameters=[{
                'topics': ['/robot1/pose', '/robot2/pose'],
                'line_length': 500,
                'min_distance': 0.001,
                'publish_topic': '/pose_lines'
            }]
        )
    ])
