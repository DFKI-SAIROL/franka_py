import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Annahme: Ihr MoveIt! Konfigurationspaket heißt 'franka_fr3_moveit_config'
    moveit_config = MoveItConfigsBuilder(
        robot_name="fr3", package_name="franka_fr3_moveit_config"
    ).to_moveit_configs()

    # MoveGroup Node (Der Planungsknoten von MoveIt!)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # WICHTIG: Hier laden wir keine Controller-Manager-Konfigurationen
            # die den standardmäßigen Executer-Action-Server aktivieren würden.
            # Nur die für die Planung notwendigen Daten werden geladen.
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz (Visualisierung der Planungsszene)
    rviz_config_file = os.path.join(
        get_package_share_directory("franka_fr3_moveit_config"),
        "config",
        "moveit_rviz.rviz", # Oder Ihre eigene RViz Konfig
    )
    rviz_node = Node(
        package="rviz_common",
        executable="rviz",
        name="rviz",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            # Stellen Sie sicher, dass die Planning Scene Visualisierung aktiv ist
        ],
    )

    return LaunchDescription([
        # ACHTUNG: Die franka_ros2 Hardware-Treiber und der
        # joint_trajectory_controller (von ros2_control) müssen separat gestartet sein.
        move_group_node,
        rviz_node,
    ])