# robot_bringup/robot_launch/robot_moveit/rviz.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("franka_fr3_moveit_config"),
        "rviz",
        "moveit.rviz"
    ])

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
            output="screen"
        )
    ])
