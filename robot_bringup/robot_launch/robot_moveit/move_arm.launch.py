from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="move_arm",
            executable="move_arm_node",
            name="move_arm_node",
            output="screen",
            parameters=[{
                "robot_base_part": "base"  # Or whatever your base frame is
            }]
        )
    ])