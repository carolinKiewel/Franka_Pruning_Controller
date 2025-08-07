from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config = FindPackageShare("franka_fr3_moveit_config")
    move_arm_config = FindPackageShare("move_arm")

    urdf_path = PathJoinSubstitution([moveit_config, "urdf", "fr3.urdf.xacro"])
    srdf_path = PathJoinSubstitution([moveit_config, "config", "fr3.srdf"])
    servo_config_path = PathJoinSubstitution([move_arm_config, "config", "franka_servo.yaml"])

    # Generate robot_description and robot_description_semantic
    robot_description = {
        "robot_description": Command(["xacro", urdf_path])
    }

    # Read SRDF as plain text, not as xacro
    robot_description_semantic = {
        "robot_description_semantic": PathJoinSubstitution([moveit_config, "config", "fr3.srdf"])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_simulation",
            default_value="true",
            description="Use simulated robot"
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="moveit_servo",
                    executable="servo_node_main",
                    name="servo_node",
                    output="screen",
                    parameters=[
                        servo_config_path,
                        robot_description,
                        {
                            "robot_description_semantic": Command([
                                "cat", srdf_path
                            ])
                        }
                    ]
                )
            ]
        )
    ])
