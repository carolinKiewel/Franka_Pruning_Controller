from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Paths
    moveit_config = get_package_share_directory("franka_fr3_moveit_config")
    move_arm_config = get_package_share_directory("move_arm")

    urdf_path = PathJoinSubstitution([
        FindPackageShare("franka_fr3_moveit_config"),
        "urdf",
        "fr3.urdf.xacro"
    ])
    srdf_path = os.path.join(moveit_config, "config", "fr3.srdf")
    servo_config = os.path.join(move_arm_config, "config", "franka_servo.yaml")

    # Launch args
    use_simulation_arg = DeclareLaunchArgument(
        "use_simulation",
        default_value="false",
        description="Use simulated robot"
    )
    use_simulation = LaunchConfiguration("use_simulation")

    # Robot description
    robot_description = {"robot_description": Command(["xacro", urdf_path])}
    robot_description_semantic = {
        "robot_description_semantic": open(srdf_path, "r", encoding="utf-8").read()
    }

    # Combined servo parameters
    servo_params = [
        servo_config,
        {"robot_description": Command(["xacro", urdf_path])},
        {"robot_description_semantic": open(srdf_path, "r").read()}
    ]


    return LaunchDescription([
        use_simulation_arg,

        # Launch MoveIt stack (rviz, move_group, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config, "launch", "moveit.launch.py")
            ),
            launch_arguments={
                "robot_ip": "127.0.0.1",  # Simulation IP
                "use_simulation": use_simulation
            }.items()
        ),

        # Launch servo node with slight delay
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="moveit_servo",
                    executable="servo_node_main",
                    name="servo_node",
                    parameters=servo_params,
                    output="screen"
                )
            ]
        )
    ])