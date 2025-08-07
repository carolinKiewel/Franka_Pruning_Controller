from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments to pass them down
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Robot IP address'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("franka_fr3_moveit_config"),
                    "launch",
                    "moveit.launch.py"
                ])
            ),
            launch_arguments={
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
                'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands'),
            }.items()
        )
    ])
