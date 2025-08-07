import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue

import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)

import yaml

# Generates the "default" nodes (controller_manager, robot_state_publisher, etc.)
# for the Franka robot. This function is called by the main launch file.
# It uses the xacro library to process the URDF file and generate the robot description.


def generate_robot_nodes(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands').perform(context)
    arm_id = LaunchConfiguration('arm_id').perform(context)

    #franka_xacro_file = os.path.join(
    #    get_package_share_directory('franka_description'),
    #    'robots', 'fr3', 'fr3.urdf.xacro'
    #)
    
    load_gripper_launch_configuration = LaunchConfiguration('load_gripper').perform(context)
    load_gripper = load_gripper_launch_configuration.lower() == 'true'
    urdf_path = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'fr3',
        'fr3.urdf.xacro',  # or whatever your actual file is
    )

    robot_description = {'robot_description': ParameterValue(
        xacro.process_file(
            urdf_path,
            mappings={
                'ros2_control': 'true',
                'arm_id': 'fr3',#LaunchConfiguration('arm_id').perform(context),
                'arm_prefix': LaunchConfiguration('arm_prefix').perform(context),
                'robot_ip': robot_ip,
                'hand': 'true',
                'use_fake_hardware': use_fake_hardware,
                'fake_sensor_commands': fake_sensor_commands,
            }
        ).toprettyxml(indent='  '),
        value_type=str
    )}


    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'srdf',
        'fr3_arm.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=true', ' arm_id:=',LaunchConfiguration('arm_id').perform(context)]
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}
    
    # robot_description_content = Command([
    #     'xacro ',
    #     urdf_path,
    #     ' ',
    #     f'arm_id:={arm_id} ',
    #     f'robot_ip:={robot_ip} ',
    #     f'hand:={load_gripper}'
    # ])

    # robot_description = {
    #     'robot_description': ParameterValue(robot_description_content, value_type=str)
    # }

    
    def load_yaml(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)

        try:
            with open(absolute_file_path, 'r') as file:
                return yaml.safe_load(file)
        except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
            return None

    kinematics_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml'
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("moveit_servo", "config/panda_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        #namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        output="screen",
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'franka_fr3_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config',
        'fr3_ros_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        #namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in ['fr3_arm_controller', 'joint_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager',
                    PathJoinSubstitution([namespace, 'controller_manager'])
                ],
                output='screen'
            )
        )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        #namespace=namespace,
        parameters=[
            {'source_list': ['franka/joint_states', 'fr3_gripper/joint_states'], 'rate': 30}],
    )

    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        #namespace=namespace,
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    namespace_parameter_name = 'namespace'

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        description='Hostname or IP address of the robot.')

    namespace_arg = DeclareLaunchArgument(
        namespace_parameter_name,
        default_value='',
        description='Namespace for the robot.'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'robot_ip': robot_ip,
                          use_fake_hardware_parameter_name: use_fake_hardware,
                          'namespace': namespace}.items(),
    )
    

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('franka_bringup'), 'config', "controllers.yaml"
    ]).perform(context)

    joint_state_publisher_sources = ['franka/joint_states', 'franka_gripper/joint_states']
    joint_state_rate = int(LaunchConfiguration('joint_state_rate').perform(context))

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("franka_fr3_moveit_config"),
        "rviz",
        "moveit.rviz"
    ])
    
    # node_rviz = Node(
    #         package="rviz2",
    #         executable="rviz2",
    #         name="rviz2",
    #         arguments=["-d", rviz_config_path],
    #         output="screen"
    #     )
    move_arm_node = Node(
        package='move_arm',
        executable='move_arm_node',  # Change if the executable name is different
        name='move_arm_node',
        output='screen',
        parameters=[  # If you need to pass parameters
            # {'param_name': 'value'}
        ],
    )

    plant_node = Node(
        package='move_arm',
        executable='plant_loader_node',
        name='plant_loader_node',
        output='screen',
        parameters=[],
    )
    
    return [
        robot_arg,
        namespace_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        rviz_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_publisher,
        run_move_group_node,
        franka_robot_state_broadcaster,
        gripper_launch_file,
        servo_node,
        *load_controllers,
        move_arm_node,
        plant_node,
    ]


# The generate_launch_description function is the entry point (like "main")
# We use it to declare the launch arguments and call the generate_robot_nodes function.


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('arm_id',
                              default_value='fr3',
                              description='ID of the type of arm used'),
        DeclareLaunchArgument('arm_prefix',
                              default_value='',
                              description='Prefix for arm topics'),
        DeclareLaunchArgument('namespace',
                              default_value='',
                              description='Namespace for the robot'),
        DeclareLaunchArgument('urdf_file',
                              default_value='fr3/fr3.urdf.xacro',
                              description='Path to URDF file'),
        DeclareLaunchArgument('robot_ip',
                              default_value='172.16.0.3',
                              description='Hostname or IP address of the robot'),
        DeclareLaunchArgument('load_gripper',
                              default_value='false',
                              description='Use Franka Gripper as an end-effector'),
        DeclareLaunchArgument('use_fake_hardware',
                              default_value='false',
                              description='Use fake hardware'),
        DeclareLaunchArgument('fake_sensor_commands',
                              default_value='false',
                              description='Fake sensor commands'),
        DeclareLaunchArgument('joint_state_rate',
                              default_value='30',
                              description='Rate for joint state publishing (Hz)'),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_robot_nodes)])




# # robot_bringup/robot_launch/robot_moveit/rviz.launch.py
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
    # rviz_config_path = PathJoinSubstitution([
    #     FindPackageShare("franka_fr3_moveit_config"),
    #     "rviz",
    #     "moveit.rviz"
    # ])

    # return LaunchDescription([
    #     Node(
    #         package="rviz2",
    #         executable="rviz2",
    #         name="rviz2",
    #         arguments=["-d", rviz_config_path],
    #         output="screen"
    #     )
    # ])