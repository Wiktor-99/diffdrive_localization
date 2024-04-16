from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "world",
            default_value="default.sdf",
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            name="use_sim",
            default_value="True",
            description='Set to "true" to run simulation',
        )
    ]

    ign_gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("ros_ign_gazebo"), 'launch', 'ign_gazebo.launch.py'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
         launch_arguments=[
            ('ign_args', [LaunchConfiguration('world'),' -v 4'])])

    ign_bridge = os.path.join(get_package_share_directory('diffdrive_bringup'), 'config', 'ign_bridge.yaml')
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        ros_arguments=["-p", f"config_file:={ign_bridge}"],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diffdrive_bringup"), 'rviz', 'localization.rviz'])

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_diffdrive_robot",
        arguments=["-name", "diffdrive_robot", "-topic", "robot_description"],
        output="screen",
    )

    xacro_file = os.path.join(get_package_share_directory('diffdrive_description'), 'urdf', 'robot.urdf.xacro')
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': Command(['xacro ', xacro_file]) }
        ],
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('diffdrive_bringup'), 'config', 'ekf.yaml'),
            {'use_sim_time' : True}
        ]
    )

    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        shell=False,
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        name="activate_diff_drive_controller",
        cmd=[ "ros2", "control", "load_controller", "--set-state", "active", "diff_drive_controller"],
        shell=False,
        output="screen",
    )

    return LaunchDescription(
        declared_arguments +
        [
            ign_bridge,
            gazebo,
            gazebo_spawn_robot,
            robot_state_publisher,
            robot_localization_node,
            load_joint_state_controller,
            load_joint_trajectory_controller,
            rviz2
        ])