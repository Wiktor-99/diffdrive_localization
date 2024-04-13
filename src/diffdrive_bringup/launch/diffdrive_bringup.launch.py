from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
        ],
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


    return LaunchDescription(declared_arguments + [ign_bridge, gazebo, rviz2])