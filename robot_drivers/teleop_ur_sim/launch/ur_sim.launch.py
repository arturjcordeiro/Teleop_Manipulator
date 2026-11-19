import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            get_package_share_directory("teleop_ur_sim"),
            "config",
            [LaunchConfiguration("robot"), ".yaml"],
        ]
    )
    node_launch = os.path.join(
        get_package_share_directory("teleop_manipulators"),
        "launch",
        "teleop_node.launch.py",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="ur_sim",
                description="Config file in config/",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(node_launch),
                launch_arguments={"robot_config_file": config}.items(),
            ),
        ]
    )
