import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """Launch the teleop_manipulators my_node executable."""
    robot = LaunchConfiguration("robot").perform(context)
    config = os.path.join(
        get_package_share_directory("teleop_manipulators"), "config", f"{robot}.yaml"
    )
    return [
        Node(
            package="teleop_manipulators",
            executable="teleop_manipulator_component_node",
            name="teleop_manipulators_node",
            parameters=[config],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot", default_value="ur5", description="Config file in config/"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
