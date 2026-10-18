from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the teleop_manipulators my_node executable."""
    return LaunchDescription(
        [
            Node(
                package="teleop_manipulators",
                executable="teleop_manipulators_node",
                name="teleop_manipulators_node",
                output="screen",
            ),
        ]
    )

