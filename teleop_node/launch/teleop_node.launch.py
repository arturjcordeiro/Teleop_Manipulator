from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            description='Path to the teleop node parameter YAML (provided by a driver package)'),
        Node(
            package='teleop_manipulators',
            executable='teleop_manipulator_component_node',
            parameters=[LaunchConfiguration('robot_config_file')],
            output='screen',
        ),
    ])
