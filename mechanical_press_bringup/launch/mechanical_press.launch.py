from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    param_file = LaunchConfiguration("param_file")
    
    # Default config file path
    default_config_file = PathJoinSubstitution([
        FindPackageShare("mechanical_press_bringup"),
        "config",
        "default_params.yaml"
    ])

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="press1"),
            DeclareLaunchArgument("param_file", default_value=default_config_file),
            Node(
                package="mechanical_press_core",
                executable="mechanical_press_core",
                name="mechanical_press",
                namespace=namespace,
                parameters=[param_file],
                output="screen",
            ),
        ]
    )