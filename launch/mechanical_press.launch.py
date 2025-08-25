from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    param_file = LaunchConfiguration("param_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="/press1"),
            DeclareLaunchArgument("param_file", default_value=""),
            Node(
                package="mechanical_press",
                executable="mechanical_press",
                name="mechanical_press",
                namespace=namespace,
                parameters=[param_file] if param_file else [],
            ),
        ]
    )
