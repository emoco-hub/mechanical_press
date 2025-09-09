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
    instance_name = LaunchConfiguration("instance_name")
    
    # Default parameter file location
    default_param_file = PathJoinSubstitution([
        FindPackageShare("mechanical_press"),
        "config", 
        "default.yaml"
    ])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", 
                default_value="/dev",
                description="ROS namespace for this press instance"
            ),
            DeclareLaunchArgument(
                "param_file", 
                default_value=default_param_file,
                description="YAML parameter file for this instance"
            ),
            DeclareLaunchArgument(
                "instance_name",
                default_value="dev_press", 
                description="Unique name for this press instance"
            ),
            Node(
                package="mechanical_press",
                executable="mechanical_press",
                name="mechanical_press",
                namespace=namespace,
                parameters=[param_file],
                # Add instance name as a parameter override
                ros_arguments=[
                    '--params-file', param_file,
                    '--param', f'instance_name:={instance_name}'
                ]
            ),
        ]
    )
