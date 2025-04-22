from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mechanical_press',
            executable='press_node',
            name='press_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=['default_params.yaml'],
        )
    ])
