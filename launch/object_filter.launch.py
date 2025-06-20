from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('objects_filter'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='objects_filter',
            executable='object_filter_node',
            name='object_filter_node',
            output='screen',
            parameters=[config_file]
        )
    ])

