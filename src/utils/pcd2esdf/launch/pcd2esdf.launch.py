import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pcd2esdf'),
        'config',
        'pcd2esdf.yaml'
    )

    return LaunchDescription([
        Node(
            package='pcd2esdf',
            executable='pcd2esdf_node',
            name='fast_esdf_generator',
            output='screen',
            parameters=[config]
        )
    ])
