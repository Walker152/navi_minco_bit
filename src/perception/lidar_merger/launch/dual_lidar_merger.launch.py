from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_merger')
    params_file = os.path.join(pkg_share, 'config', 'dual_lidar_merger.yaml')

    node = Node(
        package='lidar_merger',
        executable='lidar_merger_node',
        name='lidar_merger_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        node,
    ])
