from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_merger')
    params_file = os.path.join(pkg_share, 'config', 'dual_lidar_merger.yaml')

    container = ComposableNodeContainer(
        name='lidar_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_merger',
                plugin='LidarMergerNode',
                name='lidar_merger_node',
                parameters=[params_file],
            ),
        ],
    )

    return LaunchDescription([
        container,
    ])
