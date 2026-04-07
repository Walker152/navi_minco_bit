from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('dbscan_cluster'),
        'config',
        'dbscan_cluster.params.yaml'
    )

    return LaunchDescription([
        Node(
            package='dbscan_cluster',
            executable='dbscan_cluster_node',
            name='dbscan_cluster_node',
            output='screen',
            parameters=[config_file]
        )
    ])
