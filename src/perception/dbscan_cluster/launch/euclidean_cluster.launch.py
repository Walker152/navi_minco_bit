from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('euclidean_cluster'),
        'config',
        'euclidean_cluster.params.yaml'
    )

    return LaunchDescription([
        Node(
            package='euclidean_cluster',
            executable='euclidean_cluster_node',
            name='euclidean_cluster_node',
            output='screen',
            parameters=[config_file]
        )
    ])
