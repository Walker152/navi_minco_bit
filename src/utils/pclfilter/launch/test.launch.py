from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pclfilter',
            executable='depth_cluster_node',
            name='pcd_processor',
            output='screen',
            parameters=[
                {'input_pcd': '/home/rm/sentinel-up-gimbal/src/pclfilter/PCD/10w.pcd'},
                {'ground_output': '/home/rm/sentinel-up-gimbal/src/pclfilter/PCD/ground.pcd'},
                {'cluster_output': '/home/rm/sentinel-up-gimbal/src/pclfilter/PCD/clusters.pcd'},
                {'angular_step': 1},
                {'max_ground_height': 0.2},
                {'min_cluster_size': 32},
                {'max_cluster_size': 20},
            ],
        ),
    ])