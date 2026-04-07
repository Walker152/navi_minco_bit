from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Try to find pclfilter using FindPackageShare, fallback to hardcoded path
    try:
        pclfilter_dir = FindPackageShare('pclfilter')
        rviz_config = PathJoinSubstitution([pclfilter_dir, 'config', 'simple_rviz2.rviz'])
        config_file = PathJoinSubstitution([pclfilter_dir, 'config', 'depth_cluster.yaml'])
    except:
        # Fallback to hardcoded path
        pclfilter_dir = '/home/rm/2025-sentry-navi/install/pclfilter/share/pclfilter'
        rviz_config = os.path.join(pclfilter_dir, 'config', 'simple_rviz2.rviz')
        config_file = os.path.join(pclfilter_dir, 'config', 'depth_cluster.yaml')
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config) if isinstance(rviz_config, str) else str(rviz_config)],
        ),
        Node(
            package='pclfilter',
            executable='depth_cluster_node',
            name='depth_cluster_node',
            output='screen',
            parameters=[{
                'input_cloud_topic': '/gicp_map',
                'output_ground_topic': '/ground_points_map',
                'output_obstacles_topic': '/obstacle_clusters_map',
                'vertical_resolution': 1.0,
                'horizontal_resolution': 0.2,
                'lidar_lines': 32,
                'min_cluster_size': 20,
                'max_slope_angle': 30.0,
                'normal_estimation_radius': 0.5,
                'input_frame': 'map',
                'output_frame': 'map',
            }],
        ),
    ])
