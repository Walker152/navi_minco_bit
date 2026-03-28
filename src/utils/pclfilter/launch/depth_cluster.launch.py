from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    try:
        pclfilter_dir = get_package_share_directory('pclfilter')
    except Exception:
        pclfilter_dir = '/home/rm/2025-sentry-navi/install/pclfilter/share/pclfilter'

    config_path = os.path.join(pclfilter_dir, 'config', 'depth_cluster.yaml')

    params = {}
    try:
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
            if isinstance(cfg, dict) and 'ros__parameters' in cfg:
                params = cfg['ros__parameters']
            else:
                params = cfg if isinstance(cfg, dict) else {}
    except Exception:
        params = {}

    def flatten_dict(d, prefix=''):
        items = {}
        for k, v in d.items():
            new_key = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict):
                items.update(flatten_dict(v, new_key))
            else:
                items[new_key] = v
        return items

    flat_params = flatten_dict(params)

    defaults = {
        'topics.input_cloud_topic': '/gicp_map',
        'topics.output_obstacles_topic': '/obstacle_clusters_map',
        'topics.output_ground_topic': '/ground_points_map',
        'clustering.vertical_resolution': 1.0,
        'clustering.horizontal_resolution': 0.2,
        'clustering.lidar_lines': 32,
        'clustering.depth_threshold': 0.5,
        'clustering.angle_slack': 5.0,
        'clustering.ground.height_threshold': 2.5,
        'clustering.ground.max_slope_angle': 35.0,
        'clustering.normal_estimation_radius': 0.6,
        'clustering.euclidean.cluster_tolerance': 0.3,
        'clustering.euclidean.min_cluster_size': 5,
        'clustering.euclidean.max_cluster_size': 25000,
        'frames.input_frame': 'map',
        'frames.output_frame': 'map',
    }

    merged = defaults.copy()
    merged.update(flat_params)

    return LaunchDescription([
        Node(
            package='pclfilter',
            executable='depth_cluster_node',
            name='depth_cluster',
            output='screen',
            parameters=[merged],
        ),
    ])