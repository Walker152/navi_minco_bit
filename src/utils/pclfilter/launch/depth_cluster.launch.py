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

    # Load YAML config and extract ros__parameters mapping (if present)
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

    # Flatten nested dict to dotted parameter names
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

    # Defaults matching the node's declared parameter names
    defaults = {
        'topics.input_cloud_topic': '/gicp_map',
        'topics.output_obstacles_topic': '/obstacle_clusters_map',
        'topics.output_ground_topic': '/ground_points_map',
        'clustering.vertical_resolution': 1.0,
        'clustering.horizontal_resolution': 0.2,
        'clustering.lidar_lines': 32,
        'clustering.min_cluster_size': 20,
        # optional fallbacks mapped into reasonable namespaces
        'filtering.normal_estimation_radius': 0.5,
        'performance.max_slope_angle': 30.0,
        'frames.input_frame': 'map',
        'frames.output_frame': 'map',
    }

    # Merge loaded params over defaults
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
