import os
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve package directories and default files.
    nav2_pkg_dir = get_package_share_directory('navi2')
    slam_params = os.path.join(nav2_pkg_dir, 'params', 'slam.yaml')
    workspace_dir = os.path.abspath(os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '../../../..'))
    save_map_script = os.path.join(workspace_dir, 'scripts', 'savemap.bash')

    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    map_topic = LaunchConfiguration('map_topic')
    map_updates_topic = LaunchConfiguration('map_updates_topic')
    auto_save = LaunchConfiguration('auto_save')
    auto_save_interval = LaunchConfiguration('auto_save_interval')

    # Convert PointCloud2 to LaserScan for mapping and costmaps.
    pcl_to_scan_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'slambase',
            'transform_tolerance': 0.01,
            'queue_size': 1,
            'min_height': -0.1,
            'max_height': 1.2,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.4,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('cloud_in', cloud_topic),
            ('scan', scan_topic)
        ]
    )

    # Start slam_toolbox in online async mode.
    slam_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': scan_topic,
            }],
        remappings=[
            ('/map', map_topic),
            ('/map_updates', map_updates_topic)
        ]

    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav2_pkg_dir, 'config', 'slam.rviz')],
        remappings=[
            ('/map', map_topic),
            ('/map_updates', map_updates_topic)
        ]
    )   

    # Periodically save timestamped YAML/PGM map checkpoints.
    auto_save_cmd = ExecuteProcess(
        cmd=['bash', save_map_script, map_topic],
        output='screen',
        additional_env={'AUTO_SAVE_INTERVAL': auto_save_interval},
        condition=IfCondition(auto_save)
    )

    # Assemble launch actions.
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument(
        'cloud_topic',
        default_value='/cloud_registered_full',
        description='Input PointCloud2 topic'))
    ld.add_action(DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Output LaserScan topic'))
    ld.add_action(DeclareLaunchArgument(
        'map_topic',
        default_value='/map',
        description='SLAM occupancy grid topic published by slam_toolbox'))
    ld.add_action(DeclareLaunchArgument(
        'map_updates_topic',
        default_value='/map_updates',
        description='SLAM incremental map update topic published by slam_toolbox'))
    ld.add_action(DeclareLaunchArgument(
        'auto_save',
        default_value='true',
        description='Automatically save timestamped YAML/PGM map checkpoints'))
    ld.add_action(DeclareLaunchArgument(
        'auto_save_interval',
        default_value='60.0',
        description='Seconds between automatic map saves'))

    ld.add_action(pcl_to_scan_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(auto_save_cmd)
    return ld
