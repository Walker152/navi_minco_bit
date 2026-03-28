from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pclfilter_share = FindPackageShare('pclfilter')
    small_point_lio_share = FindPackageShare('small_point_lio')

    # small_point_lio 配置（使用包内 mid360.yaml）
    small_cfg = PathJoinSubstitution([small_point_lio_share, 'config', 'mid360.yaml'])

    # pclfilter 的配置（cube/basemap）
    cube_yaml = PathJoinSubstitution([pclfilter_share, 'config', 'cube.yaml'])
    basemap_yaml = PathJoinSubstitution([pclfilter_share, 'config', 'basemap.yaml'])

    small_node = Node(
        package='small_point_lio',
        executable='small_point_lio_node',
        name='small_point_lio',
        output='screen',
        parameters=[
            small_cfg,
            # make small_point_lio publish directly to /gicp_map so clear_node can consume it
            {'publish_topic': '/gicp_map'},
        ],
    )

    clear_node = Node(
        package='pclfilter',
        executable='clear_node',
        name='clear_node',
        output='screen',
        parameters=[
            {'cube_file': cube_yaml},
            {'polygons_file': basemap_yaml},
        ],
        # 保持与现有 clear.launch.py 相同的 remappings，确保接收来自 /cloud_registered 的数据
        remappings=[
            ('/odom', '/odom'),
            ('/cloud_registered', '/gicp_map'),
            ('/cloud_filter_baselink', '/cloud_filter_baselink'),
        ],
    )

    return LaunchDescription([
        small_node,
        clear_node,
    ])
