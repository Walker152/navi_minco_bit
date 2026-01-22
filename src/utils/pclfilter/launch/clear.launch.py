from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pclfilter_dir = FindPackageShare('pclfilter')
    cube_yaml = PathJoinSubstitution([pclfilter_dir, 'config', 'cube.yaml'])
    basemap_yaml = PathJoinSubstitution([pclfilter_dir, 'config', 'basemap.yaml'])
    
    return LaunchDescription([
        Node(
            package='pclfilter',
            executable='clear_node',
            name='clear_node',
            output='screen',
            parameters=[
                {'cube_file': cube_yaml},
                {'polygons_file': basemap_yaml},
            ],
            remappings=[
                ('/odom', '/odom'),
                ('/cloud_registered', '/gicp_map'),
                ('/cloud_filter_baselink', '/cloud_filter_baselink'),
            ],
        ),
    ])
