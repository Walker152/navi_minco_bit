from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pclfilter_dir = FindPackageShare('pclfilter')
    rviz_config = PathJoinSubstitution([pclfilter_dir, 'config', 'simple_rviz2.rviz'])
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
        Node(
            package='pclfilter',
            executable='depth_cluster_node',
            name='depth_cluster_node',
            output='screen',
        ),
    ])
