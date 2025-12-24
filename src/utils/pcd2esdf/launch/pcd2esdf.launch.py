from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_path',
            default_value='src/navigation/navi2_bringup/maps/pcd/2026rmuc.pcd',
            description='Path to the input PCD file'
        ),
        DeclareLaunchArgument(
            'output_path',
            default_value='src/utils/pcd2esdf/maps/2026',
            description='Path to the output ESDF file'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Grid resolution in meters'
        ),
        DeclareLaunchArgument(
            'downsample_resolution',
            default_value='0.05',
            description='Downsample resolution in meters'
        ),
        DeclareLaunchArgument(
            'padding',
            default_value='1.0',
            description='Padding around the point cloud in meters'
        ),
        
        Node(
            package='pcd2esdf',
            executable='pcd2esdf_node',
            name='pcd2esdf_node',
            output='screen',
            parameters=[{
                'pcd_path': LaunchConfiguration('pcd_path'),
                'output_path': LaunchConfiguration('output_path'),
                'resolution': LaunchConfiguration('resolution'),
                'downsample_resolution': LaunchConfiguration('downsample_resolution'),
                'padding': LaunchConfiguration('padding'),
                'visualize': True
            }]
        )
    ])
