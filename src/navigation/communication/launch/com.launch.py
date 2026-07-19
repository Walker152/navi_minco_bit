from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='communication',
            executable='communication',
            name='communication_sentry',
            output='screen',
            respawn=True,
            parameters=[{
                'communication.enable_performance_diagnostics': True,
            }]
        )
    ])