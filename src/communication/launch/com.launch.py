from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='communication',
            executable='sentry',
            name='sentry',
            output='screen',
            respawn=True
        )
    ])