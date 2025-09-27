from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='com',
            executable='com',
            name='com_sentry',
            output='screen',
            respawn=True
        )
    ])