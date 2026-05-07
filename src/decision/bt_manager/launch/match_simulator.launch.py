from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="bt_manager",
                executable="match_simulator",
                name="match_simulator",
                output="screen",
            ),
        ]
    )
