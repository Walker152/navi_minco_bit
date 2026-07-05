from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("point_lio"),
                            "launch",
                            "livox_pointlio_intra_process.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "driver_params_file": PathJoinSubstitution(
                        [
                            FindPackageShare("livox_ros_driver2"),
                            "config",
                            "mixed_MID360_component.yaml",
                        ]
                    ),
                    "pointlio_params_file": PathJoinSubstitution(
                        [FindPackageShare("point_lio"), "config", "mid360.yaml"]
                    ),
                    "pointlio_imu_topic": "livox/imu_192_168_1_135",
                    "use_intra_process": "true",
                    "container_name": "livox_pointlio_container",
                    "log_level": "info",
                }.items(),
            )
        ]
    )
