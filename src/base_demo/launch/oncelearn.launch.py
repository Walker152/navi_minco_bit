"""
使用格式化插件，一旦无法格式化，可能是代码写错了
"""

# 导入 LaunchDescription 类，这是所有 ROS 2 launch 文件的基类
# LaunchDescription 用于描述要启动的节点、参数和其他行为
from launch import LaunchDescription

# 导入 Node 操作类，用于定义 ROS 节点启动配置
from launch_ros.actions import Node

# 文件操作，用于载入yaml文件
import os

# 载入其他文件得launch
from launch.actions import IncludeLaunchDescription  # 关键：包含其他launch文件的动作
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 导入 ROS 2 的特殊工具函数 - 用于获取功能包共享目录路径
from ament_index_python.packages import get_package_share_directory

# 导入分组操作类
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

# 感觉没啥用，因为可以使用.bash集成
from launch.actions import ExecuteProcess  # 用于执行任意系统命令
from launch.substitutions import FindExecutable  # 在系统PATH中查找可执行文件


# 这个函数名一个字也不能错，类似 CMakeLists.txt
# 这个函数类似ROS1 launch文件的 多个<node><node/>
def generate_launch_description():
    node2 = Node(package="test1", executable="suber", name="suber1", output="screen")
    node1 = Node(
        package="test1",  # 包名
        executable="puber",  # 可执行文件，CMAKE && package.xml文件中指定的
        name="puber1",  # 节点名称，控制台时间戳后面显示的
        namespace="/mylearn",  # 多机器人启动，命名空间
        # 载入yaml文件
        parameters=[
            os.path.join(  # 构建安全路径
                get_package_share_directory("test1"),  # 获取包安装目录
                "config",  # 配置子目录
                "test.yaml",  # YAML文件名
            )
        ],
        # 话题重映射
        remappings=[
            # 格式: (原话题, 新话题)
            ("/turtle1/cmd_vel", "/cmd_vel"),
            ("/haha", "xixi"),
        ],
        # 常用于rviz，gazebo等文件启动
        arguments=[
            "-d",  # RViz的参数: 加载配置文件
            os.path.join(
                get_package_share_directory("test1"),
                "config",
                "my.rviz",  # RViz配置文件
            ),
        ],
        respawn=True,  # 自动重启崩溃的节点
        output="screen",
    )
    # 参数
    robots = ["robot1", "robot2", "robot3"]
    # 分组方法使用示例：
    for robot in robots:
        GroupAction(
            [
                PushRosNamespace(robot),
                Node(package="navigation", executable="localization", name="amcl"),
                Node(package="navigation", executable="path_planner", name="nav2"),
            ]
        )
    g0 = GroupAction(
        actions=[
            PushRosNamespace("g1"),
            GroupAction(
                [
                    PushRosNamespace("subgroup"),  # 第二层命名空间
                    node2,  # → /g1/subgroup/t1
                ]
            ),
            node2,  # → /g1/t2 (不受内层影响)
        ]
    )
    turtle1 = Node(
        package="turtlesim", executable="turtlesim_node", name="t1"  # 节点名称为"t1"
    )

    turtle2 = Node(
        package="turtlesim", executable="turtlesim_node", name="t2"  # 节点名称为"t2"
    )

    turtle3 = Node(
        package="turtlesim", executable="turtlesim_node", name="t3"  # 节点名称为"t3"
    )

    # 创建第一分组：将turtle1和turtle2放入"g1"命名空间
    g1 = GroupAction(
        actions=[
            PushRosNamespace(namespace="g1"),  # 应用命名空间到后续动作
            turtle1,  # 实际节点：全名变为/g1/t1
            turtle2,  # 实际节点：全名变为/g1/t2
        ]
    )

    # 创建第二分组：将turtle3放入"g2"命名空间
    g2 = GroupAction(
        actions=[
            PushRosNamespace(namespace="g2"),  # 应用命名空间到后续动作
            turtle3,  # 实际节点：全名变为/g2/t3
        ]
    )

    # ！！！载入其他launch文件，并remap
    other_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("test1"), "launch", "heihei.py"
            )
        ),
        launch_arguments={
            "background_r": "200",
            "background_g": "100",
            "background_b": "70",
        }.items(),
    )
    return LaunchDescription([node1, node2])
    # return LaunchDescription([node1, node2,other_launch])
