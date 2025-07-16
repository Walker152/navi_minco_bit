# 版权和许可证信息，表明代码遵循Apache 2.0协议
# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

import os

# 获取ROS2包共享文件夹的工具
from ament_index_python.packages import get_package_share_directory

# ROS2 launch框架导入
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 获取nav2_bringup包的共享文件夹路径
    bringup_dir = get_package_share_directory('nav2_bringup')

    # 定义launch文件中可配置的参数（通过命令行或ROS参数传入）
    namespace = LaunchConfiguration('namespace')  # ROS命名空间，默认空字符串
    use_sim_time = LaunchConfiguration('use_sim_time')  # 是否使用仿真时间（Gazebo）
    autostart = LaunchConfiguration('autostart')  # 是否自动启动导航节点
    params_file = LaunchConfiguration('params_file')  # 导入的参数文件
    use_composition = LaunchConfiguration('use_composition')  # 是否使用组件化（Composable Nodes）
    container_name = LaunchConfiguration('container_name')  # 组件容器名称
    container_name_full = (namespace, '/', container_name)  # 组件容器完全限定名称（带命名空间）
    use_respawn = LaunchConfiguration('use_respawn')  # 节点崩溃时是否重启
    log_level = LaunchConfiguration('log_level')  # 日志等级

    # 需要由生命周期管理器管理的节点名称列表
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # remappings设置，ROS里topic的重映射，主要针对tf和tf_static做绝对到相对路径的映射
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # 参数替换字典，用于动态替换参数文件中某些变量的值
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    # 生成一个用于参数调用的中间配置文件，基于params_file，并做参数重写
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,  # 参数文件里是否增加命名空间前缀
            param_rewrites=param_substitutions,  # 替换的参数字典
            convert_types=True),
        allow_substs=True)

    # 设置环境变量，启用日志行缓冲模式，方便实时查看日志输出
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # 接下来是声明launch文件的入口参数，方便用户通过命令行传入和指定
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # 如果未使用组件化启动（use_composition为False），则单独以节点形式启动所有导航包子节点
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),  # 条件：use_composition为False
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,  # 是否异常重启
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),  # 额外重映射cmd_vel
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),  # 额外cmd_vel重映射，方便velocity平滑
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    # 如果使用组件化（Composable Nodes），则把所有服务以组件节点的形式加载到同一个容器里
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),  # use_composition为True时执行
        target_container=container_name_full,  # 组件容器名称（带命名空间）
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',  # 组件节点对应的插件
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # 创建最终的LaunchDescription对象
    ld = LaunchDescription()

    # 设置环境变量动作，保证日志实时输出
    ld.add_action(stdout_linebuf_envvar)

    # 声明所有的launch入口参数（支持命令行参数）
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 根据是否使用组件化，加载对应的一整套导航功能节点
    ld.add_action(load_nodes)  # 非组件化节点
    ld.add_action(load_composable_nodes)  # 组件化节点

    # 返回launch描述对象
    return ld