#!/usr/bin/env python3
# Copyright (c) 2023 Your Name
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('wifi_loc')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    bag_path = LaunchConfiguration('bag_path')
    node_startup_delay = LaunchConfiguration('node_startup_delay', default='2.0')  # 设置延迟时间，默认6秒
    use_true_ap_positions = LaunchConfiguration('use_true_ap_positions', default='true')
    
    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/jay/AGLoc_ws/rosbag/corrior_01',
        description='Path to the ROS2 bag directory'
    )
    
    declare_node_startup_delay = DeclareLaunchArgument(
        'node_startup_delay',
        default_value='6.0',
        description='Delay in seconds before starting rosbag playback to allow node initialization'
    )
    
    declare_use_true_ap_positions = DeclareLaunchArgument(
        'use_true_ap_positions',
        default_value='true',
        description='Use true AP positions instead of estimated positions if true'
    )
    
    # 创建 robot_loc 节点
    robot_loc_node = Node(
        package='wifi_loc',
        executable='robot_loc',
        name='robot_loc',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_true_ap_positions': use_true_ap_positions
        }]
    )
    
    # 创建日志信息，表示节点启动
    log_node_started = LogInfo(
        msg="Robot localizer node has started, waiting for initialization..."
    )
    
    # 创建播放 rosbag 的进程，但在节点启动后延迟一段时间再执行
    play_rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
        output='screen'
    )
    
    # 创建一个延迟动作，在节点启动后等待指定时间再播放rosbag
    delayed_rosbag = TimerAction(
        period=node_startup_delay,
        actions=[
            LogInfo(msg="Starting rosbag playback now..."),
            play_rosbag
        ]
    )
    
    # 注册事件处理器，当robot_loc节点启动后触发延迟动作
    start_rosbag_after_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_loc_node,
            on_start=[
                log_node_started,
                delayed_rosbag
            ]
        )
    )
    
    # 创建事件处理器，在 robot_loc 节点退出时终止 rosbag 播放
    rosbag_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_loc_node,
            on_exit=[ExecuteProcess(
                cmd=['pkill', '-f', 'ros2 bag play'],
                output='screen'
            )]
        )
    )
    
    # 返回 LaunchDescription
    return LaunchDescription([
        declare_use_sim_time,
        declare_bag_path,
        declare_node_startup_delay,
        declare_use_true_ap_positions,
        robot_loc_node,
        start_rosbag_after_node,
        rosbag_exit_event_handler
    ])
