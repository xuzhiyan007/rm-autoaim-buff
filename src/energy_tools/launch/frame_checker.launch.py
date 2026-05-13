#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
帧时间戳调试工具启动文件 — 自带 hik_camera 发布节点，可独立测试时间戳一致性.

运行方式:
  ros2 launch energy_tools frame_checker.launch.py

设计:
  hik_camera + frame_timestamp_checker 放入同一 ComposableNodeContainer，
  启用 use_intra_process_comms 零拷贝通信，消除 DDS 序列化/传输引入的时间抖动。

可配置参数:
  topic_name              : 订阅的图片话题名 (默认 /image_raw)
  report_interval_frames  : 每隔多少帧输出一次报告 (默认 300)
  report_interval_seconds : 每隔多少秒输出一次报告 (默认 5.0)
  相机参数从 vision_bringup/config/node_params.yaml 加载
"""

import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():

    # ── 加载相机参数（与 full_system.launch.py 同源） ──
    sys.path.append(os.path.join(get_package_share_directory('vision_bringup'), 'launch'))
    from common import node_params

    with open(node_params, 'r') as f:
        camera_params = yaml.safe_load(f)['/camera_node']['ros__parameters']

    # ── 参数声明 ──
    topic_arg = DeclareLaunchArgument(
        'topic_name', default_value='/image_raw',
        description='订阅的图片话题名'
    )
    frames_arg = DeclareLaunchArgument(
        'report_interval_frames', default_value='300',
        description='每隔多少帧输出一次报告'
    )
    seconds_arg = DeclareLaunchArgument(
        'report_interval_seconds', default_value='5.0',
        description='每隔多少秒输出一次报告'
    )

    # ── 容器模式: hik_camera + frame_timestamp_checker 同进程零拷贝 ──
    checker_container = ComposableNodeContainer(
        name='frame_checker_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 1. 海康相机发布节点
            ComposableNode(
                package='hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='camera_node',
                parameters=[camera_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # 2. 帧时间戳检查节点
            ComposableNode(
                package='energy_tools',
                plugin='energy_tools::FrameTimestampChecker',
                name='frame_timestamp_checker',
                parameters=[{
                    'topic_name':              LaunchConfiguration('topic_name'),
                    'report_interval_frames':  LaunchConfiguration('report_interval_frames'),
                    'report_interval_seconds': LaunchConfiguration('report_interval_seconds'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        topic_arg,
        frames_arg,
        seconds_arg,
        checker_container,
    ])
