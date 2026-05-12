#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全系统统一启动文件(装甲板 + 能量机关同时运行).

策略:
  - 来自 vision_bringup.launch.py:robot_state_publisher,serial_driver_node,armor_processor_node
  - 来自 energy.launch.py:所有能量机关节点及时序
  - 冲突解决:hik_camera 只启动一次,与 armor_detector + openvino_infer 放入同一容器,
    三个节点共享进程内零拷贝通信(use_intra_process_comms: True)

模式控制(packet.mode -> /robot_mode UInt8):
  0 = SMALL_ENERGY ->  little_energy_client 激活
  1 = BIG_ENERGY   ->  big_energy_client   激活
  2 = ARMOR        ->  armor_processor 工作，两个 energy client 均待机

启动时序:
  立即   -> robot_state_publisher(TF 骨架,来自 vision_bringup)
  立即   -> joint_state_publisher(use_serial=False 时)
  t=1s  -> serial_driver_node(唯一,来自 vision_bringup/common.py)
  t=1.5s-> armor_processor_node(装甲板跟踪+弹道解算)
  t=2s  -> [camera_full_container](hik_camera + armor_detector + openvino_infer)
  t=4s  -> pnp_solver_node
  t=5s  -> pose_transform_node
  t=6s  -> fitting_node + inverse_pose_transform_node
  t=7s  -> big_energy_fitting_node + little_energy_fitting_node
  t=8s  -> big_energy_predict_node
  t=10s -> big_energy_client + little_energy_client

编译与启动:
  colcon build --packages-select fire jlcv_serial_driver armor_detector armor_processor \
               openvino_infer pnp_solver pose_transform fitting vision_bringup
  source install/setup.bash
  ros2 launch fire full_system.launch.py
"""

import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

# ============================
# 复用 vision_bringup/launch/common.py 的公共定义
# ============================
sys.path.append(os.path.join(get_package_share_directory('vision_bringup'), 'launch'))
from common import (
    robot_state_publisher,
    joint_state_publisher,
    armor_processor_node,
    serial_driver_node,
    declare_use_serial_cmd,
    node_params,
)


def generate_launch_description():

    # ============================
    # 加载相机 / 装甲板检测参数
    # ============================
    with open(node_params, 'r') as f:
        _all_params = yaml.safe_load(f)

    camera_params = _all_params['/camera_node']['ros__parameters']
    armor_params  = _all_params['/armor_detector']['ros__parameters']

    # ============================
    # Launch 参数声明(来自 energy.launch.py)
    # ============================
    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='1.0',
        description='初始预测时间 (s),会被弹道解算的飞行时间动态更新'
    )

    call_rate_arg = DeclareLaunchArgument(
        'call_rate',
        default_value='20.0',
        description='服务调用频率 (Hz)'
    )

    bullet_speed_arg = DeclareLaunchArgument(
        'bullet_speed',
        default_value='22.0',
        description='子弹初速度 (m/s)'
    )

    clockwise_arg = DeclareLaunchArgument(
        'clockwise',
        default_value='true',
        description='能量机关旋转方向: true=顺时针, false=逆时针'
    )

    # ============================
    # 三合一相机容器(方案 A 核心)
    # hik_camera + armor_detector + openvino_infer 共享进程,全部零拷贝
    # ============================
    camera_full_container = ComposableNodeContainer(
        name='camera_full_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 1. 相机节点(来自 vision_bringup)
            ComposableNode(
                package='hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='camera_node',
                parameters=[camera_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # 2. 装甲板检测节点(来自 vision_bringup)
            ComposableNode(
                package='armor_detector',
                plugin='auto_aim::LifecycleArmorDetector',
                name='armor_detector',
                parameters=[armor_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # 3. OpenVINO 能量机关推理节点(来自 energy.launch.py)
            ComposableNode(
                package='energy_openvino_infer',
                plugin='OpenVINOInferenceNode',
                name='openvino_infer_node',
                parameters=[{'enable_r_debug_mode': False}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    # ============================
    # PnP 解算节点(来自 energy.launch.py)
    # ============================
    pnp_solver_node = Node(
        package='energy_pnp_solver',
        executable='pnp_solver',
        name='pnp_solver_node',
        output='screen'
    )

    # ============================
    # 坐标变换节点(来自 energy.launch.py)
    # ============================
    pose_transform_node = Node(
        package='energy_pose_transform',
        executable='Pose_Transform_Node',
        name='pose_transform_node',
        output='screen'
    )

    # ============================
    # 常规拟合 + 逆坐标变换(来自 energy.launch.py)
    # ============================
    fitting_node = Node(
        package='energy_fitting',
        executable='fitting_node',
        name='fitting_node',
        output='screen'
    )

    inverse_pose_transform_node = Node(
        package='energy_pose_transform',
        executable='Inverse_Pose_Transform_Node',
        name='inverse_pose_transform_node',
        output='screen'
    )

    # ============================
    # 大能量机关专属节点(来自 energy.launch.py)
    # ============================
    big_energy_fitting_node = Node(
        package='energy_fitting',
        executable='big_energy',
        name='big_energy_fitting',
        output='screen',
        parameters=[{
            'prediction_dt': LaunchConfiguration('dt'),
        }]
    )

    big_energy_predict_node = Node(
        package='energy_fitting',
        executable='big_energy_predict',
        name='big_energy_predict',
        output='screen',
        parameters=[{
            'dt':           LaunchConfiguration('dt'),
            'call_rate':    LaunchConfiguration('call_rate'),
            'bullet_speed': LaunchConfiguration('bullet_speed'),
        }]
    )

    big_energy_client_node = Node(
        package='energy_fire',
        executable='big_energy_client',
        name='big_energy_client',
        output='screen',
        parameters=[{
            'dt':           LaunchConfiguration('dt'),
            'call_rate':    LaunchConfiguration('call_rate'),
            'bullet_speed': LaunchConfiguration('bullet_speed'),
        }]
    )

    # ============================
    # 小能量机关专属节点(来自 energy.launch.py)
    # ============================
    little_energy_fitting_node = Node(
        package='energy_fitting',
        executable='little_energy',
        name='little_energy_fitting',
        output='screen',
        parameters=[{
            'process_noise':     1e-5,
            'measurement_noise': 0.01,
            'initial_c':         0.0,
            'initial_p':         1.0,
            'prediction_dt':     LaunchConfiguration('dt'),
        }]
    )

    little_energy_client_node = Node(
        package='energy_fire',
        executable='little_energy_client',
        name='little_energy_client',
        output='screen',
        parameters=[{
            'dt':           LaunchConfiguration('dt'),
            'call_rate':    LaunchConfiguration('call_rate'),
            'bullet_speed': LaunchConfiguration('bullet_speed'),
            'clockwise':    LaunchConfiguration('clockwise'),
        }]
    )

    # ============================
    # LaunchDescription(合并时序)
    # ============================
    return LaunchDescription([
        # ---- 参数声明 ----
        declare_use_serial_cmd,
        dt_arg,
        call_rate_arg,
        bullet_speed_arg,
        clockwise_arg,

        # ---- 立即启动:TF 骨架(来自 vision_bringup) ----
        robot_state_publisher,
        joint_state_publisher,        # use_serial=False 时才实际启动

        # ---- t=1s:串口驱动(唯一,来自 vision_bringup/common.py) ----
        TimerAction(period=1.0, actions=[serial_driver_node]),

        # ---- t=1.5s:装甲板处理节点(来自 vision_bringup) ----
        TimerAction(period=1.5, actions=[armor_processor_node]),

        # ---- t=2s:三合一相机容器(方案 A 核心,零拷贝) ----
        TimerAction(period=2.0, actions=[camera_full_container]),

        # ---- t=4s:PnP 解算(来自 energy.launch.py) ----
        TimerAction(period=4.0, actions=[pnp_solver_node]),

        # ---- t=5s:坐标变换(来自 energy.launch.py) ----
        TimerAction(period=5.0, actions=[pose_transform_node]),

        # ---- t=6s:拟合 + 逆坐标变换(来自 energy.launch.py) ----
        TimerAction(period=6.0, actions=[fitting_node, inverse_pose_transform_node]),

        # ---- t=7s:大/小能量拟合服务(来自 energy.launch.py) ----
        TimerAction(period=7.0, actions=[big_energy_fitting_node, little_energy_fitting_node]),

        # ---- t=8s:大能量预测节点(来自 energy.launch.py) ----
        TimerAction(period=8.0, actions=[big_energy_predict_node]),

        # ---- t=10s:两个 client 同时启动,由 /robot_mode 自主决定激活 ----
        TimerAction(period=10.0, actions=[big_energy_client_node, little_energy_client_node]),
    ])
