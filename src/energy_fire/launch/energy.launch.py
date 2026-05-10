#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一能量机关启动文件。

根据电控发送的 packet.mode，由串口驱动发布 /robot_mode 话题（std_msgs/UInt8）：
  0 = SMALL_ENERGY ->  little_energy_client 激活，big_energy_client 待机
  1 = BIG_ENERGY   ->  big_energy_client   激活，little_energy_client 待机
  2 = ARMOR        ->  两个 client 均待机

所有节点同时启动，模式切换由各 client 节点内部的 /robot_mode 订阅实时响应。

编译与启动：
  colcon build --packages-select fire jlcv_serial_driver
  source install/setup.bash
  ros2 launch fire energy.launch.py
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():

    # ============================
    # 加载相机参数配置（与 vision_bringup.launch.py 保持一致）
    # ============================
    _node_params = os.path.join(
        get_package_share_directory('vision_bringup'), 'config', 'node_params.yaml'
    )
    with open(_node_params, 'r') as f:
        camera_params = yaml.safe_load(f)['/camera_node']['ros__parameters']

    # ============================
    # Launch 参数声明
    # ============================
    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='1.0',
        description='初始预测时间 (s)，会被弹道解算的飞行时间动态更新'
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

    # ============================
    # 公共依赖：串口驱动 launch
    # ============================
    serial_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('jlcv_serial_driver'),
                'launch',
                'serial_driver.launch.py'
            )
        )
    )

    # ============================
    # 公共节点定义
    # ============================

    # 1+2. 相机节点 + OpenVINO 推理节点：放入同一容器，启用进程内零拷贝通信
    camera_infer_container = ComposableNodeContainer(
        name='camera_infer_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='hik_camera_node',
                parameters=[camera_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='energy_openvino_infer',
                plugin='OpenVINOInferenceNode',
                name='openvino_infer_node',
                parameters=[{'enable_r_debug_mode': True}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    # 3. PnP 解算节点
    pnp_solver_node = Node(
        package='energy_pnp_solver',
        executable='pnp_solver',
        name='pnp_solver_node',
        output='screen'
    )

    # 4. 坐标变换节点 —— /pnp_poses -> /pnp_poses_ground
    pose_transform_node = Node(
        package='energy_pose_transform',
        executable='Pose_Transform_Node',
        name='pose_transform_node',
        output='screen'
    )

    # 5. 常规拟合节点
    fitting_node = Node(
        package='energy_fitting',
        executable='fitting_node',
        name='fitting_node',
        output='screen'
    )

    # 6. 逆坐标变换节点 —— 提供 /compute_target_location 服务
    inverse_pose_transform_node = Node(
        package='energy_pose_transform',
        executable='Inverse_Pose_Transform_Node',
        name='inverse_pose_transform_node',
        output='screen'
    )

    # ============================
    # 大能量机关专属节点
    # ============================

    # 大能量拟合节点 —— 提供 /predict_energy_big 服务
    big_energy_fitting_node = Node(
        package='energy_fitting',
        executable='big_energy',
        name='big_energy_fitting',
        output='screen',
        parameters=[{
            'prediction_dt': LaunchConfiguration('dt'),
        }]
    )

    # 大能量预测节点
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

    # 大能量火控客户端（订阅 /robot_mode，mode==1 时激活）
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
    # 小能量机关专属节点
    # ============================

    # 小能量拟合节点 —— 提供 /predict_energy 服务
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

    # 小能量火控客户端（订阅 /robot_mode，mode==0 时激活）
    little_energy_client_node = Node(
        package='energy_fire',
        executable='little_energy_client',
        name='little_energy_client',
        output='screen',
        parameters=[{
            'dt':           LaunchConfiguration('dt'),
            'call_rate':    LaunchConfiguration('call_rate'),
            'bullet_speed': LaunchConfiguration('bullet_speed'),
        }]
    )

    # ============================
    # LaunchDescription（统一时序）
    # ============================
    return LaunchDescription([
        # 参数声明
        dt_arg,
        call_rate_arg,
        bullet_speed_arg,

        # 0. 串口驱动（立即启动，发布 /robot_mode）
        serial_driver_launch,

        # 1+2. 相机 + OpenVINO 容器（延迟 2s，进程内通信零拷贝）
        TimerAction(period=2.0, actions=[camera_infer_container]),

        # 3. PnP 解算（延迟 4s）
        TimerAction(period=4.0, actions=[pnp_solver_node]),

        # 4. 坐标变换（延迟 5s）
        TimerAction(period=5.0, actions=[pose_transform_node]),

        # 5. 常规拟合 + 逆坐标变换（延迟 6s）
        TimerAction(period=6.0, actions=[fitting_node, inverse_pose_transform_node]),

        # 6. 大能量拟合服务 + 小能量拟合服务（延迟 7s）
        TimerAction(period=7.0, actions=[big_energy_fitting_node, little_energy_fitting_node]),

        # 7. 大能量预测节点（延迟 8s）
        TimerAction(period=8.0, actions=[big_energy_predict_node]),

        # 8. 两个 client 节点同时启动（延迟 10s，等待所有服务就绪）
        #    各节点通过订阅 /robot_mode 自动决定是否激活
        TimerAction(period=10.0, actions=[big_energy_client_node, little_energy_client_node]),
    ])
