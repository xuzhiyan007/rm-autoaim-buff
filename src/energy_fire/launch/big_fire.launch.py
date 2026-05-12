#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动 fire 包的 big_energy_client 节点及其依赖服务节点。

依赖节点：
  - fitting/big_energy             -> 提供 /predict_energy 服务
  - pose_transform/Inverse_Pose_Transform_Node -> 提供 /compute_target_location 服务

编译与启动：
  colcon build --packages-select fire
  source install/setup.bash
  ros2 launch fire big_fire.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

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

    enable_mode_gate_arg = DeclareLaunchArgument(
        'enable_mode_gate',
        default_value='false',
        description='是否启用 /robot_mode 门控（false=跳过模式检查，始终运行）'
    )

    # ============================
    # 包含外部 Launch 文件
    # ============================

    # 串口驱动 launch
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
    # 节点定义
    # ============================

    # 1. 图像节点
    image_simulation_node = Node(
        package='hik_camera',
        executable='hik_camera_node',
        name='hik_camera_node',
        output='screen'
    )

    # 2. OpenVINO 推理节点
    openvino_infer_node = Node(
        package='energy_openvino_infer',
        executable='openvino_infer_node',
        name='openvino_infer_node',
        output='screen',
        parameters=[{
            'enable_r_debug_mode': True,
            'enable_mode_gate': LaunchConfiguration('enable_mode_gate'),
        }]
    )


    # 4. PnP 解算节点
    pnp_solver_node = Node(
        package='energy_pnp_solver',
        executable='pnp_solver',
        name='pnp_solver_node',
        output='screen'
    )

    # # 5. PnP 可视化节点
    # pnp_visualizer_node = Node(
    #     package='energy_pnp_solver',
    #     executable='pnp_visualizer',
    #     name='pnp_visualizer_node',
    #     output='screen'
    # )

    # 6. 坐标变换节点 —— /pnp_poses -> /pnp_poses_ground（Inverse 节点的上游）
    pose_transform_node = Node(
        package='energy_pose_transform',
        executable='Pose_Transform_Node',
        name='pose_transform_node',
        output='screen'
    )

    # 8. 常规拟合节点
    fitting_node = Node(
        package='energy_fitting',
        executable='fitting_node',
        name='fitting_node',
        output='screen'
    )

    # 9. 逆坐标变换节点 —— 提供 /compute_target_location 服务
    inverse_pose_transform_node = Node(
        package='energy_pose_transform',
        executable='Inverse_Pose_Transform_Node',
        name='inverse_pose_transform_node',
        output='screen'
    )

    # 10. 大能量机关拟合节点 —— 提供 /predict_energy_big 服务
    big_energy_fitting_node = Node(
        package='energy_fitting',
        executable='big_energy',
        name='big_energy_fitting',
        output='screen',
        parameters=[{
            'prediction_dt': LaunchConfiguration('dt'),
            'direction':     -1,      # +1=逆时针(CCW), -1=顺时针(CW)，必须与实际大能量轮旋转方向一致
            'window_size':   50,      # Ceres 滑动窗口大小，增大可提高收敛稳定性
            'update_rate':   10.0,    # Ceres 优化触发频率 (Hz)
        }]
    )

    # 11. 大能量机关预测节点（核心节点）
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

    # 12. 大能量机关火控客户端节点
    big_energy_client_node = Node(
        package='energy_fire',
        executable='big_energy_client',
        name='big_energy_client',
        output='screen',
        parameters=[{
            'dt':               LaunchConfiguration('dt'),
            'call_rate':        LaunchConfiguration('call_rate'),
            'bullet_speed':     LaunchConfiguration('bullet_speed'),
            'enable_mode_gate': LaunchConfiguration('enable_mode_gate'),
        }]
    )

    # ============================
    # LaunchDescription
    # ============================
    return LaunchDescription([
        # 参数声明
        dt_arg,
        call_rate_arg,
        bullet_speed_arg,
        enable_mode_gate_arg,

        # 0. 串口驱动（立即启动）big_energy_predict
        serial_driver_launch,

        # 1. 图像仿真节点（立即启动）
        image_simulation_node,

        # 2. OpenVINO 推理节点（延迟 2s，等待图像节点）
        TimerAction(
            period=2.0,
            actions=[openvino_infer_node]
        ),

        # 4. PnP 解算（延迟 4s）
        TimerAction(
            period=4.0,
            actions=[pnp_solver_node]
        ),

        # 5. 坐标变换（延迟 5s，依赖 PnP 输出）
        TimerAction(
            period=5.0,
            actions=[pose_transform_node]
        ),

        # 6. 常规拟合 + 逆坐标变换（延迟 6s，依赖 /pnp_poses_ground）
        TimerAction(
            period=6.0,
            actions=[fitting_node, inverse_pose_transform_node]
        ),

        # 7. 大能量拟合服务（延迟 7s）
        TimerAction(
            period=7.0,
            actions=[big_energy_fitting_node]
        ),

        # 8. 预测节点 + 火控客户端节点（延迟 9s，等待所有服务就绪）
        TimerAction(
            period=9.0,
            actions=[big_energy_predict_node, big_energy_client_node]
        ),
    ])
