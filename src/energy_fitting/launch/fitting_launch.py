#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动 fitting 包的大能量机关拟合节点及其依赖节点。

依赖节点：
  - simulation/image_simulation_node  -> 发布 /camera/image_raw
  - openvino_infer/openvino_infer_node        -> 发布 /detection_keypoints
  - visualizer/visualizer_node        -> 可视化检测结果
  - pnp_solver/pnp_solver             -> PnP 解算
  - pose_transform/Pose_Transform_Node        -> 坐标变换
  - pose_transform/Inverse_Pose_Transform_Node -> 提供 /compute_target_location 服务
  - fitting/fitting_node              -> 常规拟合
  - fitting/big_energy                -> 大能量机关拟合，提供 /predict_energy_big 服务

编译与启动：
  colcon build --packages-select fitting
  source install/setup.bash
  ros2 launch fitting fitting_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ============================
    # Launch 参数声明
    # ============================
    prediction_dt_arg = DeclareLaunchArgument(
        'prediction_dt',
        default_value='1.0',
        description='大能量机关预测时间 (s)'
    )

    # ============================
    # 节点定义
    # ============================

    # 1. 图像仿真节点
    image_simulation_node = Node(
        package='simulation',
        executable='image_simulation_node',
        name='image_simulation_node',
        output='screen'
    )

    # 2. openvino 推理节点
    openvino_infer_node = Node(
        package='energy_openvino_infer',
        executable='openvino_infer_node',
        name='openvino_infer_node',
        output='screen'
    )

    # 3. 可视化节点
    visualizer_node = Node(
        package='visualizer',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen'
    )

    # 4. PnP 解算节点
    pnp_solver_node = Node(
        package='energy_pnp_solver',
        executable='pnp_solver',
        name='pnp_solver_node',
        output='screen'
    )

    # 5. PnP 可视化节点
    pnp_visualizer_node = Node(
        package='energy_pnp_solver',
        executable='pnp_visualizer',
        name='pnp_visualizer_node',
        output='screen'
    )

    # 6. 坐标变换节点 —— /pnp_poses -> /pnp_poses_ground（Inverse 节点的上游）
    pose_transform_node = Node(
        package='energy_pose_transform',
        executable='Pose_Transform_Node',
        name='pose_transform_node',
        output='screen'
    )

    # 7. 常规拟合节点
    fitting_node = Node(
        package='energy_fitting',
        executable='fitting_node',
        name='fitting_node',
        output='screen'
    )

    # 8. 逆坐标变换节点 —— 提供 /compute_target_location 服务
    inverse_pose_transform_node = Node(
        package='energy_pose_transform',
        executable='Inverse_Pose_Transform_Node',
        name='inverse_pose_transform_node',
        output='screen'
    )

    # 9. 大能量机关拟合节点 —— 提供 /predict_energy_big 服务
    big_energy_fitting_node = Node(
        package='energy_fitting',
        executable='big_energy',
        name='big_energy_fitting',
        output='screen',
        parameters=[{
            'window_size':    50,
            'update_rate':    10.0,
            'publish_rate':   10.0,
            'prediction_dt':  LaunchConfiguration('prediction_dt'),
            'radius':         2.0,
            'initial_a':      0.9,
            'initial_b':      1.94,
            'initial_theta0': 0.0,
            'min_a':          0.780,
            'max_a':          1.045,
            'min_b':          1.884,
            'max_b':          2.000,
        }]
    )

    # ============================
    # LaunchDescription
    # ============================
    return LaunchDescription([
        # 参数声明
        prediction_dt_arg,

        # 1. 图像仿真节点（立即启动）
        image_simulation_node,

        # 2. openvino 推理节点（延迟 2s，等待图像节点）
        TimerAction(
            period=2.0,
            actions=[openvino_infer_node]
        ),

        # 3. 可视化节点（延迟 3s）
        TimerAction(
            period=3.0,
            actions=[visualizer_node]
        ),

        # 4. PnP 解算 + PnP 可视化（延迟 4s）
        TimerAction(
            period=4.0,
            actions=[pnp_solver_node, pnp_visualizer_node]
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
    ])
