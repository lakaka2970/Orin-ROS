#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT Framework 启动文件
================================================================================
启动全部 10 个 ROS2 节点，构成完整的雷达-相机-车辆数据融合感知框架。

使用方法：
  ros2 launch ft_framework ft_framework.launch.py

单独启动某个节点：
  ros2 run ft_framework <node_name>

节点列表（按层级）：
  第一层（数据采集）：    adc_rx, camera_rx, vehicle_data_rx
  第二层（信号处理）：    rsp_mil_python, rsp_cuda
  第三层（可视化/日志）：  rviz_radar, rviz_image, logging_node
  第四层（高级感知）：    object_detection_3d, rviz_ruler

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction


def generate_launch_description():
    """生成启动描述，按层级顺序启动所有节点"""

    ld = LaunchDescription()

    # ========================================================================
    # 启动日志
    # ========================================================================
    ld.add_action(LogInfo(
        msg='=== FT Framework: 启动 10 节点 ROS2 融合感知框架 ==='))

    # ========================================================================
    # 第一层：数据采集层 (3 个接收节点)
    # ========================================================================

    # Node 1: ADC Rx — 雷达 ADC 数据接收
    node_adc_rx = Node(
        package='ft_framework',
        executable='adc_rx',
        name='adc_rx',
        output='screen',
        parameters=[{
            'radar_fps': 10.0,
            'num_targets': 50,
            'range_max': 300.0,
            'range_min': 1.0,
            'azimuth_range': 90.0,
            'elevation_range': 15.0,
        }],
    )

    # Node 2: Camera Rx — 相机数据接收
    node_camera_rx = Node(
        package='ft_framework',
        executable='camera_rx',
        name='camera_rx',
        output='screen',
        parameters=[{
            'camera_fps': 15.0,
            'image_width': 1280,
            'image_height': 720,
        }],
    )

    # Node 3: Vehicle Data Rx — 车辆数据接收
    node_vehicle_rx = Node(
        package='ft_framework',
        executable='vehicle_data_rx',
        name='vehicle_data_rx',
        output='screen',
        parameters=[{
            'vehicle_fps': 20.0,
            'sim_speed_mean': 15.0,
            'sim_speed_std': 2.0,
            'sim_yaw_rate': 0.05,
        }],
    )

    # ========================================================================
    # 第二层：雷达信号处理层 (2 个并行处理节点)
    # ========================================================================

    # Node 4: R SP MIL Python — Python 雷达信号处理
    node_rsp_py = Node(
        package='ft_framework',
        executable='rsp_mil_python',
        name='rsp_mil_python',
        output='screen',
        parameters=[{
            'processing_fps': 10.0,
            'snr_threshold': 10.0,
            'velocity_scale': 0.5,
        }],
    )

    # Node 5: R SP Cuda — CUDA 雷达信号处理
    node_rsp_cu = Node(
        package='ft_framework',
        executable='rsp_cuda',
        name='rsp_cuda',
        output='screen',
        parameters=[{
            'processing_fps': 10.0,
            'snr_threshold': 8.0,     # CUDA 版使用更低 SNR 阈值（更高灵敏度）
            'velocity_scale': 0.5,
        }],
    )

    # ========================================================================
    # 第三层：可视化与日志层 (3 个节点)
    # ========================================================================

    # Node 6: Rviz_radar — 雷达可视化
    node_rviz_radar = Node(
        package='ft_framework',
        executable='rviz_radar',
        name='rviz_radar',
        output='screen',
        parameters=[{
            'min_z': -5.0,
            'max_z': 15.0,
            'marker_lifetime': 1.0,
            'publish_hz': 10.0,
        }],
    )

    # Node 7: Rviz_Image — 图像可视化
    node_rviz_image = Node(
        package='ft_framework',
        executable='rviz_image',
        name='rviz_image',
        output='screen',
        parameters=[{
            'show_overlay': True,
        }],
    )

    # Node 8: Logging — 数据日志记录
    node_logging = Node(
        package='ft_framework',
        executable='logging_node',
        name='logging_node',
        output='screen',
        parameters=[{
            'status_log_interval': 5.0,
        }],
    )

    # ========================================================================
    # 第四层：高级感知与辅助层 (2 个节点)
    # ========================================================================

    # Node 9: 3D Object Detection — 3D 目标检测
    node_obj_det = Node(
        package='ft_framework',
        executable='object_detection_3d',
        name='object_detection_3d',
        output='screen',
        parameters=[{
            'cluster_distance': 5.0,
            'min_cluster_size': 3,
            'box_height': 2.0,
            'marker_lifetime': 1.0,
        }],
    )

    # Node 10: Rviz_Ruler — 标尺参考
    node_ruler = Node(
        package='ft_framework',
        executable='rviz_ruler',
        name='rviz_ruler',
        output='screen',
        parameters=[{
            'ruler_axis': 'x',
            'ruler_offset': -50.0,
            'ruler_interval': 20.0,
            'ruler_length': 300.0,
            'ruler_font': 0.8,
            'ruler_color': [0.8, 0.8, 0.8],
        }],
    )

    # ========================================================================
    # 按层级顺序启动（使用 TimerAction 确保上游先就绪）
    # ========================================================================

    # 第一层：立即启动
    ld.add_action(node_adc_rx)
    ld.add_action(node_camera_rx)
    ld.add_action(node_vehicle_rx)

    # 第二层：延迟 0.5s 启动（等待上游发布者就绪）
    ld.add_action(TimerAction(period=0.5, actions=[node_rsp_py]))
    ld.add_action(TimerAction(period=0.5, actions=[node_rsp_cu]))

    # 第三层：延迟 1.0s 启动
    ld.add_action(TimerAction(period=1.0, actions=[node_rviz_radar]))
    ld.add_action(TimerAction(period=1.0, actions=[node_rviz_image]))
    ld.add_action(TimerAction(period=1.0, actions=[node_logging]))

    # 第四层：延迟 1.5s 启动
    ld.add_action(TimerAction(period=1.5, actions=[node_obj_det]))
    ld.add_action(TimerAction(period=1.5, actions=[node_ruler]))

    ld.add_action(LogInfo(
        msg='=== FT Framework: 全部 10 个节点已提交启动 ==='))

    return ld
