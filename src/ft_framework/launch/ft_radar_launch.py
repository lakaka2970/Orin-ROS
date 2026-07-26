#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT Radar 框架启动文件 — V2 架构
================================================================================
V2 架构变更:
  - Logging 内置化: 数据采集层 (C++ rx 节点) 内置 Logging, 无独立 logging_node
  - 仅 GPU RSP: 移除 rsp_mil_python, 仅保留 rsp_cuda
  - 新增 system_monitor: 系统监控节点
  - 运行模式: FT_DEBUG_MODE (含 Logging) / FT_RUNNING_MODE (仅实时处理)
  - Logging 模式: ADC_MODE / RD_CELL_LIST_MODE / DET_LIST_MODE / IDLE_MODE
  - 文件路径发布: ADC/Camera 通过文件路径消息通信, DDS 带宽降低 99.99%

节点列表 (V2):
  第一层 (数据采集, C++):  adc_rx, camera_rx, vehicle_data_rx
  第二层 (信号处理, Python): rsp_cuda
  第三层 (可视化):          rviz_radar, rviz_image, rviz_ruler
  第四层 (高级感知):        object_detection_3d (可选)
  系统监控:                 system_monitor

用法:
  ros2 launch ft_framework ft_radar_launch.py
  ros2 launch ft_framework ft_radar_launch.py operation_mode:=FT_RUNNING_MODE
  ros2 launch ft_framework ft_radar_launch.py logging_mode:=RD_CELL_LIST_MODE
  ros2 launch ft_framework ft_radar_launch.py no_adc:=true

作者: zhengyuan.liu
日期: 2026-07-26 (V2)
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def _load_yaml_config():
    """从默认路径加载全局 YAML 配置文件。"""
    config_path = os.path.join(os.getcwd(), 'config', 'ft_radar_params.yaml')
    if not os.path.exists(config_path):
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    with open(config_path, 'r') as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    ld = LaunchDescription()

    # ========================================================================
    # 加载 YAML 配置
    # ========================================================================
    yaml_config = _load_yaml_config()

    adc_cfg      = yaml_config.get('adc_rx', {})
    camera_cfg   = yaml_config.get('camera_rx', {})
    vehicle_cfg  = yaml_config.get('vehicle_data_rx', {})
    rsp_cfg      = yaml_config.get('rsp', {})
    rsp_cu_cfg   = rsp_cfg.get('cuda', {})
    obj_cfg      = yaml_config.get('object_detection_3d', {})
    ruler_cfg    = yaml_config.get('rviz_ruler', {})
    rviz_radar_cfg = yaml_config.get('rviz_radar', {})
    rviz_image_cfg = yaml_config.get('rviz_image', {})
    system_cfg   = yaml_config.get('system', {})

    # ========================================================================
    # Launch 参数声明
    # ========================================================================
    ld.add_action(DeclareLaunchArgument(
        'operation_mode', default_value='FT_DEBUG_MODE',
        description='运行模式: FT_DEBUG_MODE (含Logging) | FT_RUNNING_MODE (仅实时处理)'))

    ld.add_action(DeclareLaunchArgument(
        'logging_mode', default_value='ADC_MODE',
        description='Logging模式: ADC_MODE | RD_CELL_LIST_MODE | DET_LIST_MODE | IDLE_MODE'))

    ld.add_action(DeclareLaunchArgument(
        'logging_output_dir', default_value='',
        description='Logging输出目录 (空=自动检测NVMe/eMMC)'))

    ld.add_action(DeclareLaunchArgument(
        'no_adc', default_value='false',
        description='不启动 adc_rx 节点: true | false'))

    ld.add_action(DeclareLaunchArgument(
        'enable_object_detection', default_value='true',
        description='启用 3D 目标检测节点: true | false'))

    ld.add_action(DeclareLaunchArgument(
        'enable_rviz', default_value='true',
        description='启用可视化节点: true | false'))

    # ========================================================================
    # 模式引用
    # ========================================================================
    operation_mode = LaunchConfiguration('operation_mode')
    logging_mode   = LaunchConfiguration('logging_mode')
    logging_output_dir = LaunchConfiguration('logging_output_dir')
    no_adc         = LaunchConfiguration('no_adc')
    enable_obj     = LaunchConfiguration('enable_object_detection')
    enable_rviz    = LaunchConfiguration('enable_rviz')

    adc_enabled = PythonExpression(["'", no_adc, "' != 'true'"])

    # ========================================================================
    # 启动日志
    # ========================================================================
    ld.add_action(LogInfo(
        msg=['=== FT Radar V2: operation_mode=[', operation_mode,
             ']  logging_mode=[', logging_mode, '] ===']))

    # ========================================================================
    # 第一层: 数据采集层 (C++ rx 节点, 内置 Logging)
    # ========================================================================

    # 公共参数: 运行模式 + Logging 配置
    rx_common_params = {
        'operation_mode': operation_mode,
        'logging_mode': logging_mode,
        'logging_output_dir': logging_output_dir,
    }

    ld.add_action(Node(
        package='ft_rx_cpp', executable='adc_rx_cpp', name='adc_rx',
        output='screen',
        condition=IfCondition(adc_enabled),
        parameters=[{**adc_cfg, **rx_common_params, 'use_sim_time': False}]))

    ld.add_action(Node(
        package='ft_rx_cpp', executable='camera_rx_cpp', name='camera_rx',
        output='screen',
        parameters=[{**camera_cfg, **rx_common_params, 'use_sim_time': False}]))

    ld.add_action(Node(
        package='ft_rx_cpp', executable='vehicle_data_rx_cpp',
        name='vehicle_data_rx', output='screen',
        parameters=[{**vehicle_cfg, **rx_common_params, 'use_sim_time': False}]))

    # ========================================================================
    # 第二层: 信号处理层 (GPU RSP)
    # ========================================================================
    ld.add_action(TimerAction(
        period=0.5,
        actions=[Node(
            package='ft_framework', executable='rsp_cuda',
            name='rsp_cuda', output='screen',
            parameters=[{**rsp_cu_cfg, 'use_sim_time': False}])]))

    # ========================================================================
    # 第三层: 可视化层
    # ========================================================================
    ld.add_action(TimerAction(period=1.0, actions=[Node(
        package='ft_framework', executable='rviz_radar',
        name='rviz_radar', output='screen',
        condition=IfCondition(enable_rviz),
        parameters=[rviz_radar_cfg])]))

    ld.add_action(TimerAction(period=1.0, actions=[Node(
        package='ft_framework', executable='rviz_image',
        name='rviz_image', output='screen',
        condition=IfCondition(enable_rviz),
        parameters=[rviz_image_cfg])]))

    ld.add_action(TimerAction(period=1.0, actions=[Node(
        package='ft_framework', executable='rviz_ruler',
        name='rviz_ruler', output='screen',
        condition=IfCondition(enable_rviz),
        parameters=[ruler_cfg])]))

    # ========================================================================
    # 第四层: 高级感知层 (可选)
    # ========================================================================
    ld.add_action(TimerAction(period=1.5, actions=[Node(
        package='ft_framework', executable='object_detection_3d',
        name='object_detection_3d', output='screen',
        condition=IfCondition(enable_obj),
        parameters=[obj_cfg])]))

    # ========================================================================
    # 系统监控
    # ========================================================================
    ld.add_action(TimerAction(period=2.0, actions=[Node(
        package='ft_framework', executable='system_monitor',
        name='system_monitor', output='screen',
        parameters=[{'use_sim_time': False}])]))

    # ========================================================================
    # 完成日志
    # ========================================================================
    ld.add_action(LogInfo(
        msg='=== FT Radar V2: 全部节点已提交启动 ==='))

    return ld
