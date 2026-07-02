#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT Radar 框架启动文件 — 支持 4 种 RSP 启动模式 + 2 种 ADC 数据源
================================================================================
启动全部 10 个 ROS2 节点，构成完整的雷达-相机-车辆数据融合感知框架。

启动模式:
  python         仅启动 Python 版 RSP
  cuda           仅启动 CUDA 版 RSP（默认生产模式）
  both           双路并行，独立输出话题
  both_compare   双路并行 + 自动对比输出差异

ADC 数据源:
  real           从硬件设备读取真实 ADC 数据 (默认, /dev/adc_data)
  analog         噪声池 / .bin 文件预加载模拟

用法:
  ros2 launch ft_radar_launch.py                               # 默认 cuda + real
  ros2 launch ft_radar_launch.py rsp_mode:=python               # Python RSP
  ros2 launch ft_radar_launch.py adc_source:=analog             # 模拟 ADC
  ros2 launch ft_radar_launch.py rsp_mode:=cuda adc_source:=real

  # 自定义 Logging 开关
  ros2 launch ft_radar_launch.py rsp_mode:=cuda \
    log_adc:=true log_image:=false

节点列表（按层级）:
  第一层（数据采集）:    adc_rx, camera_rx, vehicle_data_rx
  第二层（信号处理）:    rsp_mil_python, rsp_cuda（按 mode 条件启动）
  第三层（可视化）:      rviz_radar, rviz_image, logging_node
  第四层（高级感知）:    object_detection_3d, rviz_ruler

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


# ============================================================================
# YAML 配置加载
# ============================================================================

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


def _flatten_logging_cfg(cfg: dict) -> dict:
    """将嵌套的 logging YAML 配置平展为 ROS2 参数名。"""
    flat = {}
    for k, v in cfg.items():
        if k == 'max_frames':
            for mk, mv in v.items():
                flat[f'max_frames.{mk}'] = mv
        elif k == 'switches':
            for sk, sv in v.items():
                flat[f'enable_{sk}'] = sv
        else:
            flat[k] = v
    return flat


def _flatten_vehicle_cfg(cfg: dict) -> dict:
    """将嵌套的 vehicle_data_rx YAML 配置平展为 ROS2 参数名。"""
    flat = {}
    for k, v in cfg.items():
        if k == 'defaults':
            for dk, dv in v.items():
                flat[f'defaults.{dk}'] = dv
        else:
            flat[k] = v
    return flat


def generate_launch_description():
    ld = LaunchDescription()

    # ========================================================================
    # 加载 YAML 配置
    # ========================================================================

    yaml_config = _load_yaml_config()

    adc_cfg      = yaml_config.get('adc_rx', {})
    camera_cfg   = yaml_config.get('camera_rx', {})
    vehicle_cfg  = _flatten_vehicle_cfg(yaml_config.get('vehicle_data_rx', {}))
    rsp_cfg      = yaml_config.get('rsp', {})
    rsp_py_cfg   = rsp_cfg.get('python', {})
    rsp_cu_cfg   = rsp_cfg.get('cuda', {})
    obj_cfg      = yaml_config.get('object_detection_3d', {})
    ruler_cfg    = yaml_config.get('rviz_ruler', {})
    rviz_radar_cfg = yaml_config.get('rviz_radar', {})
    rviz_image_cfg = yaml_config.get('rviz_image', {})
    log_cfg      = _flatten_logging_cfg(yaml_config.get('logging', {}))
    system_cfg   = yaml_config.get('system', {})

    # YAML 默认 RSP mode → Launch 参数的默认值
    yaml_rsp_mode = rsp_cfg.get('default_mode', 'cuda')

    # ========================================================================
    # Launch 参数声明
    # ========================================================================

    ld.add_action(DeclareLaunchArgument(
        'rsp_mode', default_value=yaml_rsp_mode,
        description='RSP 启动模式: python | cuda (default) | both | both_compare'))

    ld.add_action(DeclareLaunchArgument(
        'log_adc', default_value='true',
        description='Logging: 是否录制 ADC 数据'))
    ld.add_action(DeclareLaunchArgument(
        'log_image', default_value='true',
        description='Logging: 是否录制 Image 数据'))
    ld.add_action(DeclareLaunchArgument(
        'log_det_list', default_value='true',
        description='Logging: 是否录制 Det_List 数据'))
    ld.add_action(DeclareLaunchArgument(
        'log_ego_motion', default_value='true',
        description='Logging: 是否录制 Ego_Motion 数据'))
    ld.add_action(DeclareLaunchArgument(
        'log_obj_list', default_value='true',
        description='Logging: 是否录制 Obj_List 数据'))
    ld.add_action(DeclareLaunchArgument(
        'rx_impl', default_value='cpp',
        description='Rx 节点实现: cpp (C++ 零拷贝) | python (Python 原版)'))
    ld.add_action(DeclareLaunchArgument(
        'adc_source', default_value='real',
        description='ADC 数据源: real (硬件设备) | analog (噪声池/文件模拟)'))
    ld.add_action(DeclareLaunchArgument(
        'no_adc', default_value='false',
        description='不启动 adc_rx 节点: true | false (default)'))

    # ========================================================================
    # 模式引用
    # ========================================================================

    rsp_mode   = LaunchConfiguration('rsp_mode')
    rx_impl    = LaunchConfiguration('rx_impl')
    adc_source = LaunchConfiguration('adc_source')
    no_adc     = LaunchConfiguration('no_adc')

    # 条件表达式
    python_enabled = PythonExpression([
        "'", rsp_mode, "' in ['python', 'both', 'both_compare']"
    ])
    cuda_enabled = PythonExpression([
        "'", rsp_mode, "' in ['cuda', 'both', 'both_compare']"
    ])
    use_cpp = PythonExpression(["'", rx_impl, "' == 'cpp'"])
    use_py  = PythonExpression(["'", rx_impl, "' == 'python'"])

    # adc_rx 启用条件: no_adc 不为 'true' 时才启动
    adc_enabled = PythonExpression(["'", no_adc, "' != 'true'"])
    # log_adc 有效值: no_adc 时强制 false (无 adc_rx → 无数据可录)
    log_adc_effective = PythonExpression([
        "'false' if '", no_adc, "' == 'true' else '",
        LaunchConfiguration('log_adc'), "'"])

    # ========================================================================
    # 包引用 (根据 rx_impl 选择)
    # ========================================================================

    adc_pkg = PythonExpression(["'ft_rx_cpp' if '", rx_impl, "' == 'cpp' else 'ft_framework'"])
    # ADC 可执行文件: cpp 实现下根据 adc_source 选择 real / analog
    adc_exe = PythonExpression([
        "'adc_rx_analog_cpp' if '", rx_impl, "' == 'cpp' and '", adc_source, "' == 'analog' ",
        "else 'adc_rx_cpp' if '", rx_impl, "' == 'cpp' ",
        "else 'adc_rx'"])
    cam_pkg = PythonExpression(["'ft_rx_cpp' if '", rx_impl, "' == 'cpp' else 'ft_framework'"])
    cam_exe = PythonExpression(["'camera_rx_cpp' if '", rx_impl, "' == 'cpp' else 'camera_rx'"])
    veh_pkg = PythonExpression(["'ft_rx_cpp' if '", rx_impl, "' == 'cpp' else 'ft_framework'"])
    veh_exe = PythonExpression(["'vehicle_data_rx_cpp' if '", rx_impl, "' == 'cpp' else 'vehicle_data_rx'"])

    # ========================================================================
    # 启动日志
    # ========================================================================

    ld.add_action(LogInfo(
        msg=['=== FT Radar Framework: rsp_mode=[', rsp_mode,
             ']  rx_impl=[', rx_impl,
             ']  adc_source=[', adc_source, '] ===']))

    # ========================================================================
    # 第一层：数据采集层 (3 个节点)
    # ========================================================================

    ld.add_action(Node(
        package=adc_pkg, executable=adc_exe, name='adc_rx',
        output='screen',
        condition=IfCondition(adc_enabled),
        parameters=[{**adc_cfg, 'fps': 10.0}]))
    ld.add_action(Node(
        package=cam_pkg, executable=cam_exe, name='camera_rx',
        output='screen',
        parameters=[{**camera_cfg, 'fps': 30.0}]))
    ld.add_action(Node(
        package=veh_pkg, executable=veh_exe,
        name='vehicle_data_rx', output='screen',
        parameters=[{**vehicle_cfg, 'fps': 50.0}]))

    # ========================================================================
    # 第二层：雷达信号处理层 (按 mode 条件启动)
    # ========================================================================

    ld.add_action(TimerAction(
        period=0.5,
        actions=[Node(
            package='ft_framework', executable='rsp_mil_python',
            name='rsp_mil_python', output='screen',
            condition=IfCondition(python_enabled),
            parameters=[{**rsp_py_cfg, 'rsp_mode': rsp_mode}])]))

    ld.add_action(TimerAction(
        period=0.5,
        actions=[Node(
            package='ft_framework', executable='rsp_cuda',
            name='rsp_cuda', output='screen',
            condition=IfCondition(cuda_enabled),
            parameters=[{**rsp_cu_cfg, 'rsp_mode': rsp_mode}])]))

    # ========================================================================
    # 第三层：可视化与日志层
    # ========================================================================

    ld.add_action(TimerAction(period=1.0, actions=[Node(
        package='ft_framework', executable='rviz_radar',
        name='rviz_radar', output='screen',
        parameters=[rviz_radar_cfg])]))

    ld.add_action(TimerAction(period=1.0, actions=[Node(
        package='ft_framework', executable='rviz_image',
        name='rviz_image', output='screen',
        parameters=[rviz_image_cfg])]))

    ld.add_action(TimerAction(period=1.0, actions=[Node(
        package='ft_framework', executable='logging_node',
        name='logging_node', output='screen',
        parameters=[{
            **log_cfg,
            'enable_adc':        log_adc_effective,
            'enable_image':      LaunchConfiguration('log_image'),
            'enable_det_list':   LaunchConfiguration('log_det_list'),
            'enable_ego_motion': LaunchConfiguration('log_ego_motion'),
            'enable_obj_list':   LaunchConfiguration('log_obj_list'),
        }])]))

    # ========================================================================
    # 第四层：高级感知与辅助层
    # ========================================================================

    ld.add_action(TimerAction(period=1.5, actions=[Node(
        package='ft_framework', executable='object_detection_3d',
        name='object_detection_3d', output='screen',
        parameters=[obj_cfg])]))

    ld.add_action(TimerAction(period=1.5, actions=[Node(
        package='ft_framework', executable='rviz_ruler',
        name='rviz_ruler', output='screen',
        parameters=[ruler_cfg])]))

    # ========================================================================
    # 完成日志
    # ========================================================================

    ld.add_action(LogInfo(
        msg='=== FT Radar Framework: 全部节点已提交启动 ==='))

    return ld
