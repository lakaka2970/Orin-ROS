#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT Framework — 共享工具模块
================================================================================
所有节点共用的工具函数，避免代码重复。

使用方式:
  from ft_framework.common import monotonic_us_stamp, create_header

作者: zhengyuan.liu
日期: 2026.6.9
================================================================================
"""

import time

from std_msgs.msg import Header


def monotonic_us_stamp() -> tuple:
    """
    获取单调时钟的微秒时间戳。

    使用 time.monotonic_ns() 作为时钟源，精度为纳秒。
    返回 (sec, nanosec) 元组，可直接用于 ROS2 Header.stamp。

    用途:
      - 所有 Rx 节点在数据采集第一时间注入时间戳
      - 后续节点透传此时间戳，不得覆盖
      - Logging 节点从此时间戳提取微秒整数用于文件命名
    """
    now_ns = time.monotonic_ns()
    sec = int(now_ns // 1_000_000_000)
    nsec = int(now_ns % 1_000_000_000)
    return (sec, nsec)


def create_header(frame_id: str, stamp) -> Header:
    """
    创建 ROS2 标准消息头。

    参数:
      frame_id: 坐标系名称 (如 'radar', 'map', 'camera')
      stamp:    builtin_interfaces/Time 对象（来自 msg.header.stamp 或
                self.get_clock().now().to_msg()）

    返回:
      std_msgs/Header，包含 frame_id 和时间戳

    注意:
      stamp 必须是 rclpy Time 对象，不能直接传入 (sec, nsec) 元组。
    """
    h = Header()
    h.frame_id = frame_id
    h.stamp = stamp
    return h
