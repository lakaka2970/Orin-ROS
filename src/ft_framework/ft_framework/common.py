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


def filter_det_points(points: list,
                       roi_x: tuple = (-80.0, 200.0),
                       roi_y: tuple = (-40.0, 40.0),
                       roi_z: tuple = (-15.0, 15.0),
                       rcs_near_threshold: float = -40.0,
                       rcs_far_threshold: float = -20.0,
                       rcs_range_split: float = 10.0,
                       exist_prob_min: int = 30,
                       ambgt_prob_min: int = 40) -> list:
    """
    按 FT_radar_dataset_requirement.md §5.6 的 6 条规则过滤检测点。

    过滤规则:
      1. ROI 包围盒:   x∈[rx0, rx1], y∈[ry0, ry1], z∈[rz0, rz1]
      2. 高度自适应:   |z| >= |x| × 0.1 + 2 → 剔除
      3. RCS 分段:     range≤10m 且 RCS≤-40; range>10m 且 RCS≤-20 → 剔除
      4. 存在概率:     exist_prob < 30 → 剔除
      5. SNA 无效点:   idx == 255 → 剔除
      6. 模糊概率:     ambgt_prob < 40 → 剔除

    参数:
      points:        DetPoint 对象列表
      roi_x/y/z:     ROI 包围盒边界 (m)
      rcs_near/far_threshold:  近/远场 RCS 阈值 (dBsm)
      rcs_range_split:         近/远场分界距离 (m)
      exist_prob_min:          存在概率最小阈值
      ambgt_prob_min:          模糊概率最小阈值

    返回:
      过滤后的 DetPoint 列表及统计信息 dict:
        {'passed': [...], 'total': N, 'filtered': K,
         'roi': n1, 'height': n2, 'rcs': n3,
         'exist_prob': n4, 'sna': n5, 'ambgt_prob': n6}
    """
    stats = {'total': len(points), 'filtered': 0,
             'roi': 0, 'height': 0, 'rcs': 0,
             'exist_prob': 0, 'sna': 0, 'ambgt_prob': 0}

    passed = []
    for p in points:
        # 规则 1: ROI 包围盒
        if not (roi_x[0] <= p.x <= roi_x[1] and
                roi_y[0] <= p.y <= roi_y[1] and
                roi_z[0] <= p.z <= roi_z[1]):
            stats['roi'] += 1
            continue

        # 规则 2: 高度自适应过滤
        if abs(p.z) >= abs(p.x) * 0.1 + 2.0:
            stats['height'] += 1
            continue

        # 规则 3: RCS 分段过滤 (v2: rcs → rcs_db)
        if p.range <= rcs_range_split:
            if p.rcs_db <= rcs_near_threshold:
                stats['rcs'] += 1
                continue
        else:
            if p.rcs_db <= rcs_far_threshold:
                stats['rcs'] += 1
                continue

        # 规则 4: 存在概率 (v2: exist_prob → det_conf)
        if p.det_conf < exist_prob_min:
            stats['exist_prob'] += 1
            continue

        # 规则 5: SNA 无效点 (idx == 255)
        if p.idx == 255:
            stats['sna'] += 1
            continue

        # 规则 6: 模糊概率 (v2: ambgt_prob → det_ambig_state)
        # det_ambig_state: 0=无模糊, 1=轻微模糊, 2+=严重模糊 → 剔除
        if p.det_ambig_state >= ambgt_prob_min:
            stats['ambgt_prob'] += 1
            continue

        passed.append(p)

    stats['filtered'] = stats['total'] - len(passed)
    stats['passed'] = passed
    return passed, stats


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
