#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT RViz 标尺节点 (Rviz_Ruler)
================================================================================
发布用于 RViz 可视化的标尺/参考坐标系标记（数字标记 + 刻度线 + 坐标轴线）。

话题:
  发布: /visualization/ruler   visualization_msgs/MarkerArray

连接关系:
  → Rviz_radar (pub)

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

RULER_AXIS     = 'x'           # 标尺方向: x | y
RULER_OFFSET   = -50.0         # 正交方向偏移 (m)
RULER_INTERVAL = 20.0          # 标记间隔 (m)
RULER_LENGTH   = 300.0         # 标尺总长度 (m)
RULER_FONT     = 0.8           # 字体大小
RULER_COLOR    = [0.8, 0.8, 0.8]   # RGB 颜色 (0~1)
PUBLISH_HZ     = 2.0           # 发布频率 (Hz)
FIXED_FRAME    = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from ft_framework.common import create_header


def create_ruler_markers(axis: str, offset: float, interval: float,
                          length: float, font_scale: float, color: list,
                          frame_id: str, stamp) -> MarkerArray:
    """创建坐标尺标记（数字 + 刻度线 + 轴线）"""
    marr = MarkerArray()
    n_marks = int(length / interval) + 1

    for i in range(n_marks):
        val = i * interval

        label = Marker()
        label.header = create_header(frame_id, stamp)
        label.ns = 'ruler_labels'
        label.id = i
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.scale.z = font_scale
        label.color.r = color[0]; label.color.g = color[1]
        label.color.b = color[2]; label.color.a = 1.0
        if axis == 'x':
            label.pose.position.x = val
            label.pose.position.y = offset
        else:
            label.pose.position.x = offset
            label.pose.position.y = val
        label.pose.position.z = 0.5
        label.pose.orientation.w = 1.0
        label.text = f'{val:.0f}'
        marr.markers.append(label)

        tick = Marker()
        tick.header = create_header(frame_id, stamp)
        tick.ns = 'ruler_ticks'
        tick.id = i
        tick.type = Marker.LINE_STRIP
        tick.action = Marker.ADD
        tick.scale.x = 0.05
        tick.color.r = color[0]; tick.color.g = color[1]
        tick.color.b = color[2]; tick.color.a = 0.6
        tick.pose.orientation.w = 1.0
        p1 = Point(); p2 = Point()
        if axis == 'x':
            p1.x = val; p1.y = offset - 1.0; p1.z = 0.0
            p2.x = val; p2.y = offset + 1.0; p2.z = 0.0
        else:
            p1.x = offset - 1.0; p1.y = val; p1.z = 0.0
            p2.x = offset + 1.0; p2.y = val; p2.z = 0.0
        tick.points = [p1, p2]
        marr.markers.append(tick)

    axis_line = Marker()
    axis_line.header = create_header(frame_id, stamp)
    axis_line.ns = 'ruler_axis'
    axis_line.id = 0
    axis_line.type = Marker.LINE_STRIP
    axis_line.action = Marker.ADD
    axis_line.scale.x = 0.08
    axis_line.color.r = color[0]; axis_line.color.g = color[1]
    axis_line.color.b = color[2]; axis_line.color.a = 0.8
    axis_line.pose.orientation.w = 1.0
    p_start = Point(); p_end = Point()
    if axis == 'x':
        p_start.x = 0.0;    p_start.y = offset; p_start.z = 0.0
        p_end.x   = length; p_end.y   = offset; p_end.z   = 0.0
    else:
        p_start.x = offset; p_start.y = 0.0;    p_start.z = 0.0
        p_end.x   = offset; p_end.y   = length; p_end.z   = 0.0
    axis_line.points = [p_start, p_end]
    marr.markers.append(axis_line)

    return marr


class RvizRulerNode(Node):
    """RViz 标尺节点"""

    def __init__(self):
        super().__init__('rviz_ruler')

        self.declare_parameter('ruler_axis', RULER_AXIS)
        self.declare_parameter('ruler_offset', RULER_OFFSET)
        self.declare_parameter('ruler_interval', RULER_INTERVAL)
        self.declare_parameter('ruler_length', RULER_LENGTH)
        self.declare_parameter('ruler_font', RULER_FONT)
        self.declare_parameter('ruler_color', RULER_COLOR)
        self.declare_parameter('publish_hz', PUBLISH_HZ)
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.ruler_axis     = self.get_parameter('ruler_axis').value
        self.ruler_offset   = float(self.get_parameter('ruler_offset').value)
        self.ruler_interval = float(self.get_parameter('ruler_interval').value)
        self.ruler_length   = float(self.get_parameter('ruler_length').value)
        self.ruler_font     = float(self.get_parameter('ruler_font').value)
        self.ruler_color    = list(
            float(v) for v in self.get_parameter('ruler_color').value)
        self.publish_hz     = float(self.get_parameter('publish_hz').value)
        self.fixed_frame    = self.get_parameter('fixed_frame').value

        self.pub_ruler = self.create_publisher(
            MarkerArray, '/visualization/ruler', 10)
        self.timer = self.create_timer(1.0 / self.publish_hz, self._on_timer)

        self.get_logger().info(
            f'Rviz_Ruler 启动: axis={self.ruler_axis}, '
            f'length={self.ruler_length}m, offset={self.ruler_offset}m')

    def _on_timer(self):
        stamp = self.get_clock().now().to_msg()
        ruler_arr = create_ruler_markers(
            self.ruler_axis, self.ruler_offset, self.ruler_interval,
            self.ruler_length, self.ruler_font, self.ruler_color,
            self.fixed_frame, stamp)
        self.pub_ruler.publish(ruler_arr)

    def destroy_node(self):
        self.get_logger().info('Rviz_Ruler 已停止')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RvizRulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Rviz_Ruler 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
