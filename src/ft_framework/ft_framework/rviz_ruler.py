#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT RViz 标尺节点 (Rviz_Ruler)
================================================================================
发布用于 RViz 可视化的标尺/参考坐标系标记，为雷达可视化提供空间参考。

发布话题：
  /ft/ruler   MarkerArray   标尺数字标记 + 刻度线 + 坐标轴线

连接关系：
  → Rviz_radar (pub)

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 标尺参数 ----------
RULER_AXIS     = 'x'         # 坐标尺方向：'x' 沿 X 轴 / 'y' 沿 Y 轴
RULER_OFFSET   = -50.0       # 坐标尺沿正交方向的偏移量 (m)
RULER_INTERVAL = 20.0        # 相邻标记间隔 (m)
RULER_LENGTH   = 300.0       # 坐标尺总长度 (m)，从原点向正方向延伸
RULER_FONT     = 0.8         # 字体大小 (scale.z)
RULER_COLOR    = [0.8, 0.8, 0.8]   # RGB 颜色 (0~1)

# ---------- 发布参数 ----------
PUBLISH_HZ = 2.0             # 发布频率 (Hz)

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# ============================================================================
# 工具函数
# ============================================================================

def create_header(frame_id: str, stamp) -> Header:
    """创建 ROS2 消息头"""
    h = Header()
    h.frame_id = frame_id
    h.stamp = stamp
    return h


def create_ruler_markers(axis: str, offset: float, interval: float,
                          length: float, font_scale: float, color: list,
                          frame_id: str, stamp) -> MarkerArray:
    """
    创建坐标尺标记，包含：
    - 数字标记（TEXT_VIEW_FACING）
    - 刻度线（LINE_STRIP）
    - 坐标轴主线（LINE_STRIP）
    """
    marr = MarkerArray()
    n_marks = int(length / interval) + 1

    for i in range(n_marks):
        val = i * interval

        # ---- 数字标记 ----
        m = Marker()
        m.header = create_header(frame_id, stamp)
        m.ns = 'ruler_labels'
        m.id = i
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.scale.z = font_scale
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        if axis == 'x':
            m.pose.position.x = val
            m.pose.position.y = offset
        else:
            m.pose.position.x = offset
            m.pose.position.y = val
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.text = f'{val:.0f}'
        marr.markers.append(m)

        # ---- 刻度线 ----
        tick = Marker()
        tick.header = create_header(frame_id, stamp)
        tick.ns = 'ruler_ticks'
        tick.id = i
        tick.type = Marker.LINE_STRIP
        tick.action = Marker.ADD
        tick.scale.x = 0.05
        tick.color.r = color[0]
        tick.color.g = color[1]
        tick.color.b = color[2]
        tick.color.a = 0.6
        tick.pose.orientation.w = 1.0

        p1 = Point()
        p2 = Point()
        if axis == 'x':
            p1.x = val; p1.y = offset - 1.0; p1.z = 0.0
            p2.x = val; p2.y = offset + 1.0; p2.z = 0.0
        else:
            p1.x = offset - 1.0; p1.y = val; p1.z = 0.0
            p2.x = offset + 1.0; p2.y = val; p2.z = 0.0
        tick.points = [p1, p2]
        marr.markers.append(tick)

    # ---- 坐标轴主线 ----
    axis_line = Marker()
    axis_line.header = create_header(frame_id, stamp)
    axis_line.ns = 'ruler_axis'
    axis_line.id = 0
    axis_line.type = Marker.LINE_STRIP
    axis_line.action = Marker.ADD
    axis_line.scale.x = 0.08
    axis_line.color.r = color[0]
    axis_line.color.g = color[1]
    axis_line.color.b = color[2]
    axis_line.color.a = 0.8
    axis_line.pose.orientation.w = 1.0

    p_start = Point()
    p_end = Point()
    if axis == 'x':
        p_start.x = 0.0;    p_start.y = offset; p_start.z = 0.0
        p_end.x   = length; p_end.y   = offset; p_end.z   = 0.0
    else:
        p_start.x = offset; p_start.y = 0.0;    p_start.z = 0.0
        p_end.x   = offset; p_end.y   = length; p_end.z   = 0.0
    axis_line.points = [p_start, p_end]
    marr.markers.append(axis_line)

    return marr


# ============================================================================
# ROS2 节点
# ============================================================================

class RvizRulerNode(Node):
    """
    RViz 标尺节点 —— 发布参考坐标系标记

    发布话题：
      /ft/ruler   MarkerArray
    """

    def __init__(self):
        super().__init__('rviz_ruler')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('ruler_axis',     RULER_AXIS)
        self.declare_parameter('ruler_offset',   RULER_OFFSET)
        self.declare_parameter('ruler_interval', RULER_INTERVAL)
        self.declare_parameter('ruler_length',   RULER_LENGTH)
        self.declare_parameter('ruler_font',     RULER_FONT)
        self.declare_parameter('ruler_color',    RULER_COLOR)
        self.declare_parameter('publish_hz',     PUBLISH_HZ)
        self.declare_parameter('fixed_frame',    FIXED_FRAME)

        self.ruler_axis     = self.get_parameter('ruler_axis').value
        self.ruler_offset   = float(self.get_parameter('ruler_offset').value)
        self.ruler_interval = float(self.get_parameter('ruler_interval').value)
        self.ruler_length   = float(self.get_parameter('ruler_length').value)
        self.ruler_font     = float(self.get_parameter('ruler_font').value)
        self.ruler_color    = [
            float(v) for v in self.get_parameter('ruler_color').value]
        self.publish_hz     = float(self.get_parameter('publish_hz').value)
        self.fixed_frame    = self.get_parameter('fixed_frame').value

        # ---------- 发布者 ----------
        self.pub_ruler = self.create_publisher(MarkerArray, '/ft/ruler', 10)

        # ---------- 定时器 ----------
        period = 1.0 / self.publish_hz
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Rviz_Ruler 启动: '
            f'axis={self.ruler_axis}, offset={self.ruler_offset}m, '
            f'interval={self.ruler_interval}m, length={self.ruler_length}m')

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        """定期发布标尺标记"""
        stamp = self.get_clock().now().to_msg()
        ruler_arr = create_ruler_markers(
            self.ruler_axis, self.ruler_offset, self.ruler_interval,
            self.ruler_length, self.ruler_font, self.ruler_color,
            self.fixed_frame, stamp)
        self.pub_ruler.publish(ruler_arr)
        self.get_logger().debug(
            f'发布标尺: {len(ruler_arr.markers)} 个标记')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Rviz_Ruler 已停止')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

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
