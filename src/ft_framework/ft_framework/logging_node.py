#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 数据日志记录节点 (Logging)
================================================================================
集中订阅所有传感器数据和检测结果，记录接收统计。
（本节点仅创建订阅和日志输出框架，不做具体文件写入实现）

订阅话题：
  /ft/adc_data       PointCloud2    雷达 ADC 原始数据
  /ft/video_raw      Image          相机视频数据
  /ft/vehicle_data   TwistStamped   车辆动态数据
  /ft/det_list_py    PointCloud2    Python版检测列表
  /ft/det_list_cu    PointCloud2    CUDA版检测列表

预期输出（待后续实现）：
  adc.bin         雷达 ADC 原始数据二进制文件
  video.mp4       相机视频 MP4 文件
  det_list.csv    检测目标列表 CSV 文件
  vdd.csv         车辆动态数据 CSV 文件

连接关系：
  ← ADC Rx (sub)
  ← Camera Rx (sub)
  ← Vehicle Data Rx (sub)
  ← R SP MIL Python (sub)
  ← R SP Cuda (sub)

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 日志参数 ----------
STATUS_LOG_INTERVAL = 5.0    # 状态摘要输出间隔 (s)

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import TwistStamped


# ============================================================================
# ROS2 节点
# ============================================================================

class LoggingNode(Node):
    """
    数据日志记录节点 —— 框架壳，订阅所有数据源并记录统计

    订阅话题：
      /ft/adc_data       PointCloud2
      /ft/video_raw      Image
      /ft/vehicle_data   TwistStamped
      /ft/det_list_py    PointCloud2
      /ft/det_list_cu    PointCloud2
    """

    def __init__(self):
        super().__init__('logging_node')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('status_log_interval', STATUS_LOG_INTERVAL)
        self.status_log_interval = float(
            self.get_parameter('status_log_interval').value)

        # ---------- 接收计数器 ----------
        self._counts = {
            'adc_data':     0,
            'video_raw':    0,
            'vehicle_data': 0,
            'det_list_py':  0,
            'det_list_cu':  0,
        }

        # ---------- 订阅所有上游数据源 ----------
        # （按框架描述中的连接关系覆盖全部 5 个输入）

        self.sub_adc = self.create_subscription(
            PointCloud2, '/ft/adc_data',
            lambda msg: self._on_data('adc_data'), 10)

        self.sub_video = self.create_subscription(
            Image, '/ft/video_raw',
            lambda msg: self._on_data('video_raw'), 10)

        self.sub_vehicle = self.create_subscription(
            TwistStamped, '/ft/vehicle_data',
            lambda msg: self._on_data('vehicle_data'), 10)

        self.sub_det_py = self.create_subscription(
            PointCloud2, '/ft/det_list_py',
            lambda msg: self._on_data('det_list_py'), 10)

        self.sub_det_cu = self.create_subscription(
            PointCloud2, '/ft/det_list_cu',
            lambda msg: self._on_data('det_list_cu'), 10)

        # ---------- 状态定时器 ----------
        self.timer = self.create_timer(
            self.status_log_interval, self._on_status)
        self.start_time = self.get_clock().now()

        # ---------- 启动日志 ----------
        self.get_logger().info(
            'Logging 节点启动 —— '
            '订阅: /ft/adc_data, /ft/video_raw, /ft/vehicle_data, '
            '/ft/det_list_py, /ft/det_list_cu')
        self.get_logger().info(
            '预期输出文件: adc.bin, video.mp4, det_list.csv, vdd.csv '
            '(待后续实现)')

    # ------------------------------------------------------------------
    # 数据回调
    # ------------------------------------------------------------------

    def _on_data(self, source: str):
        """通用数据回调：仅对来源计数"""
        self._counts[source] += 1

    # ------------------------------------------------------------------
    # 状态输出
    # ------------------------------------------------------------------

    def _on_status(self):
        """定期输出接收统计摘要"""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        total = sum(self._counts.values())
        parts = [f'{k}={v}' for k, v in self._counts.items()]
        self.get_logger().info(
            f'[Logging 状态] 运行 {elapsed:.1f}s | '
            f'总消息: {total} | ' + ' | '.join(parts))
        self.get_logger().debug(
            '（文件写入功能待实现: adc.bin, video.mp4, det_list.csv, vdd.csv）')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        # 输出最终统计
        self._on_status()
        self.get_logger().info('Logging 已停止')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Logging 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
