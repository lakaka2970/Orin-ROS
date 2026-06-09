#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达 ADC 数据接收节点 (ADC Rx)
================================================================================
模拟通过 v4l2 接口从雷达硬件采集原始 ADC 数据，发布为自定义 AdcRawData 消息。

规格:
  - 帧率: 15 Hz
  - 数据量: 32 MB/帧 (512 chirps × 16 antennas × 2048 samples × int16)
  - 时间戳: 全局统一，微秒 (μs) 精度，使用 time.monotonic_ns()

话题:
  发布: /adc/raw_data    ft_radar_msgs/AdcRawData

连接关系:
  → R SP MIL Python (sub)
  → R SP Cuda (sub)
  → Logging (sub)

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 采集参数 ----------
ADC_FPS                 = 15        # 帧率 (Hz)
NUM_CHIRPS              = 512       # chirp 数量
NUM_RX_ANTENNAS         = 16        # RX 天线通道数
NUM_SAMPLES_PER_CHIRP   = 2048      # 每个 chirp 的采样点数

# ---------- 模拟参数 ----------
SIM_NOISE_LEVEL         = 100       # 模拟噪声幅度（±）

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import numpy as np

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

from ft_radar_msgs.msg import AdcRawData
from ft_framework.common import monotonic_us_stamp


# ============================================================================
# ROS2 节点
# ============================================================================

class AdcRxNode(Node):
    """
    雷达 ADC 数据接收节点

    发布话题:
      /adc/raw_data    ft_radar_msgs/AdcRawData

    功能说明:
      - 模拟 v4l2 驱动的 ADC 数据采集
      - 在采集第一时间注入全局统一时间戳（微秒精度）
      - 发布 512×16×2048 int16 原始数据（32 MB/帧）
    """

    def __init__(self):
        super().__init__('adc_rx')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('fps', ADC_FPS)
        self.declare_parameter('num_chirps', NUM_CHIRPS)
        self.declare_parameter('num_rx_antennas', NUM_RX_ANTENNAS)
        self.declare_parameter('num_samples_per_chirp', NUM_SAMPLES_PER_CHIRP)
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.fps                = int(self.get_parameter('fps').value)
        self.num_chirps         = int(self.get_parameter('num_chirps').value)
        self.num_rx_antennas    = int(self.get_parameter('num_rx_antennas').value)
        self.num_samples        = int(self.get_parameter('num_samples_per_chirp').value)
        self.fixed_frame        = self.get_parameter('fixed_frame').value

        # ---------- 静态 TF：radar → map ----------
        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        sec, nsec = monotonic_us_stamp()
        tf_msg.header.stamp.sec = sec
        tf_msg.header.stamp.nanosec = nsec
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = self.fixed_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.5       # 雷达安装高度
        tf_msg.transform.rotation.w = 1.0
        self._tf_static.sendTransform(tf_msg)

        # ---------- 发布者 ----------
        self.pub_adc = self.create_publisher(AdcRawData, '/adc/raw_data', 10)

        # ---------- 定时器 ----------
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self._on_timer)
        self.frame_count = 0
        self._total_samples = self.num_chirps * self.num_rx_antennas * self.num_samples

        self.get_logger().info(
            f'ADC Rx 启动: {self.fps} Hz, '
            f'{self.num_chirps} chirps × {self.num_rx_antennas} antennas × '
            f'{self.num_samples} samples, '
            f'每帧 {self._total_samples * 2 / 1024 / 1024:.1f} MB')

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        """
        模拟 ADC 数据采集:
          1. 注入全局时间戳（单调时钟，微秒精度）
          2. 模拟 v4l2 读取原始 int16 数据
          3. 构造 AdcRawData 消息并发布
        """
        self.frame_count += 1

        # ---- 1. 注入时间戳 ----
        sec, nsec = monotonic_us_stamp()

        # ---- 2. 模拟 ADC 数据采集 ----
        # 实际部署时，此处替换为 v4l2 驱动读取:
        #   data_buffer = v4l2_device.read(frame_size_bytes)
        #   int16_array = np.frombuffer(data_buffer, dtype=np.int16)
        data_array = np.random.randint(
            -SIM_NOISE_LEVEL, SIM_NOISE_LEVEL,
            self._total_samples, dtype=np.int16)

        # ---- 3. 构造消息 ----
        msg = AdcRawData()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = self.fixed_frame
        msg.num_chirps = self.num_chirps
        msg.num_rx_antennas = self.num_rx_antennas
        msg.num_samples_per_chirp = self.num_samples
        msg.data = data_array.tolist()

        self.pub_adc.publish(msg)
        self.get_logger().debug(
            f'ADC Rx 帧 #{self.frame_count}: '
            f'timestamp={sec}.{nsec:09d}, '
            f'data_size={len(msg.data)}')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info(f'ADC Rx 已停止（共处理 {self.frame_count} 帧）')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = AdcRxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ADC Rx 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
