#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达 ADC 数据接收节点 (ADC Rx)
================================================================================
模拟通过 v4l2 接口从雷达硬件采集原始 ADC 数据，发布为自定义 AdcRawData 消息。

规格:
  - 帧率: 15 Hz
  - 数据量: 32 MB/帧 (512 rows × 16 chirps/row × 2048 samples/chirp × int16)
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
ADC_FPS                    = 10        # 帧率 (Hz)
NUM_ROWS                   = 512       # 行数
NUM_CHIRPS_PER_ROW         = 16        # 每行的 chirp 数量
NUM_SAMPLES_PER_CHIRP      = 2048      # 每个 chirp 的采样点数

# ---------- 模拟参数 ----------
SIM_NOISE_LEVEL            = 100       # 模拟噪声幅度（±）
SIM_NOISE_POOL_FACTOR      = 4         # 噪声池倍数 (预生成池 = 帧大小 × 倍数)

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import os
import numpy as np

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

from ft_radar_msgs.msg import AdcRawData
from ft_framework.common import monotonic_us_stamp
from ft_framework.perf_profiler import FrameProfiler


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
        self.declare_parameter('num_rows', NUM_ROWS)
        self.declare_parameter('num_chirps_per_row', NUM_CHIRPS_PER_ROW)
        self.declare_parameter('num_samples_per_chirp', NUM_SAMPLES_PER_CHIRP)
        self.declare_parameter('fixed_frame', FIXED_FRAME)
        self.declare_parameter('profiler_enabled', True)
        self.declare_parameter('profiler_log_interval', 50)
        self.declare_parameter('profiler_report_dir', '')

        self.fps                = int(self.get_parameter('fps').value)
        self.num_rows           = int(self.get_parameter('num_rows').value)
        self.num_chirps_per_row = int(self.get_parameter('num_chirps_per_row').value)
        self.num_samples        = int(self.get_parameter('num_samples_per_chirp').value)
        self.fixed_frame        = self.get_parameter('fixed_frame').value

        # ---------- 性能分析器 (自动接入 _on_timer) ----------
        prof_enabled  = bool(self.get_parameter('profiler_enabled').value)
        prof_interval = int(self.get_parameter('profiler_log_interval').value)
        prof_dir      = self.get_parameter('profiler_report_dir').value or os.getcwd()
        self._prof = FrameProfiler(
            self, log_every_n=prof_interval, enabled=prof_enabled,
            report_to_file=True, report_dir=prof_dir)
        self._prof.wrap_callback(self, '_on_timer')     # 自动接管 tick/tick_end
        if prof_enabled:
            self.get_logger().info(
                f'性能分析器已启用 (每 {prof_interval} 帧报告, '
                f'文件输出: {prof_dir}')

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
        self._total_samples = self.num_rows * self.num_chirps_per_row * self.num_samples

        self.get_logger().info(
            f'ADC Rx 启动: {self.fps} Hz, '
            f'{self.num_rows} rows × {self.num_chirps_per_row} chirps/row × '
            f'{self.num_samples} samples/chirp, '
            f'每帧 {self._total_samples * 2 / 1024 / 1024:.1f} MB')

        # ---------- 预生成模拟噪声池 ----------
        # 原理: np.random.randint(16.7M) 在 ARM 上需 ~82ms，
        #   改为预生成 4x 噪声池 + 每帧随机切片，将 O(16.7M) 降到 O(1)。
        pool_size = self._total_samples * SIM_NOISE_POOL_FACTOR
        self._noise_pool = np.random.randint(
            -SIM_NOISE_LEVEL, SIM_NOISE_LEVEL, pool_size, dtype=np.int16)
        self._pool_max_offset = pool_size - self._total_samples
        self.get_logger().info(
            f'噪声池已预生成: {pool_size / 1e6:.1f}M 采样 '
            f'({pool_size * 2 / 1024 / 1024:.1f} MB)')

        # ---------- 预分配消息对象 (复用避免每帧 malloc) ----------
        self._msg = AdcRawData()
        self._msg.header.frame_id = self.fixed_frame
        self._msg.num_rows = self.num_rows
        self._msg.num_chirps_per_row = self.num_chirps_per_row
        self._msg.num_samples_per_chirp = self.num_samples

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        """
        模拟 ADC 数据采集:
          1. 注入全局时间戳（单调时钟，微秒精度）
          2. 从预生成噪声池中随机切片 (替代实时 randint)
          3. 复用预分配消息对象并发布

        FrameProfiler 通过 wrap_callback 自动接管帧级计时，
        checkpoint 置于每步代码之后，确保名称与测量内容一致。
        """
        self.frame_count += 1

        # ---- 1. 注入时间戳 ----
        sec, nsec = monotonic_us_stamp()
        self._prof.checkpoint('1.timestamp')

        # ---- 2. 模拟 ADC 数据采集 (预生成池随机切片) ----
        # 实际部署时，此处替换为 v4l2 驱动读取:
        #   data_buffer = v4l2_device.read(frame_size_bytes)
        #   int16_array = np.frombuffer(data_buffer, dtype=np.int16)
        offset = np.random.randint(0, self._pool_max_offset + 1)
        data_array = self._noise_pool[offset : offset + self._total_samples]
        self._prof.checkpoint('2.np_random')

        # ---- 3. 更新消息 (复用预分配对象) ----
        self._msg.header.stamp.sec = sec
        self._msg.header.stamp.nanosec = nsec
        # ★ 绕过 setter: 直接写 _data 避免 Foxy 内部 list(bytes) 转换
        self._msg._data = data_array.tobytes()
        self._prof.checkpoint('3.build_msg')

        # ---- 4. 发布 ----
        self.pub_adc.publish(self._msg)
        self._prof.checkpoint('4.publish')

        self.get_logger().debug(
            f'ADC Rx 帧 #{self.frame_count}: '
            f'timestamp={sec}.{nsec:09d}, '
            f'data_size={self._total_samples}')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._prof.finalize()       # 输出最终报告并落盘 JSON
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
