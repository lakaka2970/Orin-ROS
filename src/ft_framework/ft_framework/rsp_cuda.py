#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达信号处理节点 —— CUDA 加速实现 (RSP Cuda)
================================================================================
融合 ADC 数据和车辆数据，以 CUDA GPU 加速方式执行雷达信号处理（模拟）。

规格:
  - 处理帧率: 10 Hz
  - SNR 阈值: 比 Python 版更低（模拟 GPU 更高灵敏度）
  - 输出字段: DetPoint 14 字段与 FT_radar_dataset_requirement 完全对齐
  - 启动模式: 通过 rsp_mode 参数控制发布话题

话题:
  订阅: /adc/raw_data         ft_radar_msgs/AdcRawData
        /vehicle/ego_motion   ft_radar_msgs/EgoMotion
  发布: /processing/radar/det_list      (单路 CUDA 模式)
        /processing/radar/det_list_cuda  (双路 both/both_compare 模式)

连接关系:
  ← ADC Rx (sub)
  ← Vehicle Data Rx (sub)
  → Rviz_radar (pub)
  → Logging (pub)

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 处理参数 ----------
PROCESSING_FPS  = 10.0        # 处理帧率 (Hz)
SNR_THRESHOLD   = 8.0         # SNR 阈值 (dB)，比 Python 版更低（更高灵敏度）
VELOCITY_SCALE  = 0.5         # 速度估算缩放因子

# ---------- 模拟检测参数 ----------
SIM_NUM_TARGETS = 45          # CUDA 版模拟更多目标（高灵敏度）
SIM_RANGE_MAX   = 300.0       # 最大探测距离 (m)
SIM_RANGE_MIN   = 30.0        # 最小探测距离 (m)
SIM_AZ_RANGE    = 45.0        # 方位角范围 (±°)

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import time
import math
import numpy as np

import rclpy
from rclpy.node import Node

from ft_radar_msgs.msg import AdcRawData, DetList, DetPoint, EgoMotion


# ============================================================================
# 时间戳工具函数
# ============================================================================

def monotonic_us_stamp() -> tuple:
    """获取单调时钟的微秒时间戳，返回 (sec, nanosec)"""
    now_ns = time.monotonic_ns()
    sec = int(now_ns // 1_000_000_000)
    nsec = int(now_ns % 1_000_000_000)
    return (sec, nsec)


# ============================================================================
# ROS2 节点
# ============================================================================

class RspCudaNode(Node):
    """
    CUDA 雷达信号处理节点（模拟 GPU 加速版本）

    话题:
      订阅: /adc/raw_data       (AdcRawData)
            /vehicle/ego_motion (EgoMotion)
      发布: /processing/radar/det_list      (单路 CUDA 模式)
             /processing/radar/det_list_cuda  (双路模式)

    功能说明:
      - 接收 ADC 原始数据和车辆数据
      - 比 Python 版更低的 SNR 阈值（模拟 GPU 的高灵敏度）
      - 根据 rsp_mode 切换发布话题以隔离输出
      - 输出 14 字段 DetPoint
    """

    def __init__(self):
        super().__init__('rsp_cuda')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('processing_fps', PROCESSING_FPS)
        self.declare_parameter('snr_threshold', SNR_THRESHOLD)
        self.declare_parameter('velocity_scale', VELOCITY_SCALE)
        self.declare_parameter('rsp_mode', 'cuda')
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.processing_fps = float(self.get_parameter('processing_fps').value)
        self.snr_threshold  = float(self.get_parameter('snr_threshold').value)
        self.velocity_scale = float(self.get_parameter('velocity_scale').value)
        self.rsp_mode       = self.get_parameter('rsp_mode').value
        self.fixed_frame    = self.get_parameter('fixed_frame').value

        # ---------- 数据缓存 ----------
        self._latest_adc = None
        self._latest_ego = None

        # ---------- 订阅 ----------
        self.create_subscription(
            AdcRawData, '/adc/raw_data', self._on_adc, 10)
        self.create_subscription(
            EgoMotion, '/vehicle/ego_motion', self._on_ego, 10)

        # ---------- 发布（按模式条件切换话题） ----------
        self._pub_enabled = self.rsp_mode in ('cuda', 'both', 'both_compare')
        if self._pub_enabled:
            # 单路 CUDA 模式 → 主话题; 双路模式 → CUDA 专属话题
            if self.rsp_mode == 'cuda':
                topic = '/processing/radar/det_list'
            else:
                topic = '/processing/radar/det_list_cuda'
            self.pub_det = self.create_publisher(DetList, topic, 10)
            self.get_logger().info(f'RSP Cuda 发布: {topic}')
        else:
            self.get_logger().info(
                f'RSP Cuda 已禁用 (rsp_mode={self.rsp_mode})')

        # ---------- 定时器 ----------
        self.timer = self.create_timer(1.0 / self.processing_fps, self._on_process)
        self.frame_count = 0

        self.get_logger().info(
            f'RSP Cuda 启动: {self.processing_fps:.0f} Hz, '
            f'SNR={self.snr_threshold} dB [GPU 模拟]')

    # ------------------------------------------------------------------
    # 数据回调
    # ------------------------------------------------------------------

    def _on_adc(self, msg: AdcRawData):
        self._latest_adc = msg

    def _on_ego(self, msg: EgoMotion):
        self._latest_ego = msg

    # ------------------------------------------------------------------
    # 处理回调
    # ------------------------------------------------------------------

    def _on_process(self):
        """
        模拟 CUDA 加速雷达信号处理 pipeline:
          1. 从 ADC 数据提取时间戳
          2. 使用更低 SNR 阈值保留更多弱目标
          3. 生成模拟检测目标
          4. 填充 DetPoint 14 字段
          5. 发布 DetList
        """
        if not self._pub_enabled or self._latest_adc is None:
            return

        self.frame_count += 1
        sec, nsec = monotonic_us_stamp()

        # 获取车辆速度
        ego_vx = 0.0
        if self._latest_ego is not None and not self._latest_ego.is_default:
            ego_vx = self._latest_ego.vx

        # ---- 模拟 CUDA 加速检测 ----
        # 使用更低 SNR 阈值 → 更多弱目标被保留
        n = SIM_NUM_TARGETS
        ranges   = np.random.uniform(SIM_RANGE_MIN, SIM_RANGE_MAX, n)
        azimuths = np.random.uniform(
            -math.radians(SIM_AZ_RANGE), math.radians(SIM_AZ_RANGE), n)
        elevations = np.random.uniform(-math.radians(5.0), math.radians(5.0), n)

        x = ranges * np.cos(elevations) * np.cos(azimuths)
        y = ranges * np.cos(elevations) * np.sin(azimuths)
        z = ranges * np.sin(elevations)

        # SNR 生成（含更多弱目标）
        snrs = 20.0 * np.log10(SIM_RANGE_MAX / (ranges + 1e-3))
        snrs = np.clip(snrs, 0, 60)
        mask = snrs >= self.snr_threshold
        valid_idx = np.where(mask)[0]

        # 速度补偿
        distances = ranges * 1.0
        dopplers = self.velocity_scale * (distances * 0.01 - ego_vx)

        # ---- 构造 DetList ----
        det_list = DetList()
        det_list.header.stamp.sec = sec
        det_list.header.stamp.nanosec = nsec
        det_list.header.frame_id = self.fixed_frame

        for i in valid_idx:
            det = DetPoint()
            det.x = float(x[i])
            det.y = float(y[i])
            det.z = float(z[i])
            det.range = float(ranges[i])
            det.azimuth = float(azimuths[i])
            det.elevation = float(elevations[i])
            det.rcs = -20.0 + np.random.uniform(-10, 10)
            det.snr = float(snrs[i])
            det.ambgt = 21.82
            det.exist_prob = int(np.random.randint(30, 100))
            det.multi_tgt_prob = 100
            det.ambgt_prob = int(np.random.randint(40, 100))
            det.raw_doppler = float(dopplers[i])
            det.idx = 128
            det_list.points.append(det)

        self.pub_det.publish(det_list)
        self.get_logger().info(
            f'[RSP-CUDA] 帧 #{self.frame_count}: '
            f'生成 {len(det_list.points)} 个 DetPoint '
            f'(SNR>{self.snr_threshold}dB, GPU 模式)')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info(f'RSP Cuda 已停止（共处理 {self.frame_count} 帧）')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = RspCudaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('RSP Cuda 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
