#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达信号处理节点 —— Python/MIL 实现 (RSP MIL Python)
================================================================================
融合 ADC 数据和车辆数据，执行完整的雷达信号处理流水线。

信号处理流水线:
  1. ADC reshape: 32 MiB byte buffer → (1024, 8, 2048) float32
  2. Range-FFT: 2048-pt FFT (Hann window) → 距离谱
  3. TDM 分离: TX0 [0:512], TX1 [512:1024]
  4. Doppler-FFT: 512-pt FFT (Hann window) → 距离-多普勒谱
  5. 2D CFAR: 两级 CA-CFAR 恒虚警检测
  6. Angle FFT: 8-RX 波束形成 → 方位角估计
  7. DetPoint: 14 字段填充 + 球坐标 → 车体系 Cartesian

规格:
  - 处理帧率: 10 Hz
  - 输出字段: DetPoint 14 字段与 FT_radar_dataset_requirement 完全对齐
  - 启动模式: 通过 rsp_mode 参数控制启停

话题:
  订阅: /adc/raw_data         ft_radar_msgs/AdcRawData
        /vehicle/ego_motion   ft_radar_msgs/EgoMotion
  发布: /processing/radar/det_list  ft_radar_msgs/DetList（仅在 python/both/both_compare 模式）

连接关系:
  ← ADC Rx (sub)
  ← Vehicle Data Rx (sub)
  → Rviz_radar (pub)
  → 3D Object Detection (pub)
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
SNR_THRESHOLD   = 10.0        # 信噪比阈值 (dB)
VELOCITY_SCALE  = 0.5         # 速度估算缩放因子

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import time

import rclpy
from rclpy.node import Node

from ft_radar_msgs.msg import AdcRawData, DetList, DetPoint, EgoMotion
from ft_framework.rsp_processor import create_processor


# ============================================================================
# ROS2 节点
# ============================================================================

class RspMilPythonNode(Node):
    """Python 雷达信号处理节点。

    话题:
      订阅: /adc/raw_data       (AdcRawData)
            /vehicle/ego_motion (EgoMotion)
      发布: /processing/radar/det_list (DetList)
    """

    def __init__(self):
        super().__init__('rsp_mil_python')

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

        # ---------- RSP 处理器 ----------
        rsp_config = {
            'snr_threshold':      self.snr_threshold,
            'velocity_scale':     self.velocity_scale,
            'processing_fps':     self.processing_fps,
        }
        self._processor = create_processor(rsp_config)
        self.get_logger().info(
            f'RSP 处理器初始化完成: '
            f'Range-FFT={self._processor.p["range_fft_size"]}pt, '
            f'Doppler-FFT={self._processor.p["doppler_fft_size"]}pt, '
            f'CFAR={self._processor.p["peak_prominence_db"]}dB')

        # ---------- 数据缓存 ----------
        self._latest_adc     = None
        self._latest_ego     = None

        # ---------- 订阅 ----------
        _qos = rclpy.qos.QoSProfile(depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            AdcRawData, '/adc/raw_data', self._on_adc, _qos)
        self.create_subscription(
            EgoMotion, '/vehicle/ego_motion', self._on_ego, _qos)

        # ---------- 发布（按模式条件） ----------
        self._pub_enabled = self.rsp_mode in ('python', 'both', 'both_compare')
        if self._pub_enabled:
            self.pub_det = self.create_publisher(
                DetList, '/processing/radar/det_list', 10)
            self.get_logger().info(
                f'RSP MIL Python 发布: /processing/radar/det_list')
        else:
            self.get_logger().info(
                f'RSP MIL Python 已禁用 (rsp_mode={self.rsp_mode})')

        # ---------- 定时器 ----------
        self.timer = self.create_timer(1.0 / self.processing_fps, self._on_process)
        self.frame_count = 0

        self.get_logger().info(
            f'RSP MIL Python 启动: {self.processing_fps:.0f} Hz, '
            f'SNR={self.snr_threshold} dB')

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
        """雷达信号处理主回调: ADC bytes → RSP → DetList 发布。"""
        if not self._pub_enabled or self._latest_adc is None:
            return

        self.frame_count += 1
        t0 = time.perf_counter()

        # 透传 ADC 时间戳
        adc_stamp = self._latest_adc.header.stamp
        adc_bytes = self._latest_adc.data

        # ---- 核心 RSP 处理 ----
        try:
            det_points = self._processor.process(adc_bytes)
        except Exception as e:
            self.get_logger().error(f'RSP 处理异常: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return

        t_proc = (time.perf_counter() - t0) * 1000.0

        # ---- 构造 DetList ----
        det_list = DetList()
        det_list.header.stamp = adc_stamp
        det_list.header.frame_id = self.fixed_frame

        for dp in det_points:
            pt = DetPoint()
            pt.x             = dp['x']
            pt.y             = dp['y']
            pt.z             = dp['z']
            pt.range         = dp['range']
            pt.azimuth       = dp['azimuth']
            pt.elevation     = dp['elevation']
            pt.rcs           = dp['rcs']
            pt.snr           = dp['snr']
            pt.ambgt         = dp['ambgt']
            pt.exist_prob    = dp['exist_prob']
            pt.multi_tgt_prob = dp['multi_tgt_prob']
            pt.ambgt_prob    = dp['ambgt_prob']
            pt.raw_doppler   = dp['raw_doppler']
            pt.idx           = dp['idx']
            det_list.points.append(pt)

        self.pub_det.publish(det_list)

        # ---- 定期日志 ----
        if self.frame_count % 10 == 0:
            if det_points:
                ranges = [p['range'] for p in det_points]
                range_str = (f'最近={min(ranges):.1f}m, '
                             f'最远={max(ranges):.1f}m, '
                             f'中位={_median(ranges):.1f}m')
            else:
                range_str = '无检测点'

            self.get_logger().info(
                f'[RSP-PY] 帧 #{self.frame_count}: '
                f'{len(det_points)} 检测点, {range_str}, '
                f'处理耗时={t_proc:.1f}ms')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info(f'RSP MIL Python 已停止（共处理 {self.frame_count} 帧）')
        super().destroy_node()


# ============================================================================
# 辅助函数
# ============================================================================

def _median(arr):
    """中位数 (无 numpy 依赖)。"""
    if not arr:
        return 0.0
    s = sorted(arr)
    n = len(s)
    if n % 2 == 1:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) / 2.0


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = RspMilPythonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('RSP MIL Python 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
