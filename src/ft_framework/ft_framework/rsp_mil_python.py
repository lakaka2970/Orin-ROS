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
from ft_framework.common import filter_det_points
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

        # ---- 构造 DetList (v2 41 字段) ----
        det_list = DetList()
        det_list.header.stamp = adc_stamp          # 1. u32TimeStamp: 透传 ADC 原始时间戳（微秒）
        det_list.header.frame_id = self.fixed_frame
        det_list.frame_id = self.frame_count        # 2. u16FrameID: 雷达帧序号

        for dp in det_points:
            pt = DetPoint()

            # -- 空间位置（车辆坐标系） --
            pt.x            = dp['x']              # 4.  f32XPos
            pt.y            = dp['y']              # 5.  f32YPos
            pt.z            = dp['z']              # 6.  f32ZPos

            # -- 雷达原视测量（雷达传感器坐标系） --
            pt.rad_vel_abs  = dp['raw_doppler']    # 7.  f32RadVelAbs  绝对径向速度
            pt.range        = dp['range']          # 8.  f32Range
            pt.speed        = abs(dp['raw_doppler'])# 9.  f32Speed     合速度
            pt.azimuth_ang  = dp['azimuth']        # 10. f32AzimuthAng
            pt.ele_ang      = dp['elevation']      # 11. f32EleAng

            # -- 信号特征 --
            pt.snr_db       = dp['snr']            # 12. f32SNRdB
            pt.rcs_db       = dp['rcs']            # 13. f32RcsdB
            pt.power_db     = dp['snr'] - 10.0     # 14. f32PowerdB  回波功率 (SNR 近似换算)

            # -- 检测标志位 --
            pt.strategy_flag     = 0               # 15. u32StrategyFlag
            pt.obj_same_rv       = 0               # 16. u32ObjSameRV
            pt.obj_quality       = 0               # 17. u32ObjQuality
            pt.obj_track_flag    = 0               # 18. u32ObjTrackFlag
            pt.ele_confident     = 0               # 19. u32EleConfident
            pt.predict_det_flag  = 0               # 20. u32PredictDetflag

            # -- DOA 与角度关联 --
            pt.doa_method          = 0             # 25. u32DOAMethod
            pt.asso_angle_filter_id = 0            # 26. u32AssoAngleFilterId

            # -- RD 索引 --
            pt.rd_cell_idx   = dp.get('rd_cell_idx', 0)   # 27. u16RdCellIdx
            pt.range_idx     = dp.get('range_idx', 0)     # 28. u16RangeIdx
            pt.doppler_idx   = dp['idx']                  # 29. u16DopplerIdx
            pt.azimuth_idx   = 0                          # 30. u8AzimuthIdx
            pt.elevation_idx = 0                          # 31. u8ElevationIdx

            # -- 峰值与 SNR --
            pt.peak_val         = dp.get('peak_val', 0)   # 32. u16PeakVal
            pt.sin_azim_snr_lin = 0                       # 33. u16SinAzimSNRLin
            pt.sin_elev_snr_lin = 0                       # 34. u16SinElevSNRLin

            # -- 速度解模糊 --
            pt.vel_amb_fac = dp.get('vel_amb_fac', 0)     # 35. s8VelAmbFac

            # -- 枚举状态 --
            pt.det_ambig_state = dp.get('det_ambig_state', 0)  # 36. eDetAmbigState
            pt.det_motion_pat  = dp.get('det_motion_pat', 0)   # 37. eDetMotionPat

            # -- 置信度与帧间标志 --
            pt.det_conf         = dp['exist_prob']        # 38. u8DetConf
            pt.inter_frame_flag = 0                       # 39. u8InterFrameFlag
            pt.asso_trk_num     = 0                       # 40. u8AssoTrkNum
            pt.chanl_phase_max  = 0                       # 41. u8ChanlPhaseMax

            det_list.points.append(pt)

        # 3. u16DetObjNum: 当前帧检测到的目标总点数
        det_list.det_obj_num = len(det_list.points)

        # ---- 应用过滤规则（v2 字段名） ----
        filtered, _ = filter_det_points(det_list.points)
        det_list.points = filtered
        det_list.det_obj_num = len(filtered)        # 更新为过滤后的目标数

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
