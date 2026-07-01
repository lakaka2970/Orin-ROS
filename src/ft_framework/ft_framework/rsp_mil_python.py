#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达信号处理节点 —— Python/MIL 实现 (RSP MIL Python)
================================================================================
融合 ADC 数据和车辆数据，执行雷达信号处理（模拟），
输出检测目标列表 DetList。

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

# ---------- 模拟检测参数 ----------
SIM_NUM_TARGETS = 30          # 每帧模拟检测目标数
SIM_RANGE_MAX   = 300.0       # 最大探测距离 (m)
SIM_RANGE_MIN   = 30.0        # 最小探测距离 (m)
SIM_AZ_RANGE    = 45.0        # 方位角范围 (±°)

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import math
import numpy as np

import rclpy
from rclpy.node import Node

from ft_radar_msgs.msg import AdcRawData, DetList, DetPoint, EgoMotion
from ft_framework.common import filter_det_points


# ============================================================================
# ROS2 节点
# ============================================================================

class RspMilPythonNode(Node):
    """
    Python 雷达信号处理节点

    话题:
      订阅: /adc/raw_data       (AdcRawData)
            /vehicle/ego_motion (EgoMotion)
      发布: /processing/radar/det_list (DetList)

    功能说明:
      - 接收 ADC 原始数据和车辆数据
      - 执行 SNR 滤波和速度补偿（模拟）
      - 输出 14 字段 DetPoint
      - 通过 rsp_mode 参数控制: 仅 python/both/both_compare 模式时运行
    """

    def __init__(self):
        super().__init__('rsp_mil_python')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('processing_fps', PROCESSING_FPS)
        self.declare_parameter('snr_threshold', SNR_THRESHOLD)
        self.declare_parameter('velocity_scale', VELOCITY_SCALE)
        self.declare_parameter('rsp_mode', 'cuda')       # 外部通过 node 参数传入
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.processing_fps = float(self.get_parameter('processing_fps').value)
        self.snr_threshold  = float(self.get_parameter('snr_threshold').value)
        self.velocity_scale = float(self.get_parameter('velocity_scale').value)
        self.rsp_mode       = self.get_parameter('rsp_mode').value
        self.fixed_frame    = self.get_parameter('fixed_frame').value

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
        """
        模拟雷达信号处理 pipeline:
          1. 从 ADC 数据提取时间戳
          2. 获取车辆速度用于补偿
          3. 生成模拟检测目标（SNR 滤波 + 速度补偿）
          4. 填充 DetPoint 14 字段
          5. 发布 DetList
        """
        if not self._pub_enabled or self._latest_adc is None:
            return

        self.frame_count += 1
        # 透传 ADC 时间戳，不得重新生成（全局时间戳规则）
        adc_stamp = self._latest_adc.header.stamp

        # 获取车辆速度
        ego_vx = 0.0
        if self._latest_ego is not None and not self._latest_ego.is_default:
            ego_vx = self._latest_ego.vx

        # ---- 模拟检测处理（替换为真实 RSP 算法） ----
        n = SIM_NUM_TARGETS
        ranges   = np.random.uniform(SIM_RANGE_MIN, SIM_RANGE_MAX, n)
        azimuths = np.random.uniform(
            -math.radians(SIM_AZ_RANGE), math.radians(SIM_AZ_RANGE), n)
        elevations = np.random.uniform(-math.radians(5.0), math.radians(5.0), n)

        # 球→笛卡尔（车辆系）
        x = ranges * np.cos(elevations) * np.cos(azimuths)
        y = ranges * np.cos(elevations) * np.sin(azimuths)
        z = ranges * np.sin(elevations)

        # SNR 过滤模拟
        snrs = 20.0 * np.log10(SIM_RANGE_MAX / (ranges + 1e-3))
        snrs = np.clip(snrs, 0, 60)
        mask = snrs >= self.snr_threshold
        valid_idx = np.where(mask)[0]

        # 速度补偿
        distances = ranges * 1.0
        dopplers = self.velocity_scale * (distances * 0.01 - ego_vx)

        # ---- 构造 DetList (v2 41 字段) ----
        det_list = DetList()
        det_list.header.stamp = adc_stamp          # 1. u32TimeStamp: 透传 ADC 原始时间戳（微秒）
        det_list.header.frame_id = self.fixed_frame
        det_list.frame_id = self.frame_count        # 2. u16FrameID: 雷达帧序号

        for i in valid_idx:
            det = DetPoint()
            det.idx = 128                                     # 有效值
            # -- 空间位置（车辆坐标系） --
            det.x = float(x[i])                     # 4.  f32XPos
            det.y = float(y[i])                     # 5.  f32YPos
            det.z = float(z[i])                     # 6.  f32ZPos

            # -- 雷达原视测量 --
            det.rad_vel_abs = float(dopplers[i])    # 7.  f32RadVelAbs  绝对径向速度
            det.range = float(ranges[i])            # 8.  f32Range
            det.speed = float(abs(dopplers[i]))     # 9.  f32Speed     合速度（取径向速度幅值）
            det.azimuth_ang = float(azimuths[i])    # 10. f32AzimuthAng
            det.ele_ang = float(elevations[i])      # 11. f32EleAng

            # -- 信号特征 --
            det.snr_db = float(snrs[i])             # 12. f32SNRdB
            det.rcs_db = -20.0 + np.random.uniform(-10, 10)  # 13. f32RcsdB  模拟 RCS
            det.power_db = float(snrs[i]) - 10.0    # 14. f32PowerdB 回波功率（SNR 近似换算）

            # -- 检测标志位（模拟/默认值） --
            det.strategy_flag = 0                   # 15. u32StrategyFlag
            det.obj_same_rv = 0                     # 16. u32ObjSameRV
            det.obj_quality = 0                     # 17. u32ObjQuality
            det.obj_track_flag = 0                  # 18. u32ObjTrackFlag
            det.ele_confident = 0                   # 19. u32EleConfident
            det.predict_det_flag = 0                # 20. u32PredictDetflag

            # -- DOA 与角度关联 --
            det.doa_method = 0                      # 25. u32DOAMethod
            det.asso_angle_filter_id = 0            # 26. u32AssoAngleFilterId

            # -- RD 索引（模拟值） --
            det.rd_cell_idx = int(128 * i)          # 27. u16RdCellIdx     RD 单元总索引
            det.range_idx = int(ranges[i] / 0.5)    # 28. u16RangeIdx      距离维索引（0.5m 分辨率）
            det.doppler_idx = 128                   # 29. u16DopplerIdx    多普勒速度维索引（有效值）
            det.azimuth_idx = 0                     # 30. u8AzimuthIdx
            det.elevation_idx = 0                   # 31. u8ElevationIdx

            # -- 峰值与 SNR --
            det.peak_val = int(np.clip(snrs[i] * 100, 0, 65535))  # 32. u16PeakVal
            det.sin_azim_snr_lin = 0                # 33. u16SinAzimSNRLin
            det.sin_elev_snr_lin = 0                # 34. u16SinElevSNRLin

            # -- 速度解模糊 --
            det.vel_amb_fac = 0                     # 35. s8VelAmbFac     速度解模糊因子

            # -- 枚举状态 --
            det.det_ambig_state = 0                 # 36. eDetAmbigState  目标速度模糊状态
            det.det_motion_pat = 0                  # 37. eDetMotionPat   目标运动模式（0=静止, 1=运动）

            # -- 置信度与帧间标志 --
            det.det_conf = int(np.random.randint(30, 255))  # 38. u8DetConf     检测点基础置信度
            det.inter_frame_flag = 0                # 39. u8InterFrameFlag
            det.asso_trk_num = 0                    # 40. u8AssoTrkNum    关联跟踪目标数量
            det.chanl_phase_max = 0                 # 41. u8ChanlPhaseMax 天线通道最大相位差

            det_list.points.append(det)

        # 3. u16DetObjNum: 当前帧检测到的目标总点数
        det_list.det_obj_num = len(det_list.points)

        # ---- 应用过滤规则（适配 v2 字段名） ----
        filtered, fstats = filter_det_points(det_list.points)
        det_list.points = filtered
        det_list.det_obj_num = len(filtered)        # 更新为过滤后的目标数

        self.pub_det.publish(det_list)
        self.get_logger().info(
            f'[RSP-PY] 帧 #{self.frame_count}: '
            f'{fstats["total"]} 候选 → {len(filtered)} 有效 '
            f'(过滤: ROI={fstats["roi"]} 高度={fstats["height"]} '
            f'RCS={fstats["rcs"]} 存在概率={fstats["exist_prob"]} '
            f'SNA={fstats["sna"]} 模糊概率={fstats["ambgt_prob"]}, '
            f'SNR>{self.snr_threshold}dB, ego_vx={ego_vx:.1f}m/s)')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info(f'RSP MIL Python 已停止（共处理 {self.frame_count} 帧）')
        super().destroy_node()


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
