#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达信号处理节点 —— CUDA 加速实现 (RSP Cuda)
================================================================================
融合 ADC 数据和车辆数据，以 CUDA GPU 加速方式执行雷达信号处理。

当前状态: 使用完整 Python/MIL 信号处理管线 (与 rsp_mil_python 一致)。
          GPU 加速版本待移植: CuPy/Numba CUDA 实现 FFT + CFAR kernel。

规格:
  - 处理帧率: 10 Hz
  - SNR 阈值: 比 Python 版更低（GPU 更高灵敏度）
  - 输出字段: DetPoint 41 字段与 FT_radar_dataset_requirement 完全对齐
  - 启动模式: 通过 rsp_mode 参数控制发布话题

信号处理流水线:
  1. ADC reshape: 32 MiB byte buffer → (1024, 8, 2048) float32
  2. Range-FFT: 2048-pt FFT (Hann window) → 距离谱
  3. TDM 分离: TX0 [0:512], TX1 [512:1024]
  4. Doppler-FFT: 512-pt FFT (Hann window) → 距离-多普勒谱
  5. 2D CFAR: 两级 CA-CFAR 恒虚警检测
  6. Angle FFT: 8-RX 波束形成 → 方位角估计
  7. DetPoint: 41 字段填充 + 球坐标 → 车体系 Cartesian

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
SNR_THRESHOLD   = 8.0         # SNR 阈值 (dB) — 比 Python 版更灵敏
VELOCITY_SCALE  = 0.5         # 速度估算缩放因子

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import time

import numpy as np
import torch
import rclpy
from rclpy.node import Node

from ft_radar_msgs.msg import AdcRawData, DetList, DetPoint, EgoMotion, RnNciData
from ft_framework.common import filter_det_points
from ft_framework.rsp_processor import create_processor
from ft_framework.signal_process.config import RadarConfig
from ft_framework.signal_process.preprocessing import radar_signal_process_final
from ft_framework.signal_process.doppler import doppler_processing_gpu
from ft_framework.signal_process.peak_detection import peak_search_gpu
from ft_framework.signal_process.arraySim import RadarArrayInitializer
from ft_framework.signal_process.doa_proc import doa_main_ultra_separated, doa_main_batch, doa_env


# ============================================================================
# ROS2 节点
# ============================================================================

class RspCudaNode(Node):
    """CUDA 雷达信号处理节点。

    (当前使用完整 Python/MIL 管线，GPU 内核待移植。)
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

        # ---------- 雷达参数与阵列初始化 ----------
        self.cfg = RadarConfig()
        self.array_env = RadarArrayInitializer()

        # 初始化 DOA 环境（预先准备 FFT 索引映射，避免每帧重复计算）
        if not doa_env.is_initialized:
            doa_env.prepare_mapping_indices(
                self.array_env.Array_Azi, self.array_env.Array_Ele)
            # 缓存子阵选择索引 (GPU tensors), 供批量 DOA 使用
            doa_env.cache_selection_indices(
                self.array_env.AziIdx_Select_gpu, self.array_env.EleIdx_Select_gpu)

        self.get_logger().info(
            f'雷达参数初始化完成: '
            f'n_samples={self.cfg.n_samples}, '
            f'n_chirps={self.cfg.n_chirps}, '
            f'n_rx={self.cfg.n_rx}, '
            f'range_res={self.cfg.range_resolution:.3f}m, '
            f'doppler_res={self.cfg.doppler_resolution:.3f}m/s')

        # 保留 RSP 处理器作为备用
        rsp_config = {
            'snr_threshold':      self.snr_threshold,
            'velocity_scale':     self.velocity_scale,
            'processing_fps':     self.processing_fps,
        }
        self._processor = create_processor(rsp_config)

        # ---------- 数据缓存 ----------
        self._latest_adc = None
        self._latest_ego = None

        # ---------- 订阅 ----------
        _qos = rclpy.qos.QoSProfile(depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            AdcRawData, '/adc/raw_data', self._on_adc, _qos)
        self.create_subscription(
            EgoMotion, '/vehicle/ego_motion', self._on_ego, _qos)

        # ---------- 发布（按模式条件切换话题） ----------
        self._pub_enabled = self.rsp_mode in ('cuda', 'both', 'both_compare')
        if self._pub_enabled:
            # 单路 CUDA 模式 → 主话题; 双路模式 → CUDA 专属话题
            if self.rsp_mode == 'cuda':
                topic = '/processing/radar/det_list'
                nci_topic = '/processing/radar/rn_nci_data'
            else:
                topic = '/processing/radar/det_list_cuda'
                nci_topic = '/processing/radar/rn_nci_data_cuda'
            self.pub_det = self.create_publisher(DetList, topic, 10)
            self.pub_rn_nci = self.create_publisher(RnNciData, nci_topic, 10)
            self.get_logger().info(f'RSP Cuda 发布: {topic}, {nci_topic}')
        else:
            self.get_logger().info(
                f'RSP Cuda 已禁用 (rsp_mode={self.rsp_mode})')

        # ---------- 定时器 ----------
        self.timer = self.create_timer(1.0 / self.processing_fps, self._on_process)
        self.frame_count = 0

        self.get_logger().info(
            f'RSP Cuda 启动: {self.processing_fps:.0f} Hz, '
            f'SNR={self.snr_threshold} dB')

    # ------------------------------------------------------------------
    # 数据回调
    # ------------------------------------------------------------------

    def _on_adc(self, msg: AdcRawData):
        self._latest_adc = msg

    def _on_ego(self, msg: EgoMotion):
        self._latest_ego = msg

    # ------------------------------------------------------------------
    # ADC 数据格式转换
    # ------------------------------------------------------------------

    def _adc_bytes_to_raw_data(self, adc_bytes: bytes) -> np.ndarray:
        """将 ROS ADC 字节流转换为 radar_signal_process_final 需要的格式.

        ADC 消息包含 ctrx0 + ctrx1 原始 int16 数据拼接:
          - 每半集: 4 RF × Ns samples × 2 I/Q × Nc chirps (Fortran order)
          - I/Q 分离 → 8 虚拟通道/CTRX
          - 两片 CTRX → 16 通道

        与 readRawBinCasc 的解析逻辑一致, 但数据源来自 ROS 消息而非文件.

        Returns:
            np.ndarray: 形状 (n_chirps, n_rx, n_samples) = (512, 16, 2048), dtype float64
        """
        data = np.frombuffer(adc_bytes, dtype=np.int16)
        half_elems = len(data) // 2

        Ns = self.cfg.n_samples   # 2048
        Nc = self.cfg.n_chirps    # 512

        raw_channels = []
        for half_idx in range(2):
            ctrx_data = data[half_idx * half_elems : (half_idx + 1) * half_elems]
            # reshape (4, Ns, 2, Nc) Fortran order — 与 MATLAB reshape(data, [4,Ns,2,Nc]) 一致
            ctrx_tmp = ctrx_data.reshape((4, Ns, 2, Nc), order='F')  # (4, Ns, 2, Nc)
            I_comp = ctrx_tmp[:, :, 0, :].copy()  # (4, Ns, Nc) — I 分量 → ch0~3
            Q_comp = ctrx_tmp[:, :, 1, :].copy()  # (4, Ns, Nc) — Q 分量 → ch4~7
            ch = np.concatenate([I_comp, Q_comp], axis=0)  # (8, Ns, Nc)
            raw_channels.append(ch)

        # 合并 → (16, Ns, Nc) → transpose → (Nc, 16, Ns)
        raw = np.concatenate(raw_channels, axis=0)  # (16, 2048, 512)
        raw = np.transpose(raw, (2, 0, 1))           # (512, 16, 2048)
        return raw.astype(np.float64, copy=False)

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

        # ---- 核心 RSP 处理: 完整流水线 (GPU 加速版) ----
        try:
            # 1. ADC 字节 → (512, 16, 2048) 原始数据
            raw_data = self._adc_bytes_to_raw_data(adc_bytes)

            # 2. 预处理 + Range-FFT (GPU)
            cube, dc_est, _ = radar_signal_process_final(
                raw_data, self.cfg.n_samples, self.cfg.n_rx,
                self.cfg.n_chirps, self.cfg.threshold_scale)

            # 3. Doppler-FFT + 非相干积累 (GPU)
            rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci = \
                doppler_processing_gpu(
                    cube, self.cfg.n_rx, self.cfg.n_chirps, cube.shape[2],
                    self.cfg.tx_ddma_idx, self.cfg.n_subbands, self.cfg.noise_est_ratio)

            # 4. 峰值检测 (GPU)
            peaks = peak_search_gpu(
                rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise_est,
                self.cfg.tx_ddma_idx, cube.shape[2], self.cfg.n_chirps, self.cfg.n_subbands,
                self.cfg.ps_scale, self.cfg.max_peaks_per_rb, self.cfg.max_total_peaks)

            # 5. DOA 估计 + 点云生成 (GPU DOA + CPU 构造)
            range_res = self.cfg.range_resolution
            doppler_res = self.cfg.doppler_resolution
            ambgt = self.cfg.ambgt
            doa_threshold_db = 28.0

            # 5a. 批量 DOA: 堆叠所有通道 → 一次并行 FFT (N, 256)
            channel_batch = torch.stack([p['channel_gpu'] for p in peaks])  # (N, 256)
            all_azi, all_ele, azi_snr, ele_snr = doa_main_batch(channel_batch, doa_threshold_db)

            # 5b. 点云生成 (CPU — 结果展开 + 坐标变换)
            det_points = []
            for i, peak in enumerate(peaks):
                azi_results = all_azi[i]
                ele_results = all_ele[i]

                if len(azi_results) == 0 or len(ele_results) == 0:
                    continue

                rb = peak['rb']
                db = peak['db']
                rng = rb * range_res
                vel = (db - rb*4) * doppler_res
                pow_linear = peak['f32PeakPowVchNci_Q7dB']
                snr_db_val = (20.0 * np.log10(pow_linear / peak['noise'])
                              if peak['noise'] > 0 else 0.0)
                rcs_db_val = 20.0 * np.log10(pow_linear)

                # 筛选有效目标 (flag==1), 按能量降序
                valid_azi = [t for t in azi_results if t[0] == 1]
                valid_ele = [t for t in ele_results if t[0] == 1]
                valid_azi.sort(key=lambda t: t[2], reverse=True)
                valid_ele.sort(key=lambda t: t[2], reverse=True)

                if len(valid_azi) == 2 and len(valid_ele) == 2:
                    pairs = [(valid_azi[0], valid_ele[0]),
                             (valid_azi[1], valid_ele[1])]
                else:
                    pairs = [(a, e) for a in valid_azi for e in valid_ele]

                n_pairs = len(pairs)
                for p_idx, (a_target, e_target) in enumerate(pairs):
                    azi_deg = a_target[4]
                    azi_rad = np.deg2rad(azi_deg)
                    ele_deg = e_target[4]
                    ele_rad = np.deg2rad(ele_deg)

                    x = rng * np.cos(ele_rad) * np.cos(azi_rad)
                    y = rng * np.cos(ele_rad) * np.sin(azi_rad)
                    z = rng * np.sin(ele_rad)

                    point = {
                        'x': x, 'y': y, 'z': z,
                        'range': rng,
                        'azimuth': azi_rad,
                        'elevation': ele_rad,
                        'rcs': rcs_db_val,
                        'snr': snr_db_val,
                        'power_db': rcs_db_val,       # 回波功率 dB = 20*log10(pow_linear)
                        'ambgt': ambgt,
                        'exist_prob': 100,
                        'multi_tgt_prob': 100,
                        'ambgt_prob': 100,
                        'raw_doppler': vel,
                        'doppler_idx': int(db),       # u16DopplerIdx: 实际多普勒 bin
                        'azimuth_idx': int(a_target[1]),   # u8AzimuthIdx:   DOA 方位 bin
                        'elevation_idx': int(e_target[1]), # u8ElevationIdx: DOA 俯仰 bin
                        'obj_same_rv': (p_idx + 1) if n_pairs > 1 else 0,  # i32ObjSameRV: 同RV, 第1个=1, 第2个=2
                        'sin_azim_snr_lin': int(azi_snr[i]),   # u16SinAzimSNRLin: 最强/次强比*1000
                        'sin_elev_snr_lin': int(ele_snr[i]),   # u16SinElevSNRLin
                        'rd_cell_idx': 0,
                        'range_idx': rb,
                        'peak_val': int(np.clip(pow_linear, 0, 65535)),
                        'vel_amb_fac': 0,
                        'det_ambig_state': 0,
                        'det_motion_pat': 0,
                    }
                    det_points.append(point)

        except Exception as e:
            self.get_logger().error(f'RSP 处理异常: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return

        # ---- 构造 DetList (v2 41 字段) ----
        det_list = DetList()
        det_list.header.stamp = adc_stamp          # 1. u32TimeStamp: 透传 ADC 原始时间戳（微秒）
        det_list.header.frame_id = self.fixed_frame
        det_list.frame_id = self.frame_count        # 2. u16FrameID: 雷达帧序号

        for dp in det_points:
            pt = DetPoint()

            # -- 空间位置（车辆坐标系） --
            pt.x            = float(dp['x'])              # 4.  f32XPos
            pt.y            = float(dp['y'])              # 5.  f32YPos
            pt.z            = float(dp['z'])              # 6.  f32ZPos

            # -- 雷达原视测量（雷达传感器坐标系） --
            pt.rad_vel_abs  = float(dp['raw_doppler'])    # 7.  f32RadVelAbs  绝对径向速度
            pt.range        = float(dp['range'])          # 8.  f32Range
            pt.speed        = float(abs(dp['raw_doppler']))# 9.  f32Speed     合速度
            pt.azimuth_ang  = float(dp['azimuth'])        # 10. f32AzimuthAng
            pt.ele_ang      = float(dp['elevation'])      # 11. f32EleAng

            # -- 信号特征 --
            pt.snr_db       = float(dp['snr'])            # 12. f32SNRdB
            pt.rcs_db       = float(dp['rcs'])            # 13. f32RcsdB
            pt.power_db     = float(dp['power_db'])        # 14. f32PowerdB  回波功率 dB = 20*log10(amplitude)

            # -- 检测标志位 --
            pt.strategy_flag     = 0               # 15. u32StrategyFlag
            pt.obj_same_rv       = int(dp.get('obj_same_rv', 0))   # 16. u32ObjSameRV
            pt.obj_quality       = 0               # 17. u32ObjQuality
            pt.obj_track_flag    = 0               # 18. u32ObjTrackFlag
            pt.ele_confident     = 0               # 19. u32EleConfident
            pt.predict_det_flag  = 0               # 20. u32PredictDetflag

            # -- DOA 与角度关联 --
            pt.doa_method          = 0             # 25. u32DOAMethod
            pt.asso_angle_filter_id = 0            # 26. u32AssoAngleFilterId

            # -- RD 索引 --
            pt.rd_cell_idx   = int(dp.get('rd_cell_idx', 0))   # 27. u16RdCellIdx
            pt.range_idx     = int(dp.get('range_idx', 0))     # 28. u16RangeIdx
            pt.doppler_idx   = int(dp.get('doppler_idx', 0))       # 29. u16DopplerIdx
            pt.azimuth_idx   = int(dp.get('azimuth_idx', 0))       # 30. u8AzimuthIdx
            pt.elevation_idx = int(dp.get('elevation_idx', 0))     # 31. u8ElevationIdx

            # -- 峰值与 SNR --
            pt.peak_val         = int(dp.get('power_db', 0)*128)       # 32. u16PeakVal
            pt.sin_azim_snr_lin = int(dp.get('sin_azim_snr_lin', 0))  # 33. u16SinAzimSNRLin
            pt.sin_elev_snr_lin = int(dp.get('sin_elev_snr_lin', 0))  # 34. u16SinElevSNRLin

            # -- 速度解模糊 --
            pt.vel_amb_fac = int(dp.get('vel_amb_fac', 0))     # 35. s8VelAmbFac

            # -- 枚举状态 --
            pt.det_ambig_state = int(dp.get('det_ambig_state', 0))  # 36. eDetAmbigState
            pt.det_motion_pat  = int(dp.get('det_motion_pat', 0))   # 37. eDetMotionPat

            # -- 置信度与帧间标志 --
            pt.det_conf         = int(dp['exist_prob'])        # 38. u8DetConf
            pt.inter_frame_flag = 0                            # 39. u8InterFrameFlag
            pt.asso_trk_num     = 0                            # 40. u8AssoTrkNum
            pt.chanl_phase_max  = 0                            # 41. u8ChanlPhaseMax

            det_list.points.append(pt)

        # 3. u16DetObjNum: 当前帧检测到的目标总点数
        det_list.det_obj_num = len(det_list.points)

        # ---- 应用过滤规则（v2 字段名） ----
        filtered, _ = filter_det_points(det_list.points)
        det_list.points = filtered
        det_list.det_obj_num = len(filtered)        # 更新为过滤后的目标数

        self.pub_det.publish(det_list)

        # ---- 发布 RnNci 中间数据 (独立异常保护, 不影响 DetList) ----
        try:
            self._publish_rn_nci(adc_stamp, peaks, rx_nci, vch_nci)
        except Exception as e:
            self.get_logger().error(f'RnNci 发布异常: {e}')

        # ---- 定期日志 ----
        if self.frame_count % 10 == 0:
            t_proc = (time.perf_counter() - t0) * 1000.0
            if det_points:
                ranges = [p['range'] for p in det_points]
                range_str = (f'最近={min(ranges):.1f}m, '
                             f'最远={max(ranges):.1f}m, '
                             f'中位={_median(ranges):.1f}m')
            else:
                range_str = '无检测点'

            self.get_logger().info(
                f'[RSP-CUDA] 帧 #{self.frame_count}: '
                f'{len(det_points)} 检测点, {range_str}, '
                f'耗时={t_proc:.1f}ms')

    # ------------------------------------------------------------------
    # 中间数据发布
    # ------------------------------------------------------------------

    def _publish_rn_nci(self, adc_stamp, peaks: list, rx_nci, vch_nci):
        """构建并发布 RD Cell List + Rx NCI 中间数据 (GPU→CPU, 对齐 spec §7+§8)。"""
        msg = RnNciData()
        msg.header.stamp = adc_stamp
        msg.header.frame_id = self.fixed_frame
        msg.frame_id = self.frame_count
        msg.frame_timestamp_us = (adc_stamp.sec * 1_000_000 + adc_stamp.nanosec // 1000) & 0xFFFFFFFF
        msg.idle_time_idx = 0

        nc = len(peaks)
        msg.num_cells = nc
        msg.rb_list   = [int(p['rb']) for p in peaks]
        msg.db_list   = [int(p['db']) for p in peaks]

        # f32PowRbNci_Q7dB[3]
        msg.pow_rb_nci_0 = [float(p['pow_rb'][0]) for p in peaks]
        msg.pow_rb_nci_1 = [float(p['pow_rb'][1]) for p in peaks]
        msg.pow_rb_nci_2 = [float(p['pow_rb'][2]) for p in peaks]

        # f32PowDbNci_Q7dB[3]
        msg.pow_db_nci_0 = [float(p['pow_db'][0]) for p in peaks]
        msg.pow_db_nci_1 = [float(p['pow_db'][1]) for p in peaks]
        msg.pow_db_nci_2 = [float(p['pow_db'][2]) for p in peaks]

        msg.peak_power_list = [float(p['f32PeakPowVchNci_Q7dB']) for p in peaks]
        msg.noise_power_list = [float(p['noise']) for p in peaks]
        msg.valid_flag_list  = [int(p.get('u8RdValidFlag', 1)) for p in peaks]
        msg.peak_flag_list   = [int(p.get('u8RdPeakFlag', 1)) for p in peaks]

        # sVch[256] — 通道复数 int32 (real, imag 交错)
        ch_bytes_all = b''
        for p in peaks:
            ch = np.asarray(p['sVch'], dtype=np.complex64)  # (256,) complex64
            real_part = np.int32(np.real(ch))
            imag_part = np.int32(np.imag(ch))
            interleaved = np.empty(512, dtype=np.int32)
            interleaved[0::2] = real_part
            interleaved[1::2] = imag_part
            ch_bytes_all += interleaved.tobytes()
        msg.channel_data_bytes = ch_bytes_all

        # Rx NCI (GPU tensor → numpy float32)
        rx_nci_np = rx_nci.cpu().numpy().astype(np.float32)
        msg.rx_nci_rows = rx_nci_np.shape[0]
        msg.rx_nci_cols = rx_nci_np.shape[1]
        msg.rx_nci_data = rx_nci_np.tobytes()

        self.pub_rn_nci.publish(msg)

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info(f'RSP Cuda 已停止（共处理 {self.frame_count} 帧）')
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
