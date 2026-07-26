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

信号处理流水线 (GPU 加速):
  1. ADC reshape: 32 MiB byte buffer → (512, 16, 2048) int16 (零拷贝 C-order view)
  2. Range-FFT: GPU 2048-pt rfft (Hann window) → 距离谱
  3. Doppler-FFT: GPU 512-pt FFT (Hann window) → 距离-多普勒谱
  4. 2D CFAR: GPU 向量化峰值检测
  5. Angle FFT: GPU 批量 DOA 估计
  6. DetPoint: 41 字段填充 + 球坐标 → 车体系 Cartesian

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

import math
import time

import numpy as np
import torch
import rclpy
from rclpy.node import Node

from ft_radar_msgs.msg import AdcFilePath, DetList, DetPoint, EgoMotion
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

        # ---- 天线阵列定义（用户指定）----
        AzmChUse = np.array([0,1,2,8,9,10,11,12,13,14,15,16,17,18,24,25,26,27,28,31,32,33,34,40,
                             42,43,45,46,48,49,50,56,58,59,61,62,64,65,66,72,74,75,77,78,80,81,82,
                             88,90,91,93,94,96,97,98,106,107,110,112,113,114,126,208,216,217,218,219,
                             220,221,224,232,234,235,236,237,248,249,251], dtype=np.int64)
        AzmPosUse = np.array([34,29,24,76,70,62,57,52,48,39,43,38,33,28,80,74,66,61,56,47,30,25,20,
                              72,58,53,44,35,26,21,16,68,54,49,40,31,22,17,12,64,50,45,36,27,18,13,8,
                              60,46,41,32,23,14,9,4,42,37,19,10,5,0,15,63,105,99,91,86,81,77,55,97,83,
                              78,73,69,90,84,71], dtype=np.float32)
        ElvChUse = np.array([43,130,131,132,133,134,135,146,147,148,149,150,151,162,163,164,165,166,167,
                             178,179,180,181,182,183,194,195,196,197,198,199,211,212,213,214,215], dtype=np.int64)
        ElvPosUse = np.array([40,74,66,58,50,34,42,81,73,65,57,41,49,67,59,51,43,27,35,54,46,38,30,
                              14,22,47,39,31,23,7,15,32,24,16,0,8], dtype=np.float32)

        n_azi = len(AzmPosUse)
        n_ele = len(ElvPosUse)
        Array_Azi_np = np.zeros((3, n_azi), dtype=np.float32)
        Array_Azi_np[0, :] = AzmPosUse
        Array_Ele_np = np.zeros((3, n_ele), dtype=np.float32)
        Array_Ele_np[1, :] = ElvPosUse

        # 存储到 self (兼容旧代码, numpy + GPU 双版本)
        self.AziIdx_Select = AzmChUse
        self.EleIdx_Select = ElvChUse
        self.Array_Azi = Array_Azi_np
        self.Array_Ele = Array_Ele_np
        self.AziIdx_Select_gpu = torch.from_numpy(AzmChUse).to(torch.device('cuda'))
        self.EleIdx_Select_gpu = torch.from_numpy(ElvChUse).to(torch.device('cuda'))
        self.Array_Azi_gpu = torch.from_numpy(Array_Azi_np).to(torch.device('cuda'))
        self.Array_Ele_gpu = torch.from_numpy(Array_Ele_np).to(torch.device('cuda'))

        # 初始化 DOA 环境（预先准备 FFT 索引映射，避免每帧重复计算）
        if not doa_env.is_initialized:
            doa_env.prepare_mapping_indices(self.Array_Azi, self.Array_Ele)
            # 缓存子阵选择索引 (GPU tensors), 供批量 DOA 使用
            doa_env.cache_selection_indices(self.AziIdx_Select_gpu, self.EleIdx_Select_gpu)

        self.get_logger().info(
            f'雷达参数初始化完成: '
            f'n_samples={self.cfg.n_samples}, '
            f'n_chirps={self.cfg.n_chirps}, '
            f'n_rx={self.cfg.n_rx}, '
            f'range_res={self.cfg.range_resolution:.3f}m, '
            f'doppler_res={self.cfg.doppler_resolution:.3f}m/s')

        # ---------- 固定内存 (Pinned Memory) 预分配: GPU 异步 DMA 传输 ----------
        # 原理: 普通分页内存 → GPU 需要驱动先拷贝到 pinned buffer → 再 DMA
        #       固定内存 → GPU 直接 DMA, 且 non_blocking=True 真正异步
        # ADC 原始数据: (n_chirps, n_rx, n_samples) = (512, 16, 2048) int16 = 32 MiB
        self._pinned_adc = torch.empty(
            (self.cfg.n_chirps, self.cfg.n_rx, self.cfg.n_samples),
            dtype=torch.int16, pin_memory=True)
        # VCH NCI: (n_range_bins, n_chirps) = (1025, 512) float32 ≈ 2 MiB
        self._pinned_vch = torch.empty(
            (self.cfg.n_samples // 2 + 1, self.cfg.n_chirps),
            dtype=torch.float32, pin_memory=True)
        self.get_logger().info(
            f'固定内存已分配: ADC={self._pinned_adc.element_size() * self._pinned_adc.numel() // 1024 // 1024} MiB, '
            f'VCH={self._pinned_vch.element_size() * self._pinned_vch.numel() // 1024} KiB')

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
            AdcFilePath, '/adc/file_path', self._on_adc, _qos)
        self.create_subscription(
            EgoMotion, '/vehicle/ego_motion', self._on_ego, _qos)

        # ---------- 发布（按模式条件切换话题） ----------
        self._pub_enabled = self.rsp_mode in ('cuda', 'both', 'both_compare')
        if self._pub_enabled:
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

        # ---------- V2: RSP 处理完成信号 (通知 adc_rx 释放 V4L2 buffer) ----------
        from std_msgs.msg import Bool
        self.pub_processing_complete = self.create_publisher(
            Bool, '/system/processing_complete', 10)

        self.get_logger().info(
            f'RSP Cuda 启动: {self.processing_fps:.0f} Hz, '
            f'SNR={self.snr_threshold} dB')

    # ------------------------------------------------------------------
    # 数据回调
    # ------------------------------------------------------------------

    def _on_adc(self, msg: AdcFilePath):
        """V2: 从文件路径读取 ADC 数据 (替代 32MB DDS 传输)."""
        if not msg.file_ready or not msg.file_path:
            return
        try:
            with open(msg.file_path, 'rb') as f:
                # 跳过 20 字节文件头 (magic + version + timestamp + data_size)
                f.seek(20)
                adc_bytes = f.read()
            if len(adc_bytes) > 0:
                self._latest_adc = adc_bytes
                self._latest_adc_stamp = msg.header.stamp
        except (IOError, OSError) as e:
            self.get_logger().warn(f'ADC 文件读取失败: {msg.file_path}: {e}')

    def _on_ego(self, msg: EgoMotion):
        self._latest_ego = msg

    # ------------------------------------------------------------------
    # ADC 数据格式转换
    # ------------------------------------------------------------------

    def _adc_bytes_to_raw_data(self, adc_bytes: bytes) -> torch.Tensor:
        """将 ROS ADC 字节流转换为 GPU 就绪的固定内存 tensor.

        ADC 消息包含 ctrx0 + ctrx1 原始 int16 数据拼接:
          - 每半集: 4 RF * Ns samples * 2 I/Q * Nc chirps
          - I/Q 分离 -> 8 虚拟通道/CTRX, 两片 CTRX -> 16 通道

        优化 (2026-07-02 v2):
          - 最终结果写入 pin_memory=True 的固定内存 tensor
          - GPU 传输使用 non_blocking=True → 真正异步 DMA (无需驱动隐式拷贝)
          - 消除 CPU→GPU 传输时的隐式 pageable→pinned 拷贝开销 (~15ms)

        Returns:
            torch.Tensor: 形状 (n_chirps, n_rx, n_samples) = (512, 16, 2048), dtype int16,
                          pin_memory=True (CPU 固定内存, 就绪异步 GPU 传输)
        """
        data = np.frombuffer(adc_bytes, dtype=np.int16)
        half_elems = len(data) // 2
        Ns = self.cfg.n_samples   # 2048
        Nc = self.cfg.n_chirps    # 512

        halves = []
        for half_idx in range(2):
            ctrx = data[half_idx * half_elems : (half_idx + 1) * half_elems]
            # C-order reshape: (Nc, 2, Ns, 4) - 零拷贝 view
            ctrx = ctrx.reshape(Nc, 2, Ns, 4)
            ch = np.concatenate([
                ctrx[:, 0, :, :].transpose(2, 1, 0),
                ctrx[:, 1, :, :].transpose(2, 1, 0),
            ], axis=0)  # (8, Ns, Nc)
            halves.append(ch)

        # 合并两半 -> (16, Ns, Nc) -> transpose -> (Nc, 16, Ns)
        raw = np.concatenate(halves, axis=0)
        result = np.ascontiguousarray(raw.transpose(2, 0, 1))
        # 拷贝到固定内存: 后续 GPU 传输可真正异步
        self._pinned_adc.copy_(torch.from_numpy(result))
        return self._pinned_adc

    # ------------------------------------------------------------------
    # 处理回调
    # ------------------------------------------------------------------

    def _on_process(self):
        """雷达信号处理主回调: ADC bytes → RSP → DetList 发布。"""
        if not self._pub_enabled or self._latest_adc is None:
            return

        self.frame_count += 1
        t0 = time.perf_counter()

        # V2: 透传 ADC 硬件时间戳 (从 AdcFilePath 消息)
        adc_stamp = getattr(self, '_latest_adc_stamp', self.get_clock().now().to_msg())
        adc_bytes = self._latest_adc

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

            det_points = []
            n_peaks = len(peaks)

            # 5a. 批量 DOA: 堆叠所有通道 → 一次并行 FFT (N, 256)
            # 修复 (2026-07-02): 空 peaks 时跳过 DOA, torch.stack([]) 会抛 RuntimeError
            if n_peaks > 0:
                channel_batch = torch.stack([p['channel_gpu'] for p in peaks])  # (N, 256)
                all_azi, all_ele, azi_snr, ele_snr = doa_main_batch(channel_batch, doa_threshold_db)

            # 5b. 点云生成 (CPU — 批量预计算 + math 标量加速)
            if n_peaks > 0:
                # 批量预计算: 提取所有 peak 的标量值到 numpy 数组, 避免 per-peak dict 访问
                rbs = np.array([p['rb'] for p in peaks], dtype=np.float64)
                dbs = np.array([p['db'] for p in peaks], dtype=np.float64)
                pows = np.array([p['f32PeakPowVchNci_Q7dB'] for p in peaks], dtype=np.float64)
                noises = np.array([p['noise'] for p in peaks], dtype=np.float64)

                rngs = rbs * range_res
                vels = (dbs - rbs * 4.0) * doppler_res
                snr_vals = np.where(noises > 0, 20.0 * np.log10(pows / noises), 0.0)
                rcs_vals = 20.0 * np.log10(pows)

                for i in range(n_peaks):
                    azi_results = all_azi[i]
                    ele_results = all_ele[i]

                    if len(azi_results) == 0 or len(ele_results) == 0:
                        continue

                    rb_i, db_i = int(rbs[i]), int(dbs[i])
                    rng_i = float(rngs[i])
                    vel_i = float(vels[i])
                    snr_i = float(snr_vals[i])
                    rcs_i = float(rcs_vals[i])
                    pow_i = float(pows[i])

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
                        # math.cos/sin 比 np.cos/np.sin 快 ~5x (标量场景)
                        azi_rad = math.radians(a_target[4])
                        ele_rad = math.radians(e_target[4])
                        cos_ele = math.cos(ele_rad)

                        x = rng_i * cos_ele * math.cos(azi_rad)
                        y = rng_i * cos_ele * math.sin(azi_rad)
                        z = rng_i * math.sin(ele_rad)

                        # ★ 优化: 直接构造 DetPoint, 消除中间 dict 分配
                        #   原来: dict → DetPoint (每点 2 次内存分配 + 30+ 字段拷贝)
                        #   现在: DetPoint 直接赋值 (1 次内存分配)
                        pt = DetPoint()
                        pt.x            = x
                        pt.y            = y
                        pt.z            = z
                        pt.range        = rng_i
                        pt.azimuth_ang  = azi_rad
                        pt.ele_ang      = ele_rad
                        pt.rcs_db       = rcs_i
                        pt.snr_db       = snr_i
                        pt.power_db     = rcs_i
                        pt.rad_vel_abs  = vel_i
                        pt.speed        = abs(vel_i)
                        pt.obj_same_rv  = (p_idx + 1) if n_pairs > 1 else 0
                        pt.sin_azim_snr_lin = int(azi_snr[i])
                        pt.sin_elev_snr_lin = int(ele_snr[i])
                        pt.rd_cell_idx   = 0
                        pt.range_idx     = rb_i
                        pt.doppler_idx   = db_i
                        pt.azimuth_idx   = int(a_target[1])
                        pt.elevation_idx = int(e_target[1])
                        pt.peak_val      = int(rcs_i * 128)  # 与旧 DetPoint 行为一致: power_db * 128
                        pt.det_conf      = 100  # exist_prob
                        # 其余字段保持默认 0
                        det_points.append(pt)

        except Exception as e:
            self.get_logger().error(f'RSP 处理异常: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return

        # ---- 构造 DetList (v2 41 字段) ----
        det_list = DetList()
        det_list.header.stamp = adc_stamp
        det_list.header.frame_id = self.fixed_frame
        det_list.frame_id = self.frame_count
        det_list.points = det_points
        det_list.det_obj_num = len(det_points)

        # ---- 应用过滤规则（v2 字段名） ----
        filtered, _ = filter_det_points(det_list.points)
        det_list.points = filtered
        det_list.det_obj_num = len(filtered)        # 更新为过滤后的目标数

        self.pub_det.publish(det_list)

        # ---- V2: 通知 adc_rx 处理完成, 释放 V4L2 buffer ----
        from std_msgs.msg import Bool
        complete_msg = Bool()
        complete_msg.data = True
        self.pub_processing_complete.publish(complete_msg)

        # ---- 发布 RnNci 中间数据 (独立异常保护, 不影响 DetList) ----
        try:
            self._publish_rn_nci(adc_stamp, peaks, rx_nci, vch_nci)
        except Exception as e:
            self.get_logger().error(f'RnNci 发布异常: {e}')

        # ---- 定期日志 ----
        if self.frame_count % 10 == 0:
            t_proc = (time.perf_counter() - t0) * 1000.0
            if det_points:
                ranges = [p.range for p in det_points]
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
        """构建并发布 RD Cell List + Rx NCI 中间数据 (GPU→CPU, 对齐 spec §7+§8)。

        优化 (2026-07-02 v2):
          - 通道数据 sVch[256] 批量交叠: 一次性 stack → real/imag 分离 → 写入,
            消除 O(N) 次 per-peak numpy 调用 (~80ms → ~5ms)
          - VCH NCI 使用固定内存: GPU→CPU 异步传输 + 消除隐式拷贝
        """
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
        # ★ 批量优化: 一次性 stack → 向量化 real/imag 分离 → 批量交叠
        #   原来: O(N) 次 per-peak numpy 调用 → 现在: O(1) 批量操作
        if nc > 0:
            all_ch = np.array([p['sVch'] for p in peaks], dtype=np.complex64)  # (N, 256)
            real_part = np.int32(np.real(all_ch))                               # (N, 256)
            imag_part = np.int32(np.imag(all_ch))                               # (N, 256)
            interleaved = np.empty((nc, 512), dtype=np.int32)
            interleaved[:, 0::2] = real_part
            interleaved[:, 1::2] = imag_part
            msg.channel_data_bytes = interleaved.tobytes()
        else:
            msg.channel_data_bytes = b''

        # VCH NCI (GPU tensor → 固定内存 → bytes, 异步传输)
        # 使用预分配固定内存消除 GPU→CPU 传输时的隐式拷贝
        vch_rows, vch_cols = vch_nci.shape[0], vch_nci.shape[1]
        self._pinned_vch[:vch_rows, :vch_cols].copy_(vch_nci, non_blocking=True)
        # 同步点: 必须等待传输完成才能 tobytes()
        torch.cuda.synchronize()
        msg.rx_nci_rows = vch_rows
        msg.rx_nci_cols = vch_cols
        msg.rx_nci_data = self._pinned_vch[:vch_rows, :vch_cols].numpy().tobytes()

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
