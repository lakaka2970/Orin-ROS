#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达信号处理引擎 (RSP Processor) — 优化混合流水线
================================================================================
混合流水线: Chirp 相干累积 + Range-FFT → 峰值检测 → 选择性 Doppler-FFT → 角度估计

优化策略:
  1. Range-FFT 前沿 chirp 维相干求和 (线性性质): 8192 次 FFT → 16 次 FFT (~500x 加速)
  2. 仅对检测到的 range 峰值做选择性 Doppler-FFT (每帧 ~30 次 × 8 RX × 512-pt)
  3. 角度估计仅对最终的 (range, doppler) 检测点执行
  4. rfft 替代 fft 处理实数输入 (~1.4x 加速)
  5. 始终保持 float32 精度, 减少内存带宽

性能: < 100ms/帧 (Jetson AGX Orin), 满足 10 Hz 实时处理

数据规格:
  输入: 32 MiB ADC 字节缓冲 (1024 chirps × 8 RX × 2048 samples × int16)
  输出: DetPoint 列表 (14 字段)

作者: zhengyuan.liu
日期: 2026.6.30
================================================================================
"""

import numpy as np

# ============================================================================
# 默认物理参数 (77 GHz 汽车雷达典型值)
# ============================================================================

DEFAULT_PARAMS = {
    # -- 数据维度 --
    'num_chirps':           1024,
    'num_tx':               2,
    'num_rx':               8,
    'num_samples':          2048,
    'rx_per_half':          4,
    'chirps_per_tx':        512,

    # -- FFT 尺寸 --
    'range_fft_size':       2048,
    'doppler_fft_size':     512,
    'angle_fft_size':       64,

    # -- 物理参数 (77 GHz) --
    'range_resolution_m':   0.1465,
    'velocity_max_ms':      16.0,
    'lambda_m':             0.003896,
    'rx_spacing_m':         0.00195,

    # -- 检测参数 --
    'min_snr_db':           10.0,
    'min_range_m':          1.0,
    'max_range_m':          150.0,
    'max_detections':       200,
    'peak_prominence_db':   8.0,         # 峰值显著性阈值 (dB)
    'peak_min_distance':    2,           # 峰值最小间距 (bins)

    # -- 速度缩放因子 --
    'velocity_scale':       0.5,
}


# ============================================================================
# RSP 处理引擎
# ============================================================================

class RspProcessor:
    """实时雷达信号处理引擎 (优化混合流水线).

    用法:
        proc = RspProcessor(params)
        det_points = proc.process(adc_bytes)  # → list[dict]
    """

    def __init__(self, params: dict = None):
        p = dict(DEFAULT_PARAMS)
        if params:
            p.update(params)
        self.p = p

        # -- 派生参数 --
        self.chirps_per_tx = p['num_chirps'] // p['num_tx']        # 512
        self.range_bin_to_m = p['range_resolution_m']
        self.max_range_bin = p['range_fft_size'] // 2              # 1024 (rfft 输出)
        self.min_range_bin = max(1, int(p['min_range_m'] / p['range_resolution_m']))
        self.max_range_bin_clip = min(
            self.max_range_bin - 1,
            int(p['max_range_m'] / p['range_resolution_m']))

        # Doppler 分辨率 (m/s per bin)
        self.doppler_bin_to_v = (
            2.0 * p['velocity_max_ms'] / p['doppler_fft_size'])

        # -- 预计算窗函数 --
        self._range_win = np.hanning(p['num_samples']).astype(np.float32)
        self._doppler_win = np.hanning(self.chirps_per_tx).astype(np.float32)

        # -- 角度 FFT 导向矢量 --
        n_angle = p['angle_fft_size']
        self._angle_sin = np.clip(
            (np.arange(n_angle) - n_angle / 2) / (n_angle / 2),
            -1.0, 1.0).astype(np.float32)
        self._angle_rad = np.arcsin(self._angle_sin).astype(np.float32)

    # ------------------------------------------------------------------
    # 主流水线 (优化版)
    # ------------------------------------------------------------------

    def process(self, adc_bytes: bytes):
        """混合快速流水线: 相干累积 Range-FFT → 峰值检测 → 选择性 Doppler → 角度估计.

        步骤:
          1. ADC reshape → (2, 2, 512, 4, 2048)  [TX, half, chirp, RX, sample]
          2. DC 偏移消除
          3. Chirp 相干求和 → (2, 2, 4, 2048)
          4. Range rfft → (2, 8, 1025) complex64
          5. 非相干合并, 峰值检测 → range_bins[]
          6. 对每个 range peak 做选择性 Doppler FFT:
             a. 提取 per-chirp Range-FFT 值 (单 bin DFT)
             b. 512-pt Doppler FFT → (512,) Doppler 谱
             c. Doppler 峰值检测
          7. 对每个 (range, doppler) 检测做角度 FFT
          8. 生成 DetPoint 列表
        """
        # ---- Step 1: 加载并 reshape ADC 数据 ----
        try:
            half0, half1 = self._load_halves(adc_bytes)
        except Exception:
            return []

        # ---- Step 2: DC 偏移消除 (逐 chirp × 逐 RX) ----
        half0 -= half0.mean(axis=2, keepdims=True)
        half1 -= half1.mean(axis=2, keepdims=True)

        # ---- Step 3: TDM 分离 + Chirp 相干求和 ----
        # half0: (1024, 4, 2048) → TX0 [0:512], TX1 [512:1024]
        tx0_h0 = half0[0:self.chirps_per_tx, :, :]    # (512, 4, 2048)
        tx0_h1 = half1[0:self.chirps_per_tx, :, :]    # (512, 4, 2048)
        tx1_h0 = half0[self.chirps_per_tx:, :, :]     # (512, 4, 2048)
        tx1_h1 = half1[self.chirps_per_tx:, :, :]     # (512, 4, 2048)

        # 相干求和 (沿 chirp 维度), 得到 (4, 2048) 每半集
        # 线性性质: sum(FFT(chirp)) = FFT(sum(chirp))
        sum_tx0_h0 = tx0_h0.sum(axis=0)  # (4, 2048)
        sum_tx0_h1 = tx0_h1.sum(axis=0)  # (4, 2048)
        sum_tx1_h0 = tx1_h0.sum(axis=0)  # (4, 2048)
        sum_tx1_h1 = tx1_h1.sum(axis=0)  # (4, 2048)

        # ---- Step 4: Range rfft ----
        # 对每个 (TX, half) 分别做 rfft
        range_tx0_h0 = self._range_rfft(sum_tx0_h0)   # (4, 1025) complex
        range_tx0_h1 = self._range_rfft(sum_tx0_h1)   # (4, 1025)
        range_tx1_h0 = self._range_rfft(sum_tx1_h0)   # (4, 1025)
        range_tx1_h1 = self._range_rfft(sum_tx1_h1)   # (4, 1025)

        # 合并所有 8 RX + 2 TX 的 Range FFT 结果
        # TX0: (8, 1025), TX1: (8, 1025)
        range_tx0 = np.concatenate([range_tx0_h0, range_tx0_h1], axis=0)  # (8, 1025)
        range_tx1 = np.concatenate([range_tx1_h0, range_tx1_h1], axis=0)  # (8, 1025)

        # ---- Step 5: 非相干合并 + 峰值检测 ----
        # 功率谱: |TX0|² + |TX1|², 在 RX 维度求和
        range_power = (np.abs(range_tx0) ** 2 + np.abs(range_tx1) ** 2).sum(axis=0)
        range_log = 10.0 * np.log10(range_power + 1e-10)  # (1025,)

        # 峰值检测
        range_peaks = self._find_range_peaks(range_log)  # list of (range_bin, snr_db)

        if not range_peaks:
            return []

        # ---- Step 6: 批次选择性 Doppler FFT ----
        # 合并 TX0 chirps: (512, 8, 2048) - ctrx0(4RX) + ctrx1(4RX)
        tx0_chirps = np.concatenate([tx0_h0, tx0_h1], axis=1)  # (512, 8, 2048)

        # 提取所有 range peaks 的 per-chirp 复数值 (批次矩阵乘法)
        # chirps_flat: (n_chirps*n_rx, n_samples) = (4096, 2048)
        # kernel: (n_samples, K) for K range bins
        # result: (4096, K) → reshape to (512, 8, K)
        range_bins_arr = np.array([rb for rb, _ in range_peaks], dtype=np.int32)
        doppler_cube = self._extract_range_bins_batch(
            tx0_chirps, range_bins_arr)  # (512, 8, K) complex64

        detections = []
        for peak_idx, (range_bin, snr_range) in enumerate(range_peaks):
            if len(detections) >= self.p['max_detections']:
                break

            # 获取该 range bin 的 per-chirp 复数据
            doppler_in = doppler_cube[:, :, peak_idx]  # (512, 8) complex64

            # Doppler FFT 沿 chirp 维
            doppler_fft = self._doppler_fft_1d(doppler_in)  # (512, 8) complex64

            # 功率谱 (非相干合并 RX)
            doppler_power = np.sum(np.abs(doppler_fft) ** 2, axis=1)  # (512,)
            doppler_log = 10.0 * np.log10(doppler_power + 1e-10)

            # 找 Doppler 峰值
            dop_peaks = self._find_doppler_peaks(doppler_log, snr_range)
            dop_peaks = dop_peaks[:3]

            for doppler_bin, snr_combined in dop_peaks:
                if snr_combined < self.p['min_snr_db']:
                    continue

                # ---- Step 7: 角度估计 ----
                rx_vec = doppler_fft[doppler_bin, :]  # (8,) complex64
                azimuth = self._estimate_azimuth(rx_vec)

                # ---- Step 8: 生成 DetPoint ----
                point = self._build_detpoint(
                    range_bin, doppler_bin, azimuth, snr_combined)
                if point is not None:
                    detections.append(point)

        return detections

    # ------------------------------------------------------------------
    # Step 1: 快速 ADC 加载
    # ------------------------------------------------------------------

    def _load_halves(self, adc_bytes: bytes):
        """将字节缓冲加载为两个半集 (1024, 4, 2048) float32."""
        data = np.frombuffer(adc_bytes, dtype=np.int16).astype(np.float32, copy=False)
        n_total = self.p['num_chirps'] * self.p['num_rx'] * self.p['num_samples']
        half_elems = n_total // 2

        if len(data) < n_total:
            padded = np.zeros(n_total, dtype=np.float32)
            padded[:len(data)] = data
            data = padded

        half0 = data[:half_elems].reshape(
            self.p['num_chirps'], self.p['rx_per_half'], self.p['num_samples'])
        half1 = data[half_elems:2*half_elems].reshape(
            self.p['num_chirps'], self.p['rx_per_half'], self.p['num_samples'])

        return half0.astype(np.float32, copy=False), half1.astype(np.float32, copy=False)

    # ------------------------------------------------------------------
    # Step 4: Range rfft
    # ------------------------------------------------------------------

    def _range_rfft(self, sum_chirps: np.ndarray) -> np.ndarray:
        """对 chirp-求和后的数据 (n_rx, 2048) 做实数 Range FFT."""
        windowed = sum_chirps * self._range_win[np.newaxis, :]
        return np.fft.rfft(windowed, n=self.p['range_fft_size'], axis=1)

    # ------------------------------------------------------------------
    # Step 5: Range 峰值检测
    # ------------------------------------------------------------------

    def _find_range_peaks(self, range_log: np.ndarray):
        """检测 range profile 中的峰值.

        使用简单的局部最大值 + 显著性阈值。
        """
        n_bins = len(range_log)
        min_dist = self.p['peak_min_distance']
        prominence = self.p['peak_prominence_db']

        peaks = []
        # 限制搜索范围
        start = max(self.min_range_bin, min_dist)
        end = min(self.max_range_bin_clip, n_bins - min_dist)

        for r in range(start, end):
            val = range_log[r]
            # 检查是否是局部最大值
            if val <= range_log[r - 1] or val <= range_log[r + 1]:
                continue
            # 检查一定窗口内的唯一性
            left = range_log[max(0, r - min_dist):r]
            right = range_log[r + 1:min(n_bins, r + min_dist + 1)]
            if len(left) > 0 and val <= np.max(left):
                continue
            if len(right) > 0 and val <= np.max(right):
                continue

            # 计算局部显著性 (与局部中值比较)
            bg_start = max(0, r - 20)
            bg_end = min(n_bins, r + 21)
            bg_vals = np.concatenate([
                range_log[bg_start:max(bg_start, r - 4)],
                range_log[min(bg_end, r + 5):bg_end]
            ])
            if len(bg_vals) < 5:
                continue
            bg_median = np.median(bg_vals)
            snr = val - bg_median
            if snr > prominence:
                peaks.append((r, float(snr)))

        # 按 SNR 降序, 取前 N 个
        peaks.sort(key=lambda x: x[1], reverse=True)
        return peaks[:self.p['max_detections']]

    # ------------------------------------------------------------------
    # Step 6: 批次 Range bin 提取
    # ------------------------------------------------------------------

    def _extract_range_bins_batch(self, chirps: np.ndarray,
                                   range_bins: np.ndarray) -> np.ndarray:
        """批次提取多个 range bin 的 per-chirp per-RX 复 DFT 值.

        chirps: (n_chirps, n_rx, n_samples) float32
        range_bins: (K,) int32, K 个 range bin 索引
        返回: (n_chirps, n_rx, K) complex64

        使用矩阵乘法一次计算所有 (chirp, RX) × 所有 range bins:
          result = windowed_2d @ kernel
          其中 windowed_2d: (n_chirps*n_rx, n_samples) float32
              kernel:        (n_samples, K) complex64
        """
        n_chirps, n_rx, n_samples = chirps.shape
        K = len(range_bins)
        M = n_chirps * n_rx
        N = self.p['range_fft_size']

        # 展平 chirps + RX 维度 → (M, n_samples)
        # concatenate 产生的是 C-contiguous 数组, reshape 无需复制
        chirps_flat = chirps.reshape(M, n_samples)

        # 加窗
        windowed = chirps_flat.astype(np.float32) * self._range_win[np.newaxis, :]

        # 构建 kernel: (N, K) complex64
        # kernel[n, k] = exp(-j * 2π * range_bins[k] * n / N)
        n_arr = np.arange(N, dtype=np.float32)
        k_arr = range_bins.astype(np.float32)
        kernel = np.exp(
            -2j * np.pi * np.outer(n_arr, k_arr) / N
        ).astype(np.complex64)

        # 矩阵乘法: (M, N) @ (N, K) → (M, K)
        result = np.dot(windowed, kernel)

        # 重塑为 (n_chirps, n_rx, K)
        return result.reshape(n_chirps, n_rx, K)

    # ------------------------------------------------------------------
    # Step 6b: Doppler FFT (1D)
    # ------------------------------------------------------------------

    def _doppler_fft_1d(self, doppler_in: np.ndarray) -> np.ndarray:
        """沿 chirp 维度做 512-pt Doppler FFT.

        doppler_in: (512, 8) complex64
        返回: (512, 8) complex64
        """
        n_chirps, n_rx = doppler_in.shape
        # Hann 窗沿 chirp 维
        windowed = doppler_in * self._doppler_win[:, np.newaxis]
        return np.fft.fft(windowed, n=self.p['doppler_fft_size'], axis=0)

    # ------------------------------------------------------------------
    # Step 6c: Doppler 峰值检测
    # ------------------------------------------------------------------

    def _find_doppler_peaks(self, doppler_log: np.ndarray,
                            snr_range: float) -> list:
        """检测 Doppler 谱中的峰值."""
        n_bins = len(doppler_log)
        prominence = self.p['peak_prominence_db']
        min_dist = self.p['peak_min_distance']

        peaks = []
        for d in range(min_dist, n_bins - min_dist):
            val = doppler_log[d]
            if val <= doppler_log[d - 1] or val <= doppler_log[d + 1]:
                continue

            # 局部背景
            bg = np.concatenate([
                doppler_log[max(0, d - 10):max(0, d - 3)],
                doppler_log[min(n_bins, d + 4):min(n_bins, d + 11)]
            ])
            if len(bg) < 3:
                continue

            snr_dop = val - np.median(bg)
            if snr_dop > prominence:
                # 综合 SNR (取 range + doppler 的较小值)
                snr_combined = float(min(snr_range, snr_dop))
                peaks.append((d, snr_combined))

        peaks.sort(key=lambda x: x[1], reverse=True)
        return peaks[:5]

    # ------------------------------------------------------------------
    # Step 7: 角度估计
    # ------------------------------------------------------------------

    def _estimate_azimuth(self, rx_vector: np.ndarray) -> float:
        """对 8-RX 复向量做角度 FFT 估计方位角."""
        n_angle = self.p['angle_fft_size']
        angle_fft = np.fft.fft(rx_vector, n=n_angle)
        angle_fft = np.fft.fftshift(angle_fft)
        angle_power = np.abs(angle_fft) ** 2
        peak_bin = int(np.argmax(angle_power))
        return float(self._angle_rad[peak_bin])

    # ------------------------------------------------------------------
    # Step 8: DetPoint 生成
    # ------------------------------------------------------------------

    def _build_detpoint(self, range_bin: int, doppler_bin: int,
                        azimuth: float, snr_db: float) -> dict:
        """从检测参数构建 DetPoint 字典."""
        range_m = range_bin * self.range_bin_to_m

        # 多普勒速度
        half_dop = self.p['doppler_fft_size'] // 2
        if doppler_bin <= half_dop:
            v = doppler_bin / self.p['doppler_fft_size'] * 2.0 * self.p['velocity_max_ms']
        else:
            v = ((doppler_bin - self.p['doppler_fft_size']) /
                 self.p['doppler_fft_size'] * 2.0 * self.p['velocity_max_ms'])
        raw_doppler = v * self.p['velocity_scale']

        # 坐标变换: 球 → Cartesian
        cos_az = np.cos(azimuth)
        sin_az = np.sin(azimuth)

        # RCS 估算
        rcs = np.clip(snr_db + 40.0 * np.log10(max(range_m, 1.0)) - 60.0, -100.0, 50.0)

        # 概率估计 (基于 SNR)
        exist_prob = int(np.clip((snr_db - 5.0) / 30.0 * 255, 0, 255))
        ambgt_prob = int(np.clip(snr_db / 30.0 * 255, 0, 255))

        return {
            'x':              float(range_m * cos_az),
            'y':              float(range_m * sin_az),
            'z':              0.0,
            'range':          float(range_m),
            'azimuth':        float(azimuth),
            'elevation':      0.0,
            'rcs':            float(rcs),
            'snr':            float(snr_db),
            'ambgt':          float(2.0 * self.p['velocity_max_ms']),
            'exist_prob':     exist_prob,
            'multi_tgt_prob': 0,
            'ambgt_prob':     ambgt_prob,
            'raw_doppler':    float(raw_doppler),
            'idx':            0,
        }


# ============================================================================
# 便捷工厂函数
# ============================================================================

def create_processor(yaml_rsp_config: dict = None) -> RspProcessor:
    """从 YAML 配置字典创建 RspProcessor 实例."""
    params = dict(DEFAULT_PARAMS)

    if yaml_rsp_config:
        key_map = {
            'snr_threshold':       'min_snr_db',
            'velocity_scale':      'velocity_scale',
            'processing_fps':      None,
            'range_fft_size':      'range_fft_size',
            'doppler_fft_size':    'doppler_fft_size',
            'angle_fft_size':      'angle_fft_size',
            'range_resolution_m':  'range_resolution_m',
            'velocity_max_ms':     'velocity_max_ms',
            'cfar_threshold_db':   'peak_prominence_db',
            'min_snr_db':          'min_snr_db',
            'min_range_m':         'min_range_m',
            'max_range_m':         'max_range_m',
            'max_detections':      'max_detections',
        }
        for yaml_key, internal_key in key_map.items():
            if internal_key is not None and yaml_key in yaml_rsp_config:
                val = yaml_rsp_config[yaml_key]
                if val is not None and not (isinstance(val, float) and np.isnan(val)):
                    if internal_key == 'peak_prominence_db':
                        # CFAR threshold 映射为 peak prominence
                        params.setdefault(internal_key, float(val))
                    else:
                        params[internal_key] = val

    return RspProcessor(params)
