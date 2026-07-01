"""
多普勒处理模块：对雷达数据立方体执行多普勒 FFT、非相干积累、DDMA 解调，
并提取各子带最大峰值位置。
"""

import torch

DEVICE = torch.device('cuda')
_n_chirps = 512      # Chirp 数量（与 RadarConfig 保持一致）

# 多普勒维汉宁窗（预先加载到 GPU）
win_doppler = torch.hann_window(_n_chirps, periodic=False, dtype=torch.float32, device=DEVICE)[None, :, None]


@torch.inference_mode()
def doppler_processing_gpu(radarcube, n_rx, n_chirps, n_range_bins,
                           tx_ddma_idx, n_subbands, noise_est_ratio=50):
    """
    GPU 加速的多普勒处理流水线。

    参数:
        radarcube: 距离 FFT 后的数据，形状 (n_rx, n_chirps, n_range_bins)
        n_rx: 接收天线数
        n_chirps: Chirp 数
        n_range_bins: 距离门数（通常为 n_samples//2 + 1）
        tx_ddma_idx: DDMA 发射天线索引（长度 n_tx）
        n_subbands: 子带数（多普勒域分割份数）
        noise_est_ratio: 噪声估计百分位数（0-100）

    返回:
        rd_cube: 距离-多普勒谱 (n_rx, n_chirps, n_range_bins)
        rx_nci: 接收通道非相干积累 (n_chirps, n_range_bins)
        noise_est: 每距离门的噪声基底 (n_range_bins,)
        vch_nci: 虚拟通道非相干积累 (n_range_bins, n_chirps)
        max_subband_idx: 每个距离门-子带对应的最大多普勒索引 (n_range_bins, n_subbands)
        max_vch_nci: 对应的峰值功率 (n_range_bins, n_subbands)
    """
    # ----- 1. 多普勒 FFT（沿 chirp 维）-----
    # radarcube: (rx, chirp, range) -> 乘窗后 FFT
    rd_cube = torch.fft.fft(radarcube * win_doppler, dim=1)   # (rx, chirp, range)

    # ----- 2. RX 非相干积累（功率求和）-----
    rx_nci = torch.sum(torch.abs(rd_cube).pow(2), dim=0)       # (chirp, range)
    # 转换为对数域（近似 dB，系数 4096 为经验缩放）
    rx_nci = 4096.0 * torch.log2(rx_nci + 1e-12)

    # ----- 3. 噪声估计（沿多普勒维的百分位数）-----
    q = noise_est_ratio / 100.0
    noise_est = torch.quantile(rx_nci, q, dim=0).to(torch.float32)   # (range,)

    # ----- 4. VCH 非相干积累（DDMA 解调）-----
    tx_ddma = torch.tensor(tx_ddma_idx, dtype=torch.int64, device=DEVICE)  # (n_tx,)
    doppler_indices = torch.arange(n_chirps, dtype=torch.int64, device=DEVICE)[None, :]  # (1, chirp)
    # 每个发射天线对应的多普勒频移索引（循环移位）
    db_idx = (doppler_indices + tx_ddma[:, None] * (n_chirps // n_subbands)) % n_chirps   # (n_tx, n_chirps)

    # 按发射天线求和，然后转置为 (range, chirp) 便于后续子带处理
    vch_nci = torch.sum(rx_nci[db_idx, :], dim=0).t().contiguous()    # (range, chirp)

    # ----- 5. 子带最大值提取（避免 view 导致的不连续内存错误）-----
    subband_step = n_chirps // n_subbands
    max_subband_idx = torch.zeros((n_range_bins, n_subbands), dtype=torch.int32, device=DEVICE)
    max_vch_nci = torch.zeros((n_range_bins, n_subbands), dtype=torch.float32, device=DEVICE)

    # 逐子带循环（子带数通常 <= 32，开销很小）
    for sub_idx in range(n_subbands):
        # 当前子带覆盖的多普勒频点索引：sub_idx, sub_idx+step, ...
        dop_positions = torch.arange(sub_idx, n_chirps, n_subbands, dtype=torch.int64, device=DEVICE)
        vals = vch_nci[:, dop_positions]                      # (range, subband_width)
        max_vals, max_indices = torch.max(vals, dim=1)        # 沿子带宽度取最大值
        max_subband_idx[:, sub_idx] = dop_positions[max_indices].to(torch.int32)
        max_vch_nci[:, sub_idx] = max_vals

    return rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci