"""
多普勒处理模块：对雷达数据立方体执行多普勒 FFT、非相干积累、DDMA 解调，
并提取各子带最大峰值位置。
"""

import torch
import matplotlib.pyplot as plt

DEVICE = torch.device('cuda')
_n_chirps = 512      # Chirp 数量（与 RadarConfig 保持一致）

# 多普勒维汉宁窗（预先加载到 GPU）
win_doppler = torch.hann_window(_n_chirps, periodic=False, dtype=torch.float32, device=DEVICE)[None, :, None]


@torch.inference_mode()
def doppler_processing_gpu(radarcube, n_rx, n_chirps, n_range_bins,
                           tx_ddma_idx, n_subbands, noise_est_ratio):
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

    # ----- 2. RX 非相干积累：rx 0-7 正常, rx 8-15 偏移累加 -----
    n_rx_half = n_rx // 2
    rx_nci_pre = torch.sum(torch.abs(rd_cube), dim=0)                      # (chirp, range) — 全 rx 正常累加 (对比用)
    rx_nci = torch.sum(torch.abs(rd_cube[:n_rx_half]), dim=0)              # (chirp, range)
    rd_shifted = torch.abs(rd_cube[n_rx_half:])                            # (8, chirps, range)
    rd_shifted = torch.roll(rd_shifted, shifts=-4, dims=1)                 # doppler+4
    rd_shifted = torch.roll(rd_shifted, shifts=-1, dims=2)                 # range+1
    rd_shifted[:, :, -1] = 0.0                                             # 防 range 溢出
    rx_nci += torch.sum(rd_shifted, dim=0)
    # 转换为对数域（近似 dB，系数 4096 为经验缩放）

    # ----- 3. 噪声估计（沿多普勒维的百分位数）-----
    q = noise_est_ratio / 100.0
    noise_est = torch.quantile(rx_nci, q, dim=0).to(torch.float32)   # (range,)

    # ----- 4. VCH 非相干积累（DDMA 解调）-----
    tx_ddma = torch.tensor(tx_ddma_idx, dtype=torch.int64, device=DEVICE)
    n_tx = tx_ddma.shape[0]
    n_tx_half = n_tx // 2
    doppler_indices = torch.arange(n_chirps, dtype=torch.int64, device=DEVICE)[None, :]
    doppler_step = n_chirps // n_subbands

    # 全 tx 正常 DDMA (对比用)
    db_idx_all = (doppler_indices + tx_ddma[:, None] * doppler_step) % n_chirps
    vch_nci_pre = torch.sum(rx_nci[db_idx_all, :], dim=0).t().contiguous()  # (range, chirp)

    # 前一半 tx: 正常多普勒索引
    db_idx_first = (doppler_indices + tx_ddma[:n_tx_half, None] * doppler_step) % n_chirps
    vch_first = rx_nci[db_idx_first, :]                                    # (n_tx_half, chirp, range)

    # 后一半 tx: doppler+4, range+1
    db_idx_second = (doppler_indices + 4 + tx_ddma[n_tx_half:, None] * doppler_step) % n_chirps
    vch_second = rx_nci[db_idx_second, :]                                  # (n_tx_half, chirp, range)
    vch_second = torch.roll(vch_second, shifts=-1, dims=2)                 # range+1
    vch_second[:, :, -1] = 0.0                                             # 防 range 溢出

    vch_nci = (torch.sum(vch_first, dim=0) + torch.sum(vch_second, dim=0)).t().contiguous()

    # ---- 对比绘图 (默认关闭, 改为 True 可显示) ----
    if False:
        rangebin_idx = 16
        doppler_axis = torch.arange(n_chirps, device=DEVICE)

        # rx_nci: 全 rx 正常 vs split 累加
        plt.figure(figsize=(10, 5))
        plt.plot(doppler_axis.cpu(), rx_nci_pre[:, rangebin_idx].cpu(), label='rx_nci_pre (all rx normal)', alpha=0.7)
        plt.plot(doppler_axis.cpu(), rx_nci[:, rangebin_idx].cpu(), label='rx_nci (split accumulation)', alpha=0.7)
        plt.xlabel('Doppler bin')
        plt.ylabel('Power')
        plt.title(f'GPU rx_nci comparison at rangebin={rangebin_idx}')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()

        # vch_nci: 全 tx 正常 DDMA vs split DDMA
        plt.figure(figsize=(10, 5))
        plt.plot(doppler_axis.cpu(), vch_nci_pre[rangebin_idx, :].cpu(), label='vch_nci_pre (all tx normal)', alpha=0.7)
        plt.plot(doppler_axis.cpu(), vch_nci[rangebin_idx, :].cpu(), label='vch_nci (split tx accumulation)', alpha=0.7)
        plt.xlabel('Doppler bin')
        plt.ylabel('Power')
        plt.title(f'GPU vch_nci comparison at rangebin={rangebin_idx}')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()

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