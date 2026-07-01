"""
多普勒处理模块
包含：多普勒FFT、非相干积累、DDMA解调、子带峰值提取
"""
import numpy as np
import matplotlib.pyplot as plt

_N_CHIRPS = 512
WIN_DOPPLER = np.hanning(_N_CHIRPS).astype(np.float32).reshape(1, -1, 1)


def doppler_processing_numpy(
    radarcube: np.ndarray,
    n_rx: int,
    n_chirps: int,
    n_range_bins: int,
    tx_ddma_idx: np.ndarray,
    n_subbands: int,
    noise_est_ratio: float
) -> tuple:
    """
    多普勒处理主函数

    Args:
        radarcube: 距离FFT结果，形状 (n_rx, n_chirps, n_range_bins)
        n_rx: 接收天线数
        n_chirps: Chirp数量
        n_range_bins: 距离门数
        tx_ddma_idx: 发射天线DDMA索引，形状 (n_tx,)
        n_subbands: 子带数
        noise_est_ratio: 噪声估计百分位数（%）

    Returns:
        rd_cube: 距离-多普勒立方体 (n_rx, n_chirps, n_range_bins)
        rx_nci: 接收通道非相干积累，形状 (n_chirps, n_range_bins) [对数功率]
        noise_est: 每距离门的噪声估计，形状 (n_range_bins,)
        vch_nci: 虚拟通道非相干积累 (n_range_bins, n_chirps)
        max_subband_idx: 每个距离门-子带中最大值的多普勒索引 (n_range_bins, n_subbands)
        max_vch_nci: 每个距离门-子带中的最大功率 (n_range_bins, n_subbands)
    """
    # 1. 多普勒FFT（沿Chirp维）
    rd_cube = np.fft.fft(radarcube * WIN_DOPPLER, axis=1)        # (rx, chirp, range)

    
    # 2. RX非相干积累：rx 0-7 正常累加，rx 8-15 偏移累加
    n_rx_half = n_rx // 2                                          # 8
    # rx 0-7: 正常累加（同一 rangebin, 同一 dopplerbin）
    rx_nci = np.sum(np.abs(rd_cube[:n_rx_half]), axis=0)          # (chirp, range)
    # rx 8-15: rangebin+1, dopplerbin+4 位置累加
    rd_shifted = np.abs(rd_cube[n_rx_half:])                      # (8, chirps, range)
    rd_shifted = np.roll(rd_shifted, shift=-4, axis=1)            # doppler+4（取模回绕防溢出）
    rd_shifted = np.roll(rd_shifted, shift=-1, axis=2)            # range+1
    rd_shifted[:, :, -1] = 0.0                                    # 最后一列填充0，防止range溢出回绕
    rx_nci += np.sum(rd_shifted, axis=0)
    #rx_nci = 4096.0 * np.log2(rx_nci + 1e-12)

    # ---- 对比绘图：rx_nci_pre vs rx_nci @ rangebin=16 ----
    # rx_nci_pre = np.sum(np.abs(rd_cube), axis=0)
    # rangebin_idx = 16
    # doppler_axis = np.arange(n_chirps)
    # plt.figure(figsize=(10, 5))
    # plt.plot(doppler_axis, rx_nci_pre[:, rangebin_idx], label='rx_nci_pre (all rx normal)', alpha=0.7)
    # plt.plot(doppler_axis, rx_nci[:, rangebin_idx], label='rx_nci (split accumulation)', alpha=0.7)
    # plt.xlabel('Doppler bin')
    # plt.ylabel('Power')
    # plt.title(f'rx_nci comparison at rangebin={rangebin_idx}')
    # plt.legend()
    # plt.grid(True, alpha=0.3)
    # plt.show()
    # --------------------------------------------------------

    # 3. 噪声估计（按距离维的分位数）
    q = noise_est_ratio / 100.0
    noise_est = np.quantile(rx_nci, q, axis=0).astype(np.float32)   # (range,)  method='linear' removed for numpy<1.22 compat

    # 4. VCH非相干积累（DDMA解调）
    tx_ddma = np.asarray(tx_ddma_idx, dtype=np.int64)
    n_tx = len(tx_ddma)
    n_tx_half = n_tx // 2
    doppler_indices = np.arange(n_chirps, dtype=np.int64)[np.newaxis, :]          # (1, chirp)
    doppler_step = n_chirps // n_subbands

    # 前一半发射天线：正常多普勒索引
    db_idx_first = (doppler_indices + tx_ddma[:n_tx_half, np.newaxis] * doppler_step) % n_chirps
    vch_first = rx_nci[db_idx_first, :]                                           # (n_tx_half, chirp, range)

    # 后一半发射天线：doppler索引+4, rangebin+1
    db_idx_second = (doppler_indices + 4 + tx_ddma[n_tx_half:, np.newaxis] * doppler_step) % n_chirps
    vch_second = rx_nci[db_idx_second, :]                                         # (n_tx_half, chirp, range)
    vch_second = np.roll(vch_second, shift=-1, axis=2)                            # range+1
    vch_second[:, :, -1] = 0.0                                                    # 防止range溢出回绕

    vch_nci = (np.sum(vch_first, axis=0) + np.sum(vch_second, axis=0)).T         # (range, chirp)


    # ---- 对比绘图：vch_nci_pre vs vch_nci @ rangebin=16 ----
    # db_idx_all = (doppler_indices + tx_ddma[:, np.newaxis] * doppler_step) % n_chirps
    # vch_nci_pre = np.sum(rx_nci[db_idx_all, :], axis=0).T                          # (range, chirp)

    # rangebin_idx = 16
    # doppler_axis = np.arange(n_chirps)
    # plt.figure(figsize=(10, 5))
    # plt.plot(doppler_axis, vch_nci_pre[rangebin_idx, :], label='vch_nci_pre (all tx normal)', alpha=0.7)
    # plt.plot(doppler_axis, vch_nci[rangebin_idx, :], label='vch_nci (split tx accumulation)', alpha=0.7)
    # plt.xlabel('Doppler bin')
    # plt.ylabel('Power')
    # plt.title(f'vch_nci comparison at rangebin={rangebin_idx}')
    # plt.legend()
    # plt.grid(True, alpha=0.3)
    # plt.show()
    # ----------------------------------------------------------

    # 5. 子带最大值提取（512 Doppler分16组，每组32个，取最大值）
    n_groups = (n_chirps // n_subbands)
    max_subband_idx = np.zeros((n_range_bins, n_groups), dtype=np.int32)
    max_vch_nci = np.zeros((n_range_bins, n_groups), dtype=np.float32)

    for grp in range(n_groups):
        # 交错分组: Group k = [k, k+16, k+32, ..., k+496]  共32个bin
        dop_positions = np.arange(grp, n_chirps, n_groups, dtype=np.int64)
        vals = vch_nci[:, dop_positions]                              # (range, 32)
        max_vals = np.max(vals, axis=1)
        max_idx = dop_positions[np.argmax(vals, axis=1)]
        max_subband_idx[:, grp] = max_idx
        max_vch_nci[:, grp] = max_vals

    return rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci