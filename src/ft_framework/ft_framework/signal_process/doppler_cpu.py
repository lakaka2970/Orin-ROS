"""
多普勒处理模块
包含：多普勒FFT、非相干积累、DDMA解调、子带峰值提取
"""
import numpy as np

_N_CHIRPS = 512
WIN_DOPPLER = np.hanning(_N_CHIRPS).astype(np.float32).reshape(1, -1, 1)


def doppler_processing_numpy(
    radarcube: np.ndarray,
    n_rx: int,
    n_chirps: int,
    n_range_bins: int,
    tx_ddma_idx: np.ndarray,
    n_subbands: int,
    noise_est_ratio: float = 50.0
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

    # 2. RX非相干积累：对各天线功率求和并转换到对数域
    rx_nci = np.sum(np.abs(rd_cube) ** 2, axis=0)                # (chirp, range)
    rx_nci = 4096.0 * np.log2(rx_nci + 1e-12)

    # 3. 噪声估计（按距离维的分位数）
    q = noise_est_ratio / 100.0
    noise_est = np.quantile(rx_nci, q, axis=0, interpolation='linear').astype(np.float32)
    # 4. VCH非相干积累（DDMA解调）
    tx_ddma = np.asarray(tx_ddma_idx, dtype=np.int64)
    doppler_indices = np.arange(n_chirps, dtype=np.int64)[np.newaxis, :]          # (1, chirp)
    # 每个发射天线的多普勒偏移索引
    db_idx = (doppler_indices + tx_ddma[:, np.newaxis] * (n_chirps // n_subbands)) % n_chirps   # (n_tx, chirp)
    vch_nci = np.sum(rx_nci[db_idx, :], axis=0).T                                 # (range, chirp)

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