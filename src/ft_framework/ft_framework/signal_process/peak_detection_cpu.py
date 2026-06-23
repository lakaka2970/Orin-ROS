"""
峰值检测模块：在距离-多普勒谱中检测局部极值，并提取对应通道数据
"""
import numpy as np


def peak_search_numpy(
    rd_cube: np.ndarray,
    max_vch_nci: np.ndarray,
    max_subband_idx: np.ndarray,
    rx_nci: np.ndarray,
    noise: np.ndarray,
    tx_ddma_idx: np.ndarray,
    n_range_bins: int,
    n_doppler: int,
    n_subbands: int,
    ps_scale: float = 25.0,
    max_peaks_per_rb: int = 12,
    max_total_peaks: int = 1024
) -> list:
    """
    峰值检测主函数

    Args:
        rd_cube: 距离-多普勒立方体 (n_rx, n_doppler, n_range_bins)
        max_vch_nci: 每个距离门-子带的最大功率 (n_range_bins, n_subbands)
        max_subband_idx: 对应最大功率的多普勒索引 (n_range_bins, n_subbands)
        rx_nci: 接收通道非相干积累 (n_doppler, n_range_bins)
        noise: 每距离门的噪声估计 (n_range_bins,)
        tx_ddma_idx: 发射天线DDMA索引 (n_tx,)
        n_range_bins: 距离门数
        n_doppler: 多普勒单元数
        n_subbands: 子带数
        ps_scale: 峰值信噪比门限缩放
        max_peaks_per_rb: 每距离门最多保留峰值数
        max_total_peaks: 全局最多保留峰值数

    Returns:
        rdcell_list: 峰值列表，每个元素为字典，包含距离门、多普勒门、功率、噪声、通道数据等
    """
    # 转置使维度为 (range, doppler)
    rx_nci_rd = rx_nci.T.copy()            # (n_range_bins, n_doppler)

    # 1. 构造阈值矩阵（每个距离门一个阈值）
    thresholds = noise[:, np.newaxis] * ps_scale     # (n_range_bins, 1)

    # 2. 计算二维局部极大值掩码（多普勒方向循环，距离方向边界复制）
    # 多普勒方向左右邻居（循环）
    p_left = np.roll(rx_nci_rd, shift=1, axis=1)
    p_right = np.roll(rx_nci_rd, shift=-1, axis=1)

    # 距离方向上下邻居（边界复制）
    p_up = np.zeros_like(rx_nci_rd)
    p_down = np.zeros_like(rx_nci_rd)
    p_up[1:] = rx_nci_rd[:-1]
    p_up[0] = rx_nci_rd[0]
    p_down[:-1] = rx_nci_rd[1:]
    p_down[-1] = rx_nci_rd[-1]

    local_max_mask = (rx_nci_rd >= p_up) & (rx_nci_rd >= p_down) & \
                     (rx_nci_rd >= p_left) & (rx_nci_rd >= p_right)

    # 3. 映射到子带峰值上：只考虑每个子带最大值位置是否为局部极大值
    db_best = max_subband_idx.astype(np.int64)
    is_local_max_subband = np.zeros_like(max_vch_nci, dtype=bool)
    for r in range(n_range_bins):
        for sb in range(n_subbands):
            is_local_max_subband[r, sb] = local_max_mask[r, db_best[r, sb]]

    # 4. 合并阈值条件
    peak_mask = (max_vch_nci > thresholds) & is_local_max_subband

    # 5. 按距离门提取前 max_peaks_per_rb 个峰值
    vch_masked = np.where(peak_mask, max_vch_nci, -1e9)

    topk_vals = np.full((n_range_bins, max_peaks_per_rb), -1e9, dtype=np.float32)
    topk_sub_indices = np.zeros((n_range_bins, max_peaks_per_rb), dtype=np.int64)

    for r in range(n_range_bins):
        row = vch_masked[r]
        valid = row > -1e8
        if np.any(valid):
            sorted_idx = np.argsort(row[valid])[::-1]
            k = min(max_peaks_per_rb, np.sum(valid))
            topk_vals[r, :k] = row[valid][sorted_idx[:k]]
            orig_idx = np.where(valid)[0]
            topk_sub_indices[r, :k] = orig_idx[sorted_idx[:k]]

    # 6. 收集所有有效峰值的坐标
    rb_idx, col_idx = np.where(topk_vals > -1e8)
    if len(rb_idx) == 0:
        return []

    final_rb = rb_idx
    final_sub = topk_sub_indices[rb_idx, col_idx]
    final_db = db_best[final_rb, final_sub]
    final_vch = topk_vals[rb_idx, col_idx]

    # 7. 全局峰值数量限制
    if len(final_rb) > max_total_peaks:
        order = np.argsort(final_vch)[::-1][:max_total_peaks]
        final_rb = final_rb[order]
        final_db = final_db[order]
        final_vch = final_vch[order]

    # 8. 提取每个峰值的通道数据（256通道）
    n_tx = 16
    n_rx = 16
    tx_ddma = np.asarray(tx_ddma_idx, dtype=np.int64)
    tx_grid = np.arange(n_tx)[:, np.newaxis]       # (16,1)
    rx_grid = np.arange(n_rx)[np.newaxis, :]       # (1,16)

    channel_list = []
    for i in range(len(final_rb)):
        rb = final_rb[i]
        db = final_db[i]
        # 计算每个发射天线对应的多普勒索引（DDMA解调）
        idx_dem_dop = (db + tx_ddma[:, np.newaxis] * 16) % n_doppler   # (16,16)
        channel_mat = rd_cube[rx_grid, idx_dem_dop, rb]                # (16,16)
        channel_list.append(channel_mat.flatten().astype(np.complex64))

    # 9. 构造返回列表
    rdcell_list = []
    for i in range(len(final_rb)):
        r = final_rb[i]
        d = final_db[i]
        rdcell_list.append({
            'rb': int(r),
            'db': int(d),
            'pow_rb': [float(p_up[r, d]), float(rx_nci_rd[r, d]), float(p_down[r, d])],
            'pow_db': [float(p_left[r, d]), float(rx_nci_rd[r, d]), float(p_right[r, d])],
            'noise': float(noise[r]),
            'pow_vch': float(final_vch[i]),
            'channel': channel_list[i]
        })
    return rdcell_list