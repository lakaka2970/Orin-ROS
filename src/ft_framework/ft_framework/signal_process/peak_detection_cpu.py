"""
峰值检测模块：在距离-多普勒谱中检测局部极值，并提取对应通道数据
"""
import numpy as np
from calibration import apply_calibration


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
    max_total_peaks: int = 1024,
    frame_timestamp_us: int = 0,
    frame_id: int = 0,
    idle_time_idx: int = 0,
    do_calibrate: bool = True
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
        frame_timestamp_us: 帧时间戳（微秒）
        frame_id: 帧ID
        idle_time_idx: 空闲时间索引

    Returns:
        rdcell_list: 峰值列表，每个元素为字典，字段与 RDCell 结构体对应
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
        for sb in range(n_doppler//n_subbands):
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

    n_tx_half = n_tx // 2                                           # 8
    n_rx_half = n_rx // 2                                           # 8

    # 4 象限定义: (tx_slice, rx_slice, db_offset, rb_offset)
    #   Q1: tx 0-7, rx 0-7   → 正常
    #   Q2: tx 8-15, rx 0-7  → doppler+4, range+1
    #   Q3: tx 0-7, rx 8-15  → doppler+4, range+1
    #   Q4: tx 8-15, rx 8-15 → doppler+8, range+2
    tx_slices = [slice(0, n_tx_half), slice(n_tx_half, n_tx),
                 slice(0, n_tx_half), slice(n_tx_half, n_tx)]
    rx_slices = [slice(0, n_rx_half), slice(0, n_rx_half),
                 slice(n_rx_half, n_rx), slice(n_rx_half, n_rx)]
    db_offsets = [0, 4, 4, 8]
    rb_offsets = [0, 1, 1, 2]

    channel_list = []
    for i in range(len(final_rb)):
        rb = final_rb[i]
        db = final_db[i]

        quadrants = []
        for tx_sl, rx_sl, db_off, rb_off in zip(tx_slices, rx_slices, db_offsets, rb_offsets):
            db_q = (db + db_off) % n_doppler                           # doppler 取模防溢出
            rb_q = rb + rb_off
            if rb_q >= n_range_bins:                                   # range 防溢出 → 填零
                n_tx_q = tx_sl.stop - tx_sl.start
                n_rx_q = rx_sl.stop - rx_sl.start
                chan_q = np.zeros((n_tx_q, n_rx_q), dtype=np.complex64)
            else:
                idx_dop_q = (db_q + tx_ddma[tx_sl, np.newaxis] * 16) % n_doppler
                chan_q = rd_cube[rx_grid[:, rx_sl], idx_dop_q, rb_q]   # (n_tx_q, n_rx_q)
            quadrants.append(chan_q)

        # 拼接: [Q1|Q3] 上, [Q2|Q4] 下 → (16,16)
        top    = np.hstack([quadrants[0], quadrants[2]])
        bottom = np.hstack([quadrants[1], quadrants[3]])
        channel_mat = np.vstack([top, bottom])
        channel_list.append(channel_mat.flatten().astype(np.complex64))

    # 通道校准
    if do_calibrate:
        channel_list = [apply_calibration(ch) for ch in channel_list]

    # 9. 构造返回列表（字段与 RDCell 结构体对齐）
    rdcell_list = []
    for i in range(len(final_rb)):
        r = final_rb[i]
        d = final_db[i]

        rdcell_list.append({
            'u32FrameTimeStamp':   int(frame_timestamp_us),          # 1  帧时间戳(us)
            'u16FrameId':          int(frame_id),                    # 2  帧ID
            'u16NofRdCell':        0,                                # 3  RD单元总数（调用方回填）
            'u8Index_Idletime':    int(idle_time_idx),               # 4  空闲时间索引
            'u16Rb':               int(r),                           # 5  距离bin
            'u16Db':               int(d),                           # 6  多普勒bin
            'f32PowRbNci_Q7dB':    [float(p_up[r, d]),
                                     float(rx_nci_rd[r, d]),
                                     float(p_down[r, d])],          # 7  距离向NCI功率(3ch)
            'f32PowDbNci_Q7dB':    [float(p_left[r, d]),
                                     float(rx_nci_rd[r, d]),
                                     float(p_right[r, d])],         # 8  多普勒向NCI功率(3ch)
            'f32PeakPowVchNci_Q7dB': float(final_vch[i]),           # 9  峰值功率(Q7 dB)
            'f32NoiseNci_Q7dB':     float(noise[r]),                 # 10 噪声功率(Q7 dB)
            'u8RdValidFlag':       1,                                # 11 RD有效标志
            'u8RdPeakFlag':        1,                                # 12 RD峰值标志
            'sVch':                channel_list[i],                  # 13 通道数据 complex64(256,)
            # ---- 兼容旧字段 ----
            'rb':      int(r),
            'db':      int(d),
            'pow_rb':  [float(p_up[r, d]), float(rx_nci_rd[r, d]), float(p_down[r, d])],
            'pow_db':  [float(p_left[r, d]), float(rx_nci_rd[r, d]), float(p_right[r, d])],
            'noise':   float(noise[r]),
            'pow_vch': float(final_vch[i]),
            'channel': channel_list[i],
        })
    # 回填 u16NofRdCell
    n_cells = len(rdcell_list)
    for cell in rdcell_list:
        cell['u16NofRdCell'] = n_cells

    return rdcell_list