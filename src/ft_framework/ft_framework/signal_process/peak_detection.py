"""
峰值检测模块：在距离-多普勒谱上执行 2D 局部最大值搜索，并进行能量筛选、
TopK 截断，同时提取对应通道的复数据用于 DOA 估计。
"""

import torch

DEVICE = torch.device('cuda')


@torch.inference_mode()
def peak_search_gpu(rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise,
                    tx_ddma_idx, n_range_bins, n_doppler, n_subbands,
                    ps_scale=25.0, max_peaks_per_rb=12, max_total_peaks=1024):
    """
    全向量化 GPU 峰值检测与筛选。

    参数:
        rd_cube: 距离-多普勒谱 (rx, chirp, range) —— 此处实际传入的是距离 FFT 后的 cube，形状 (rx, chirp, range)
        max_vch_nci: 各子带最大值 (range, subband)
        max_subband_idx: 各子带最大值对应的多普勒索引 (range, subband)
        rx_nci: 接收非相干积累结果 (chirp, range)
        noise: 每距离门噪声基底 (range,)
        tx_ddma_idx: DDMA 发射天线索引 (n_tx,)
        其他: 尺寸参数、阈值比例等

    返回:
        rdcell_list: 每个检测到的峰值字典，包含距离门、多普勒门、功率、噪声、通道数据等。
    """
    # 将 rx_nci 转置为 (range, chirp) 以便与子带矩阵对齐
    rx_nci_rd = rx_nci.t().contiguous()   # (range, chirp)

    # ----- 1. 计算全局检测阈值（噪声基底 × 缩放因子）-----
    thresholds = (noise * ps_scale)[:, None]   # (range, 1)

    # ----- 2. 2D 邻域最大值判断（循环边界处理）-----
    # 多普勒轴（dim=1）为环形
    p_left  = torch.roll(rx_nci_rd, shifts=1, dims=1)
    p_right = torch.roll(rx_nci_rd, shifts=-1, dims=1)
    # 距离轴（dim=0）非环形，边界复制
    p_up   = torch.zeros_like(rx_nci_rd)
    p_down = torch.zeros_like(rx_nci_rd)
    p_up[1:]     = rx_nci_rd[:-1]
    p_up[0]      = rx_nci_rd[0]
    p_down[:-1]  = rx_nci_rd[1:]
    p_down[-1]   = rx_nci_rd[-1]

    local_max_mask = (rx_nci_rd >= p_up) & (rx_nci_rd >= p_down) & \
                     (rx_nci_rd >= p_left) & (rx_nci_rd >= p_right)

    # ----- 3. 将局部极大值掩码映射到子带空间 -----
    db_best = max_subband_idx.to(torch.int64)
    is_local_max_subband = torch.gather(local_max_mask, 1, db_best)   # (range, subband)

    # ----- 4. 合并阈值条件 -----
    peak_mask = (max_vch_nci > thresholds) & is_local_max_subband

    # ----- 5. 每距离门保留最强若干峰值（TopK）-----
    vch_masked = torch.where(peak_mask, max_vch_nci, torch.tensor(-1e9, device=DEVICE))
    topk_vals, topk_sub_indices = torch.topk(vch_masked, k=min(max_peaks_per_rb, n_subbands), dim=1)
    valid_topk_mask = topk_vals > -1e8
    rb_indices, local_cols = torch.where(valid_topk_mask)

    if rb_indices.numel() == 0:
        return []

    final_rb = rb_indices
    final_sub_idx = topk_sub_indices[rb_indices, local_cols]
    final_db = db_best[final_rb, final_sub_idx]
    final_vch = topk_vals[rb_indices, local_cols]

    # 全局 TopK 截断
    if final_rb.numel() > max_total_peaks:
        _, global_topk_idx = torch.topk(final_vch, k=max_total_peaks)
        final_rb = final_rb[global_topk_idx]
        final_db = final_db[global_topk_idx]
        final_vch = final_vch[global_topk_idx]

    n_tx = 16
    n_rx = 16
    tx_idx = torch.arange(n_tx, device=DEVICE).view(-1, 1).expand(n_tx, n_rx)   # (16,16)
    rx_idx = torch.arange(n_rx, device=DEVICE).view(1, -1).expand(n_tx, n_rx)   # (16,16)
    tx_ddma = torch.tensor(tx_ddma_idx, dtype=torch.int64, device=DEVICE)

    # ----- 6. 提取每个峰值的通道数据（256 个虚拟通道）-----
    n_peaks = final_rb.shape[0]
    channel_list = []
    for i in range(n_peaks):
        rb = final_rb[i]
        db = final_db[i]
        # DDMA 解调：每个发射天线对应的多普勒偏移步长（此处为 16）
        idx_dem_dop = (db + tx_ddma[tx_idx] * 16) % n_doppler   # (16,16)
        # rd_cube 索引: (rx, chirp, range) -> 注意此处 rd_cube 形状为 (rx, chirp, range)
        # 使用 rx_idx 和 idx_dem_dop 索引多普勒维，rb 索引距离维
        channel_mat = rd_cube[rx_idx, idx_dem_dop, rb]           # (16,16) 复数
        channel_flat = channel_mat.flatten().cpu().numpy()       # (256,)
        channel_list.append(channel_flat)

    # ----- 7. 组装返回字典（移至 CPU）-----
    rb_cpu = final_rb.cpu().numpy()
    db_cpu = final_db.cpu().numpy()
    vch_cpu = final_vch.cpu().numpy()
    noise_cpu = noise.cpu().numpy()
    p_up_c = p_up.cpu().numpy()
    p_down_c = p_down.cpu().numpy()
    p_left_c = p_left.cpu().numpy()
    p_right_c = p_right.cpu().numpy()
    rx_nci_c = rx_nci_rd.cpu().numpy()

    rdcell_list = []
    for i in range(len(rb_cpu)):
        r, d = rb_cpu[i], db_cpu[i]
        rdcell_list.append({
            'rb': int(r),
            'db': int(d),
            'pow_rb': [float(p_up_c[r, d]), float(rx_nci_c[r, d]), float(p_down_c[r, d])],
            'pow_db': [float(p_left_c[r, d]), float(rx_nci_c[r, d]), float(p_right_c[r, d])],
            'noise': float(noise_cpu[r]),
            'pow_vch': float(vch_cpu[i]),
            'channel': channel_list[i]
        })
    return rdcell_list