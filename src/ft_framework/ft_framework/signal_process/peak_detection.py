"""
峰值检测模块：在距离-多普勒谱上执行 2D 局部最大值搜索，并进行能量筛选、
TopK 截断，同时提取对应通道的复数据用于 DOA 估计。
"""

import torch
import numpy as np
from ft_framework.signal_process.calibration import apply_calibration

DEVICE = torch.device('cuda')


@torch.inference_mode()
def peak_search_gpu(rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise,
                    tx_ddma_idx, n_range_bins, n_doppler, n_subbands,
                    ps_scale=25.0, max_peaks_per_rb=12, max_total_peaks=1024,
                    frame_timestamp_us=0, frame_id=0, idle_time_idx=0,
                    do_calibrate=True):
    """
    全向量化 GPU 峰值检测与筛选。

    参数:
        rd_cube: 距离-多普勒谱 (rx, chirp, range)
        max_vch_nci: 各子带最大值 (range, subband)
        max_subband_idx: 各子带最大值对应的多普勒索引 (range, subband)
        rx_nci: 接收非相干积累结果 (chirp, range)
        noise: 每距离门噪声基底 (range,)
        tx_ddma_idx: DDMA 发射天线索引 (n_tx,)
        frame_timestamp_us: 帧时间戳(us)
        frame_id: 帧ID
        idle_time_idx: 空闲时间索引
        其他: 尺寸参数、阈值比例等

    返回:
        rdcell_list: 每个检测到的峰值字典，字段与 RDCell 结构体对齐
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
    n_peaks = final_rb.shape[0]

    if n_peaks == 0:
        return []

    # ----- 6. 向量化通道提取: 一次 gather 所有 peaks 的 (16,16) 虚拟通道矩阵 -----
    # 计算每个 (tx, rx) 位置的 db/rb 偏移量:
    #   rb_off = (tx >= 8) + (rx >= 8)    → 0, 1, 2
    #   db_off = rb_off * 4               → 0, 4, 8
    tx_grid = torch.arange(n_tx, device=DEVICE).view(1, 16, 1)   # (1, 16, 1)
    rx_grid = torch.arange(n_rx, device=DEVICE).view(1, 1, 16)   # (1, 1, 16)
    tx_hi  = (tx_grid >= 8).to(torch.int64)                      # (1, 16, 1)
    rx_hi  = (rx_grid >= 8).to(torch.int64)                      # (1, 1, 16)
    rb_off = (tx_hi + rx_hi)                                     # (1, 16, 16) 每元素为 0/1/2
    db_off = rb_off * 4                                           # (1, 16, 16) 每元素为 0/4/8

    # 展开 peak 维度: (N, 1, 1) + (1, 16, 16) → (N, 16, 16)
    rb_all = final_rb.view(-1, 1, 1)                             # (N, 1, 1)
    db_all = final_db.view(-1, 1, 1)                             # (N, 1, 1)

    rb_idx = rb_all + rb_off                                     # (N, 16, 16)
    # DDMA 解调: doppler = (db + db_off + tx_ddma[tx] * step) % n_doppler
    tx_ddma_t = torch.tensor(tx_ddma_idx, dtype=torch.int64, device=DEVICE)
    ddma_step = n_doppler // n_subbands                           # 16
    ddma_shift = tx_ddma_t[tx_grid] * ddma_step                  # (1, 16, 1)
    db_idx = (db_all + db_off + ddma_shift) % n_doppler          # (N, 16, 16)

    # range 溢出遮罩
    overflow = rb_idx >= n_range_bins
    rb_idx = torch.clamp(rb_idx, 0, n_range_bins - 1)

    # ★ 单次向量化 gather: (N, 16, 16) 一次完成所有 peaks 的通道提取
    # rd_cube: (n_rx=16, chirp=512, range) — rx_grid 广播到 (N,16,16)
    rx_batch = rx_grid.expand(n_peaks, -1, -1)                   # (N, 16, 16)
    channel_all = rd_cube[rx_batch, db_idx, rb_idx]              # (N, 16, 16)
    channel_all[overflow] = 0j                                    # 溢出位置填零

    # 展平为 (N, 256) GPU tensor
    channel_flat_all = channel_all.reshape(n_peaks, -1)           # (N, 256)

    # ----- 7. 通道校准 (批量 CPU 传输, 避免 per-peak 往返) -----
    if do_calibrate:
        # 单次 GPU→CPU 批量传输
        channels_np = channel_flat_all.cpu().numpy()              # (N, 256)
        calibrated_np = [apply_calibration(ch) for ch in channels_np]
        # 单次 CPU→GPU 批量回传
        channel_list = calibrated_np
        channel_gpu_list = [torch.from_numpy(ch).to(DEVICE) for ch in calibrated_np]
    else:
        channel_list = [ch.cpu().numpy() for ch in channel_flat_all]
        channel_gpu_list = [ch for ch in channel_flat_all]        # 保持 GPU tensor 引用

    # ----- 7. 组装返回字典（移至 CPU）-----
    rb_cpu = final_rb.cpu().numpy()
    db_cpu = final_db.cpu().numpy()
    noise_cpu = noise.cpu().numpy()
    p_up_c = p_up.cpu().numpy()
    p_down_c = p_down.cpu().numpy()
    p_left_c = p_left.cpu().numpy()
    p_right_c = p_right.cpu().numpy()
    rx_nci_c = rx_nci_rd.cpu().numpy()

    # ----- 8. 组装返回字典（字段与 RDCell 结构体对齐）-----
    rdcell_list = []
    for i in range(len(rb_cpu)):
        r, d = rb_cpu[i], db_cpu[i]

        rdcell_list.append({
            'u32FrameTimeStamp':   int(frame_timestamp_us),
            'u16FrameId':          int(frame_id),
            'u16NofRdCell':        0,
            'u8Index_Idletime':    0,
            'u16Rb':               int(r),
            'u16Db':               int(d),
            'f32PowRbNci_Q7dB':    [float(p_up_c[r, d]),
                                     float(rx_nci_c[r, d]),
                                     float(p_down_c[r, d])],
            'f32PowDbNci_Q7dB':    [float(p_left_c[r, d]),
                                     float(rx_nci_c[r, d]),
                                     float(p_right_c[r, d])],
            'f32PeakPowVchNci_Q7dB': float(rx_nci_c[r, d]),  # RX NCI 功率 (实际信号功率)
            'f32NoiseNci_Q7dB':     float(noise_cpu[r]),
            'u8RdValidFlag':       1,
            'u8RdPeakFlag':        1,
            'sVch':                channel_list[i],        # complex64(256,) numpy
            # 兼容旧字段
            'rb':      int(r),
            'db':      int(d),
            'pow_rb':  [float(p_up_c[r, d]), float(rx_nci_c[r, d]), float(p_down_c[r, d])],
            'pow_db':  [float(p_left_c[r, d]), float(rx_nci_c[r, d]), float(p_right_c[r, d])],
            'noise':   float(noise_cpu[r]),
            'channel':       channel_list[i],              # (256,) numpy — 兼容旧代码
            'channel_gpu':   channel_gpu_list[i],          # (256,) GPU tensor — DOA 直通, 零拷贝
        })
    n_cells = len(rdcell_list)
    for cell in rdcell_list:
        cell['u16NofRdCell'] = n_cells

    return rdcell_list