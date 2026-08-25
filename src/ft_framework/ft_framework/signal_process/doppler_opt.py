"""
多普勒处理模块（优化版）：多普勒 FFT、非相干积累、DDMA 解调、子带峰值。

【策略变更记录】
  1. 子带最大值提取：原版 32 次 for 循环（每次 arange+gather+max+scatter），
     profile_hotspots.py 实测 ~13.1 ms。本版利用 stride 语义 reshape+permute+max
     一次完成，实测 ~0.24 ms（约 55×），数值严格等价。
       原: 对 sub_idx 取 vch_nci[:, sub_idx::32] 求 max
       优化: vch_nci.view(R,16,32).permute(0,2,1) -> [r,s,w]=vch_nci[r,w*32+s]，
             max over w 即等价；全局多普勒索引 = w*32 + s。
  2. RX 非相干积累：原版对后 8 通道先 abs 得到 (8,512,1025) 大张量，再两次 roll
     大张量后 sum。roll 与 sum(dim=0) 可交换，故改为"先 sum 后 roll"，roll 作用在
     (512,1025) 上（8× 小），数值严格等价。
"""
import torch

DEVICE = torch.device('cuda')

win_doppler = torch.hann_window(512, periodic=False, dtype=torch.float32, device=DEVICE)[None, :, None]

# CUDA Graph 兼容缓存：DDMA 索引张量需在 capture 之前预创建
# （torch.tensor(numpy) 含 host->device 拷贝，捕获期间禁止）
_ddma_cache = {}


def _get_ddma_tensors(tx_ddma_idx, n_chirps, n_subbands):
    key = (tuple(int(x) for x in tx_ddma_idx), int(n_chirps), int(n_subbands))
    if key not in _ddma_cache:
        tx_ddma = torch.tensor(tx_ddma_idx, dtype=torch.int64, device=DEVICE)
        n_tx = tx_ddma.shape[0]
        n_tx_half = n_tx // 2
        doppler_indices = torch.arange(n_chirps, dtype=torch.int64, device=DEVICE)[None, :]
        doppler_step = n_chirps // n_subbands
        db_idx_first = (doppler_indices + tx_ddma[:n_tx_half, None] * doppler_step) % n_chirps
        db_idx_second = (doppler_indices + 4 + tx_ddma[n_tx_half:, None] * doppler_step) % n_chirps
        subband_idx = torch.arange(n_subbands, dtype=torch.int64, device=DEVICE)
        _ddma_cache[key] = (db_idx_first, db_idx_second, subband_idx)
    return _ddma_cache[key]


@torch.inference_mode()
def doppler_processing_gpu(radarcube, n_rx, n_chirps, n_range_bins,
                           tx_ddma_idx, n_subbands, noise_est_ratio):
    # 1. 多普勒 FFT
    rd_cube = torch.fft.fft(radarcube * win_doppler, dim=1)

    # 2. RX 非相干积累（sum-before-roll 优化）
    n_rx_half = n_rx // 2
    rx_nci = torch.sum(torch.abs(rd_cube[:n_rx_half]), dim=0)      # (chirp, range)
    shifted_sum = torch.sum(torch.abs(rd_cube[n_rx_half:]), dim=0) # (chirp, range)
    shifted_sum = torch.roll(shifted_sum, shifts=-4, dims=0)       # doppler+4
    shifted_sum = torch.roll(shifted_sum, shifts=-1, dims=1)       # range+1
    shifted_sum[:, -1] = 0.0
    rx_nci = rx_nci + shifted_sum

    # 3. 噪声估计
    q = noise_est_ratio / 100.0
    noise_est = torch.quantile(rx_nci, q, dim=0).to(torch.float32)

    # 4. VCH 非相干积累（DDMA 解调）
    db_idx_first, db_idx_second, subband_idx = _get_ddma_tensors(
        tx_ddma_idx, n_chirps, n_subbands)
    vch_first = rx_nci[db_idx_first, :]
    vch_second = rx_nci[db_idx_second, :]
    vch_second = torch.roll(vch_second, shifts=-1, dims=2)
    vch_second[:, :, -1] = 0.0
    vch_nci = (torch.sum(vch_first, dim=0) + torch.sum(vch_second, dim=0)).t().contiguous()

    # 5. 子带最大值提取（向量化）
    # vch_nci: (range, chirp), chirp = n_subbands * n_positions
    # 原循环对 sub_idx 取 vch_nci[:, sub_idx::n_subbands] 求 max（stride = n_subbands）
    n_positions = n_chirps // n_subbands
    vch_view = vch_nci.view(n_range_bins, n_positions, n_subbands).permute(0, 2, 1)
    # [r, s, w] = vch_nci[r, w*n_subbands + s]；max over w 即等价
    max_vch_nci, max_local = torch.max(vch_view, dim=2)           # (range, subband)
    max_subband_idx = (max_local * n_subbands + subband_idx).to(torch.int32)

    return rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci
