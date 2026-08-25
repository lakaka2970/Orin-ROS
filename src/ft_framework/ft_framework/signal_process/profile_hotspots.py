"""
热点剖析脚本：逐微操作计时，定位 GPU 单帧真正的耗时来源。

目的：区分 (a) cuFFT 计算占比 与 (b) 小算子 launch/访存开销占比，
     为选择优化策略（CUDA Graph vs fp16 FFT vs 去除冗余）提供依据。

用法: python profile_hotspots.py   (在含 ctrx*.bin 的目录下运行)
"""
import os, sys, time, statistics
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import RadarConfig
from data_io import readRawBinCasc

DEVICE = torch.device('cuda')

WIN_RANGE = torch.hann_window(2048, periodic=False, dtype=torch.float32, device=DEVICE)[None, None, :]
WIN_DOPP  = torch.hann_window(512, periodic=False, dtype=torch.float32, device=DEVICE)[None, :, None]


def bench(name, fn, n=15):
    """预热 3 次后计时 n 次（纯计时，返回中位数）。"""
    for _ in range(3):
        fn()
    torch.cuda.synchronize()
    ts = []
    for _ in range(n):
        torch.cuda.synchronize(); t0 = time.perf_counter()
        fn()
        torch.cuda.synchronize(); ts.append((time.perf_counter() - t0) * 1000.0)
    mn, md, me = min(ts), statistics.median(ts), sum(ts) / len(ts)
    print(f"[micro] {name:44s} min={mn:8.3f}  med={md:8.3f}  mean={me:8.3f} ms")
    return md


def main():
    cfg = RadarConfig()
    print(f"[GPU] {torch.cuda.get_device_name(0)}, cap={torch.cuda.get_device_capability(0)}")

    raw = readRawBinCasc(".", 0, cfg.n_samples, cfg.n_chirps, cfg.n_rx)
    raw_int16 = raw.astype(np.int16)
    raw_gpu = torch.from_numpy(raw_int16).to(DEVICE)          # int16 (512,16,2048)

    print("\n===== 预处理 (preprocess) 微操作 =====")
    adc = raw_gpu
    bench("1. int16->float32 转换", lambda: adc.float())
    adc_f = adc.float()

    bench("2. DC 均值 (mean dim=0,2)", lambda: torch.mean(adc_f, dim=(0, 2)))
    dc = torch.mean(adc_f, dim=(0, 2))

    bench("3. DC 广播相减", lambda: adc_f - dc[None, :, None])
    adc_dc = adc_f - dc[None, :, None]

    def interp_suppress():
        diff = torch.abs(adc_dc[:, :, 1:] - adc_dc[:, :, :-1])
        diff_ave = torch.sum(diff, dim=2) // 2048
        threshold = (diff_ave * cfg.threshold_scale)[:, :, None]
        return adc_dc > threshold
    bench("4. [死代码] 干扰抑制(diff+sum+thr+mask)", interp_suppress)

    bench("5. 距离维汉宁加窗", lambda: adc_dc * WIN_RANGE)
    adc_win = adc_dc * WIN_RANGE

    bench("6. 距离维 rfft (8192x2048)", lambda: torch.fft.rfft(adc_win, dim=2))
    cube = torch.fft.rfft(adc_win, dim=2)

    bench("7. permute -> (rx,chirp,range)", lambda: cube.permute(1, 0, 2))
    cube = cube.permute(1, 0, 2)                               # (16,512,1025)

    print("\n===== 多普勒 (doppler) 微操作 =====")
    bench("8. 多普勒维 fft (16400x512)", lambda: torch.fft.fft(cube * WIN_DOPP, dim=1))
    rd = torch.fft.fft(cube * WIN_DOPP, dim=1)

    def rx_nci_fn():
        nci = torch.sum(torch.abs(rd[:8]), dim=0)
        sh = torch.abs(rd[8:])
        sh = torch.roll(sh, shifts=-4, dims=1)
        sh = torch.roll(sh, shifts=-1, dims=2)
        sh[:, :, -1] = 0.0
        return nci + torch.sum(sh, dim=0)
    rx_nci = rx_nci_fn()
    bench("9. RX 非相干积累 (abs+sum+roll)", rx_nci_fn)
    bench("10. 噪声估计 (quantile)", lambda: torch.quantile(rx_nci, 0.5, dim=0))

    tx_ddma = torch.tensor(cfg.tx_ddma_idx, dtype=torch.int64, device=DEVICE)
    doppler_indices = torch.arange(512, dtype=torch.int64, device=DEVICE)[None, :]
    doppler_step = 512 // 32
    db_idx_first = (doppler_indices + tx_ddma[:8, None] * doppler_step) % 512
    db_idx_second = (doppler_indices + 4 + tx_ddma[8:, None] * doppler_step) % 512

    def ddma_fn():
        vf = rx_nci[db_idx_first, :]
        vs = rx_nci[db_idx_second, :]
        vs = torch.roll(vs, shifts=-1, dims=2)
        vs[:, :, -1] = 0.0
        return (torch.sum(vf, dim=0) + torch.sum(vs, dim=0)).t().contiguous()
    vch_nci = ddma_fn()
    bench("11. DDMA 解调 (2 次大 gather+sum)", ddma_fn)

    def subband_loop():
        mx = torch.zeros((1025, 32), dtype=torch.float32, device=DEVICE)
        for sub_idx in range(32):
            dop_positions = torch.arange(sub_idx, 512, 32, dtype=torch.int64, device=DEVICE)
            vals = vch_nci[:, dop_positions]
            max_vals, _ = torch.max(vals, dim=1)
            mx[:, sub_idx] = max_vals
        return mx
    bench("12. [原] 子带最大值 32次循环", subband_loop)

    def subband_vec():
        return torch.max(vch_nci.view(1025, 16, 32).permute(0, 2, 1), dim=2).values
    bench("13. [优化] 子带最大值 向量化", subband_vec)

    print("\n[完成] 关键: 看第 6/8 步 FFT 占比决定策略。")


if __name__ == "__main__":
    main()
