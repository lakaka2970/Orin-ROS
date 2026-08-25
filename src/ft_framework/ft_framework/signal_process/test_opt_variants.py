"""优化变体对比：连续布局 FFT vs 跨步 FFT、autocast 效果。"""
import os, sys, time, statistics
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import RadarConfig
from data_io import readRawBinCasc

DEVICE = torch.device('cuda')


def bench(name, fn, n=20):
    for _ in range(3):
        fn()
    torch.cuda.synchronize()
    ts = []
    for _ in range(n):
        torch.cuda.synchronize(); t0 = time.perf_counter()
        fn()
        torch.cuda.synchronize(); ts.append((time.perf_counter() - t0) * 1000.0)
    print(f"[bench] {name:46s} min={min(ts):7.3f}  med={statistics.median(ts):7.3f} ms")
    return statistics.median(ts)


def main():
    cfg = RadarConfig()
    raw = readRawBinCasc(".", 0, cfg.n_samples, cfg.n_chirps, cfg.n_rx).astype(np.int16)
    raw_gpu = torch.from_numpy(raw).to(DEVICE)          # (512,16,2048) int16

    W = torch.hann_window(2048, periodic=False, dtype=torch.float32, device=DEVICE)[None, None, :]
    f = raw_gpu.float()
    dc = torch.mean(f, dim=(0, 2))
    f = (f - dc[None, :, None]) * W
    cube_cur = torch.fft.rfft(f, dim=2).permute(1, 0, 2)   # (16,512,1025) 跨步
    W_D = torch.hann_window(512, periodic=False, dtype=torch.float32, device=DEVICE)[None, :, None]

    bench("doppler fft 跨步 dim=1 (现状)", lambda: torch.fft.fft(cube_cur * W_D, dim=1))

    def fft_contig():
        c = cube_cur.permute(0, 2, 1).contiguous()         # (16,1025,512) 连续
        return torch.fft.fft(c * W_D.permute(0, 2, 1), dim=2)
    bench("doppler fft 连续 dim=2 (含 permute+contig)", fft_contig)

    bench("仅 transpose(16,512,1025)->(16,1025,512)", lambda: cube_cur.permute(0, 2, 1).contiguous())

    # 距离维 rfft 本身
    bench("距离维 rfft (8192x2048) 连续", lambda: torch.fft.rfft(f, dim=2))

    # 完整 preprocess+doppler 无 autocast / 有 autocast
    from preprocessing_opt import radar_signal_process_final
    from doppler_opt import doppler_processing_gpu

    def full():
        c, _, _ = radar_signal_process_final(raw_gpu, cfg.n_samples, cfg.n_rx, cfg.n_chirps, cfg.threshold_scale)
        return doppler_processing_gpu(c, cfg.n_rx, cfg.n_chirps, c.shape[2],
                                      cfg.tx_ddma_idx, cfg.n_subbands, cfg.noise_est_ratio)
    bench("完整 preprocess+doppler (无 autocast)", full)

    def full_ac():
        with torch.autocast('cuda', dtype=torch.float16):
            c, _, _ = radar_signal_process_final(raw_gpu, cfg.n_samples, cfg.n_rx, cfg.n_chirps, cfg.threshold_scale)
            return doppler_processing_gpu(c, cfg.n_rx, cfg.n_chirps, c.shape[2],
                                          cfg.tx_ddma_idx, cfg.n_subbands, cfg.noise_est_ratio)
    bench("完整 preprocess+doppler (autocast fp16)", full_ac)


if __name__ == "__main__":
    main()
