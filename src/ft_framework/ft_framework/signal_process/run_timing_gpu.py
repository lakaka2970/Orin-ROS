"""
GPU 信号处理链分阶段计时 + 正确性验证（读取 ctrx*.bin 合成数据）

在 CUDA GPU 上运行完整信号处理链，测量各阶段 GPU 耗时（kernel 执行 + 启动开销 + 必要的
CPU 后处理），用于与 CPU 串行实现对比。数据流与 rsp_cuda 一致。

注意：
  - 与 CPU 版 run_timing.py 一致，磁盘读取与 CPU→GPU 传输单独记录，不计入"处理耗时"。
  - 原始 ADC 数据为 int16（真实帧 32 MB），传输计时按 int16 测量。
  - 预热 3 次后，正式计时 N 次，输出 min/median/mean。
"""
import os
import sys
import time
import json
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import RadarConfig
from data_io import readRawBinCasc
from preprocessing import radar_signal_process_final
from doppler import doppler_processing_gpu
from peak_detection import peak_search_gpu
from doa_proc import doa_main_batch, doa_main_ultra_separated, doa_env

DEVICE = torch.device('cuda')


def run_once(cfg, raw_gpu, Array_Azi_gpu, AziIdx_Select_gpu, Array_Ele_gpu, EleIdx_Select_gpu):
    """运行一次完整链，返回各阶段耗时(ms) 与检测到的峰值 (rb, db)。"""
    t = {}

    torch.cuda.synchronize(); t0 = time.perf_counter()
    cube, dc, _ = radar_signal_process_final(
        raw_gpu, cfg.n_samples, cfg.n_rx, cfg.n_chirps, cfg.threshold_scale)
    torch.cuda.synchronize(); t['preprocess'] = (time.perf_counter() - t0) * 1000.0

    torch.cuda.synchronize(); t0 = time.perf_counter()
    rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci = \
        doppler_processing_gpu(
            cube, cfg.n_rx, cfg.n_chirps, cube.shape[2],
            cfg.tx_ddma_idx, cfg.n_subbands, cfg.noise_est_ratio)
    torch.cuda.synchronize(); t['doppler'] = (time.perf_counter() - t0) * 1000.0

    torch.cuda.synchronize(); t0 = time.perf_counter()
    peaks = peak_search_gpu(
        rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise_est,
        cfg.tx_ddma_idx, cube.shape[2], cfg.n_chirps, cfg.n_subbands,
        ps_scale=cfg.ps_scale, max_peaks_per_rb=cfg.max_peaks_per_rb,
        max_total_peaks=cfg.max_total_peaks)
    torch.cuda.synchronize(); t['peak'] = (time.perf_counter() - t0) * 1000.0

    doa_threshold_db = 28.0
    if peaks:
        channel_batch = torch.stack([p['channel_gpu'] for p in peaks])
        torch.cuda.synchronize(); t0 = time.perf_counter()
        doa_main_batch(channel_batch, doa_threshold_db)
        torch.cuda.synchronize(); t['doa_batch'] = (time.perf_counter() - t0) * 1000.0
    else:
        t['doa_batch'] = 0.0

    if peaks:
        torch.cuda.synchronize(); t0 = time.perf_counter()
        for peak in peaks:
            doa_main_ultra_separated(
                peak['channel_gpu'], Array_Azi_gpu, AziIdx_Select_gpu,
                Array_Ele_gpu, EleIdx_Select_gpu, doa_threshold_db)
        torch.cuda.synchronize(); t['doa_sep'] = (time.perf_counter() - t0) * 1000.0
    else:
        t['doa_sep'] = 0.0

    t['total'] = t['preprocess'] + t['doppler'] + t['peak'] + t['doa_batch']
    detected = sorted([(int(p['rb']), int(p['db'])) for p in peaks])
    return t, detected, len(peaks)


def main():
    cfg = RadarConfig()
    assert torch.cuda.is_available(), "CUDA 不可用"
    print(f"[GPU] device={torch.cuda.get_device_name(0)}, "
          f"cap={torch.cuda.get_device_capability(0)}, "
          f"torch={torch.__version__}, cuda={torch.version.cuda}")

    # ---- 天线阵列定义（与 rsp_cuda / main_timming_test 一致）----
    AzmChUse = np.array([0,1,2,8,9,10,11,12,13,14,15,16,17,18,24,25,26,27,28,31,32,33,34,40,
                         42,43,45,46,48,49,50,56,58,59,61,62,64,65,66,72,74,75,77,78,80,81,82,
                         88,90,91,93,94,96,97,98,106,107,110,112,113,114,126,208,216,217,218,219,
                         220,221,224,232,234,235,236,237,248,249,251], dtype=np.int64)
    AzmPosUse = np.array([34,29,24,76,70,62,57,52,48,39,43,38,33,28,80,74,66,61,56,47,30,25,20,
                          72,58,53,44,35,26,21,16,68,54,49,40,31,22,17,12,64,50,45,36,27,18,13,8,
                          60,46,41,32,23,14,9,4,42,37,19,10,5,0,15,63,105,99,91,86,81,77,55,97,83,
                          78,73,69,90,84,71], dtype=np.float32)
    ElvChUse = np.array([43,130,131,132,133,134,135,146,147,148,149,150,151,162,163,164,165,166,167,
                         178,179,180,181,182,183,194,195,196,197,198,199,211,212,213,214,215], dtype=np.int64)
    ElvPosUse = np.array([40,74,66,58,50,34,42,81,73,65,57,41,49,67,59,51,43,27,35,54,46,38,30,
                          14,22,47,39,31,23,7,15,32,24,16,0,8], dtype=np.float32)
    n_azi = len(AzmPosUse)
    n_ele = len(ElvPosUse)
    Array_Azi = np.zeros((3, n_azi), dtype=np.float32)
    Array_Azi[0, :] = AzmPosUse
    Array_Ele = np.zeros((3, n_ele), dtype=np.float32)
    Array_Ele[1, :] = ElvPosUse

    AziIdx_Select_gpu = torch.from_numpy(AzmChUse).to(DEVICE)
    EleIdx_Select_gpu = torch.from_numpy(ElvChUse).to(DEVICE)
    Array_Azi_gpu = torch.from_numpy(Array_Azi).to(DEVICE)
    Array_Ele_gpu = torch.from_numpy(Array_Ele).to(DEVICE)

    if not doa_env.is_initialized:
        doa_env.prepare_mapping_indices(Array_Azi, Array_Ele)
        doa_env.cache_selection_indices(AziIdx_Select_gpu, EleIdx_Select_gpu)

    # ---- 读数据（CPU，不计入处理耗时）----
    raw_data = readRawBinCasc(".", frameNr=0, nSamples=cfg.n_samples,
                              nRamps=cfg.n_chirps, nChannels=cfg.n_rx)
    print(f"[Data] raw_data 形状 {raw_data.shape}, dtype {raw_data.dtype}")

    # ---- CPU→GPU 传输（int16，真实 ADC 帧 32 MB，单独记录）----
    raw_int16 = raw_data.astype(np.int16)
    torch.cuda.synchronize()
    t0 = time.perf_counter()
    raw_gpu = torch.from_numpy(raw_int16).to(DEVICE, non_blocking=True)
    torch.cuda.synchronize()
    t_transfer = (time.perf_counter() - t0) * 1000.0
    print(f"[Timing] CPU→GPU 传输(int16, {raw_int16.nbytes/1e6:.1f} MB): {t_transfer:.2f} ms")

    # ---- 预热 ----
    for _ in range(3):
        run_once(cfg, raw_gpu, Array_Azi_gpu, AziIdx_Select_gpu, Array_Ele_gpu, EleIdx_Select_gpu)
    torch.cuda.synchronize()

    # ---- 正式计时（N 次取 min/median/mean）----
    N = 5
    all_t = []
    detected = None
    n_peaks = 0
    for i in range(N):
        t, detected, n_peaks = run_once(cfg, raw_gpu, Array_Azi_gpu, AziIdx_Select_gpu,
                                        Array_Ele_gpu, EleIdx_Select_gpu)
        all_t.append(t)
        print(f"[iter {i}] pre={t['preprocess']:.2f} dop={t['doppler']:.2f} "
              f"peak={t['peak']:.2f} doa_batch={t['doa_batch']:.2f} "
              f"doa_sep={t['doa_sep']:.2f} total={t['total']:.2f} ms, {n_peaks} 峰值")

    keys = ['preprocess', 'doppler', 'peak', 'doa_batch', 'doa_sep', 'total']
    stats = {}
    for k in keys:
        vals = [t[k] for t in all_t]
        stats[k] = {
            'min': round(min(vals), 3),
            'median': round(float(np.median(vals)), 3),
            'mean': round(float(np.mean(vals)), 3),
        }
    stats['transfer_ms'] = round(t_transfer, 3)

    # ---- 正确性 ----
    gt = [(80, 100), (139, 130), (139, 210), (250, 200)]
    print("\n===== 检测峰值 (rb, db) =====")
    print("真值:      ", gt)
    print("检测值:    ", detected)
    print(f"range_res={cfg.range_resolution:.4f} m, doppler_res={cfg.doppler_resolution:.4f} m/s")
    for (rb, db) in detected:
        print(f"  rb={rb} (R={rb*cfg.range_resolution:.2f} m), db={db} (v={db*cfg.doppler_resolution:.2f} m/s)")

    out = {
        'device': torch.cuda.get_device_name(0),
        'capability': f"{torch.cuda.get_device_capability(0)}",
        'torch_version': torch.__version__,
        'cuda_version': torch.version.cuda,
        'frame_bytes_int16': int(raw_int16.nbytes),
        'transfer_ms': round(t_transfer, 3),
        'timings_ms': stats,
        'n_peaks': n_peaks,
        'detected_rb_db': detected,
        'ground_truth_rb_db': gt,
        'range_resolution': round(cfg.range_resolution, 4),
        'doppler_resolution': round(cfg.doppler_resolution, 4),
    }
    with open("gpu_timing_result.json", "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("\n[out] 结果已写入 gpu_timing_result.json")


if __name__ == "__main__":
    main()
