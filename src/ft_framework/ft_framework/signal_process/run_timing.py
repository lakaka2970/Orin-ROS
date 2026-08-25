"""
CPU 信号处理链分阶段计时 + 正确性验证（读取 ctrx*.bin 合成数据）

输出：各处理阶段耗时（毫秒）、检测到的峰值 (rb, db) 及其与真值的对比。
"""
import os
import sys
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import RadarConfig
from data_io import readRawBinCasc
from preprocessing_cpu import radar_signal_process_final
from doppler_cpu import doppler_processing_numpy
from peak_detection_cpu import peak_search_numpy
from arraySim_cpu import RadarArrayInitializer
from doa_proc_cpu import doa_main_ultra_separated, doa_env


def main():
    cfg = RadarConfig()
    array_env = RadarArrayInitializer()

    # ---- 天线阵列定义（与 main_timming_test_cpu.py 一致）----
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
    # ------------------------------------------

    timings = {}

    # 1. 读数据
    t0 = time.perf_counter()
    raw_data = readRawBinCasc(".", frameNr=0, nSamples=cfg.n_samples,
                              nRamps=cfg.n_chirps, nChannels=cfg.n_rx)
    timings['read'] = (time.perf_counter() - t0) * 1000.0
    print(f"[Timing] 数据读取: {timings['read']:.2f} ms, 形状 {raw_data.shape}")

    # 2. 预处理（DC去除+加窗+距离FFT）
    t0 = time.perf_counter()
    cube, dc_est, _ = radar_signal_process_final(raw_data, cfg.n_samples, cfg.n_rx,
                                                 cfg.n_chirps, cfg.threshold_scale)
    timings['preprocess'] = (time.perf_counter() - t0) * 1000.0
    print(f"[Timing] 预处理(距离FFT): {timings['preprocess']:.2f} ms, cube {cube.shape}")

    # 3. 多普勒处理（多普勒FFT+NCI+DDMA+子带）
    t0 = time.perf_counter()
    rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci = doppler_processing_numpy(
        cube, cfg.n_rx, cfg.n_chirps, cube.shape[2], cfg.tx_ddma_idx,
        cfg.n_subbands, cfg.noise_est_ratio)
    timings['doppler'] = (time.perf_counter() - t0) * 1000.0
    print(f"[Timing] 多普勒处理(FFT+NCI+DDMA): {timings['doppler']:.2f} ms")

    # 4. 峰值搜索（2D CFAR）
    t0 = time.perf_counter()
    peaks = peak_search_numpy(rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise_est,
                              cfg.tx_ddma_idx, cube.shape[2], cfg.n_chirps, cfg.n_subbands,
                              cfg.ps_scale, cfg.max_peaks_per_rb, cfg.max_total_peaks)
    timings['peak'] = (time.perf_counter() - t0) * 1000.0
    print(f"[Timing] 峰值搜索(2D CFAR): {timings['peak']:.2f} ms, 检测到 {len(peaks)} 个峰值")

    # 5. DOA（角度估计，逐峰值）
    if not doa_env.is_initialized:
        doa_env.prepare_mapping_indices(Array_Azi, Array_Ele)
    t0 = time.perf_counter()
    n_doa = 0
    for peak in peaks:
        channel_data = peak['channel']
        azi_results, ele_results, _, _ = doa_main_ultra_separated(
            channel_data, Array_Azi, AzmChUse, Array_Ele, ElvChUse, 3.0)
        n_doa += 1
    timings['doa'] = (time.perf_counter() - t0) * 1000.0
    timings['doa_per_peak'] = timings['doa'] / n_doa if n_doa else 0.0
    print(f"[Timing] DOA角度估计: {timings['doa']:.2f} ms ({n_doa} 个峰值, 每峰值 {timings['doa_per_peak']:.2f} ms)")

    timings['total'] = timings['preprocess'] + timings['doppler'] + timings['peak'] + timings['doa']
    print(f"[Timing] 总处理时间(不含读取): {timings['total']:.2f} ms")
    print(f"[Timing] 总处理时间(含读取): {timings['total'] + timings['read']:.2f} ms")

    # ---- 正确性：打印检测峰值 vs 真值 ----
    print("\n===== 检测峰值 (rb, db) =====")
    gt = [(80, 100), (139, 130), (139, 210), (250, 200)]
    detected = sorted([(int(p['rb']), int(p['db'])) for p in peaks])
    print("真值:      ", gt)
    print("检测值:    ", detected)
    print(f"range_res={cfg.range_resolution:.4f} m, doppler_res={cfg.doppler_resolution:.4f} m/s")
    for (rb, db) in detected:
        print(f"  rb={rb} (R={rb*cfg.range_resolution:.2f} m), db={db} (v={db*cfg.doppler_resolution:.2f} m/s)")

    # 保存结构化结果
    out = {
        'timings_ms': {k: round(v, 3) for k, v in timings.items()},
        'n_peaks': len(peaks),
        'detected_rb_db': detected,
        'ground_truth_rb_db': gt,
        'range_resolution': round(cfg.range_resolution, 4),
        'doppler_resolution': round(cfg.doppler_resolution, 4),
    }
    import json
    with open("cpu_timing_result.json", "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("\n[out] 结果已写入 cpu_timing_result.json")


if __name__ == "__main__":
    main()
