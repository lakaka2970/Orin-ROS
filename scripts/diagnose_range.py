#!/usr/bin/env python3
"""
诊断脚本: 分析原始 ADC 数据的 Range Profile
检查真实信号峰值对应的距离 bin，定位距离估计错误的根源。
"""

import numpy as np
import sys

# 参数
NUM_CHIRPS = 1024
NUM_RX = 8
NUM_SAMPLES = 2048
RX_PER_HALF = 4
FRAME_BYTES = NUM_CHIRPS * NUM_RX * NUM_SAMPLES * 2  # 32 MiB

def load_frame(filepath, frame_idx=0):
    """加载第 frame_idx 帧 (32 MiB)"""
    offset = frame_idx * FRAME_BYTES
    with open(filepath, 'rb') as f:
        f.seek(offset)
        data = f.read(FRAME_BYTES)
    arr = np.frombuffer(data, dtype=np.int16).astype(np.float32)
    half_elems = len(arr) // 2
    half0 = arr[:half_elems].reshape(NUM_CHIRPS, RX_PER_HALF, NUM_SAMPLES)
    half1 = arr[half_elems:2*half_elems].reshape(NUM_CHIRPS, RX_PER_HALF, NUM_SAMPLES)
    return half0, half1

def analyze_chirp(chirp_data, label="", show_bins=50):
    """分析单个 chirp 的 range profile"""
    # chirp_data: (2048,) float32
    # DC removal
    chirp = chirp_data - chirp_data.mean()

    # Hann window
    win = np.hanning(len(chirp))
    chirp_win = chirp * win

    # Real FFT
    spectrum = np.fft.rfft(chirp_win, n=2048)
    power = np.abs(spectrum) ** 2
    log_power = 10 * np.log10(power + 1e-10)

    # 找前 10 个峰值
    peaks = []
    for i in range(2, len(log_power) - 1):
        if log_power[i] > log_power[i-1] and log_power[i] > log_power[i+1]:
            peaks.append((i, log_power[i]))
    peaks.sort(key=lambda x: -x[1])
    top_peaks = peaks[:15]

    print(f"\n{'='*70}")
    print(f"  {label}")
    print(f"{'='*70}")
    print(f"  DC 电平: {chirp_data.mean():.1f}, std: {chirp_data.std():.1f}")
    print(f"  噪声底板: {np.median(log_power):.1f} dB, 最大值: {log_power.max():.1f} dB")
    print(f"\n  Top 15 峰值 (range_bin, power_dB):")
    for bin_idx, pwr in top_peaks:
        # 尝试不同 range_resolution
        r_0146 = bin_idx * 0.1465
        r_001 = bin_idx * 0.01
        r_0005 = bin_idx * 0.005
        r_bin_khz = bin_idx  # just show bin
        print(f"    bin={bin_idx:5d}  power={pwr:7.1f} dB  "
              f"(R@0.1465={r_0146:8.2f}m, R@0.01={r_001:6.2f}m, R@0.005={r_0005:6.2f}m)")

    return log_power, top_peaks

def analyze_as_complex(chirp_data, label="", show_bins=50):
    """将数据按 I/Q 交织解析: 2048 real → 1024 complex"""
    real_part = chirp_data[0::2]
    imag_part = chirp_data[1::2]
    complex_data = (real_part + 1j * imag_part).astype(np.complex64)

    # DC removal
    complex_data -= complex_data.mean()

    # Hann window
    win = np.hanning(len(complex_data))
    complex_win = complex_data * win

    # Complex FFT (双边谱)
    spectrum = np.fft.fft(complex_win, n=1024)
    spectrum_shifted = np.fft.fftshift(spectrum)
    power = np.abs(spectrum_shifted) ** 2
    log_power = 10 * np.log10(power + 1e-10)

    peaks = []
    for i in range(2, len(log_power) - 1):
        if log_power[i] > log_power[i-1] and log_power[i] > log_power[i+1]:
            peaks.append((i - len(log_power)//2, log_power[i]))  # 居中 bin 索引
    peaks.sort(key=lambda x: -x[1])
    top_peaks = peaks[:15]

    print(f"\n{'='*70}")
    print(f"  {label} [COMPLEX I/Q 解析]")
    print(f"{'='*70}")
    print(f"  I 均值: {real_part.mean():.1f}, Q 均值: {imag_part.mean():.1f}")
    print(f"  噪声底板: {np.median(log_power):.1f} dB, 最大值: {log_power.max():.1f} dB")
    print(f"\n  Top 15 峰值 (centered_bin, power_dB):")
    for bin_idx, pwr in top_peaks:
        # 正频率 bin 对应 0 到 512
        abs_bin = abs(bin_idx)
        r_0146 = abs_bin * 0.1465 * 2  # 2x because half the samples
        print(f"    bin={bin_idx:+5d}  abs_bin={abs_bin:4d}  power={pwr:7.1f} dB  "
              f"(R@0.1465≈{r_0146:.2f}m)")

    return log_power, top_peaks

def analyze_coherent_sum(half0, half1, label=""):
    """分析相干求和后的 range profile (模拟当前 RSP 流水线)"""
    chirps_per_tx = NUM_CHIRPS // 2  # 512

    # TDM 分离
    tx0_h0 = half0[0:chirps_per_tx, :, :]
    tx0_h1 = half1[0:chirps_per_tx, :, :]

    # 相干求和
    sum_h0 = tx0_h0.sum(axis=0)  # (4, 2048)
    sum_h1 = tx0_h1.sum(axis=0)  # (4, 2048)

    # 合并 8 RX
    summed_8rx = np.concatenate([sum_h0, sum_h1], axis=0)  # (8, 2048)

    # 每 RX 做 Range rfft
    win = np.hanning(NUM_SAMPLES).astype(np.float32)
    all_spectra = []
    for rx in range(8):
        chirp_win = summed_8rx[rx] * win
        spec = np.fft.rfft(chirp_win, n=2048)
        all_spectra.append(np.abs(spec)**2)

    power = np.sum(all_spectra, axis=0)
    log_power = 10 * np.log10(power + 1e-10)

    peaks = []
    for i in range(2, len(log_power) - 1):
        if log_power[i] > log_power[i-1] and log_power[i] > log_power[i+1]:
            peaks.append((i, log_power[i]))
    peaks.sort(key=lambda x: -x[1])
    top_peaks = peaks[:20]

    print(f"\n{'='*70}")
    print(f"  {label} [相干求和 + Range FFT — 当前 RSP 流水线]")
    print(f"{'='*70}")
    print(f"  噪声底板: {np.median(log_power):.1f} dB, 最大值: {log_power.max():.1f} dB")
    print(f"\n  Top 20 峰值 (range_bin, power_dB):")
    for bin_idx, pwr in top_peaks:
        r_0146 = bin_idx * 0.1465
        print(f"    bin={bin_idx:5d}  power={pwr:7.1f} dB  R@0.1465={r_0146:8.2f}m")

    return log_power, top_peaks

def main():
    import os
    ctrx0 = os.path.expanduser("/home/zhengyuanliu/Desktop/Orin-ROS/data/ctrx0_raw.bin")
    ctrx1 = os.path.expanduser("/home/zhengyuanliu/Desktop/Orin-ROS/data/ctrx1_raw.bin")

    print("=" * 70)
    print("  RSP 距离估计诊断工具")
    print("=" * 70)
    print(f"  ctrx0: {ctrx0}")
    print(f"  ctrx1: {ctrx1}")

    # 加载第 0 帧
    h0_0, h1_0 = load_frame(ctrx0, 0)
    h0_1, h1_1 = load_frame(ctrx1, 0)

    print(f"\n  数据形状:")
    print(f"    ctrx0 half0: {h0_0.shape}  (chirps={h0_0.shape[0]}, RX={h0_0.shape[1]}, samples={h0_0.shape[2]})")
    print(f"    ctrx0 half1: {h1_0.shape}")
    print(f"    ctrx1 half0: {h0_1.shape}")
    print(f"    ctrx1 half1: {h1_1.shape}")

    # =========================================================================
    # 测试 1: 单个 chirp 的 range profile (用 ctrx0 RX0 第一个 chirp)
    # =========================================================================
    chirp_rx0 = h0_0[0, 0, :]  # chirp 0, RX 0, all 2048 samples
    analyze_chirp(chirp_rx0, "ctrx0 chirp#0 RX#0 — 单个 chirp Range Profile")

    # 测试几个不同 chirp
    chirp_rx0_mid = h0_0[256, 0, :]  # chirp 256, RX 0
    analyze_chirp(chirp_rx0_mid, "ctrx0 chirp#256 RX#0 — 单个 chirp Range Profile")

    # =========================================================================
    # 测试 2: 按 I/Q 交织解析
    # =========================================================================
    analyze_as_complex(chirp_rx0, "ctrx0 chirp#0 RX#0 — I/Q 交织解析")

    # =========================================================================
    # 测试 3: 相干求和后的 range profile (模拟当前 RSP 流水线)
    # =========================================================================
    # 将 ctrx0 和 ctrx1 合并为完整的 8-RX 数据
    full_h0 = h0_0  # (1024, 4, 2048) — ctrx0 的 4 RX
    full_h1 = h0_1  # (1024, 4, 2048) — ctrx1 的 4 RX (这是 4 RX, 需要 reshape)

    # 实际数据布局: ctrx0 和 ctrx1 各自是完整的 (1024, 4, 2048)
    # ctrx0 = RX 0-3, ctrx1 = RX 4-7
    analyze_coherent_sum(full_h0, full_h1, "TX0 chirps [0:512] 相干求和")

    # =========================================================================
    # 测试 4: 不同 range 范围的详细查看
    # =========================================================================
    print(f"\n{'='*70}")
    print(f"  详细: 近场 (bin 1-50) 频谱检查")
    print(f"{'='*70}")

    # 使用相干求和的结果
    chirps_per_tx = 512
    tx0_h0 = h0_0[0:chirps_per_tx, :, :]
    tx0_h1 = h0_1[0:chirps_per_tx, :, :]
    sum_h0 = tx0_h0.sum(axis=0)
    sum_h1 = tx0_h1.sum(axis=0)
    summed_8rx = np.concatenate([sum_h0, sum_h1], axis=0)
    win = np.hanning(2048).astype(np.float32)
    all_power = []
    for rx in range(8):
        spec = np.fft.rfft(summed_8rx[rx] * win, n=2048)
        all_power.append(np.abs(spec)**2)
    power = np.sum(all_power, axis=0)
    log_power = 10 * np.log10(power + 1e-10)

    print(f"  bin  1-10:  {[f'{log_power[i]:.1f}' for i in range(1, 11)]}")
    print(f"  bin 11-20:  {[f'{log_power[i]:.1f}' for i in range(11, 21)]}")
    print(f"  bin 21-30:  {[f'{log_power[i]:.1f}' for i in range(21, 31)]}")
    print(f"  bin 31-40:  {[f'{log_power[i]:.1f}' for i in range(31, 41)]}")
    print(f"  bin 41-50:  {[f'{log_power[i]:.1f}' for i in range(41, 51)]}")
    print(f"  ...")
    print(f"  bin 500-510: {[f'{log_power[i]:.1f}' for i in range(500, 511)]}")
    print(f"  bin 1010-1024: {[f'{log_power[i]:.1f}' for i in range(1010, 1025)]}")

    # =========================================================================
    # 测试 5: 对比有/无相干求和的差异
    # =========================================================================
    print(f"\n{'='*70}")
    print(f"  对比: 相干求和 vs 单 chirp 的 Range FFT")
    print(f"{'='*70}")

    # 单 chirp (不做求和)
    single_chirp_rx0 = tx0_h0[0, 0, :]  # TX0, chirp 0, RX0
    single_chirp_win = single_chirp_rx0 * win
    single_spec = np.fft.rfft(single_chirp_win, n=2048)
    single_power = np.abs(single_spec)**2
    single_log = 10 * np.log10(single_power + 1e-10)

    # 找单 chirp 的峰值
    single_peaks = []
    for i in range(2, len(single_log) - 1):
        if single_log[i] > single_log[i-1] and single_log[i] > single_log[i+1]:
            single_peaks.append((i, single_log[i]))
    single_peaks.sort(key=lambda x: -x[1])

    print(f"  单 chirp (TX0 chirp#0 RX#0) 噪声底板: {np.median(single_log):.1f} dB")
    print(f"  单 chirp Top 10 峰值:")
    for bin_idx, pwr in single_peaks[:10]:
        print(f"    bin={bin_idx:5d}  power={pwr:7.1f} dB  R@0.1465={bin_idx*0.1465:.2f}m")

    print(f"\n  相干求和 (512 chirps) 噪声底板: {np.median(log_power):.1f} dB")
    print(f"  增益: {np.median(log_power) - np.median(single_log):.1f} dB (理论: {10*np.log10(512):.1f} dB)")

if __name__ == '__main__':
    main()
