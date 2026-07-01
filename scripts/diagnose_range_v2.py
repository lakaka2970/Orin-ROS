#!/usr/bin/env python3
"""
诊断脚本 v2: 验证 TX-RX 泄漏谐波假说 + 寻找真实近场目标信号
"""

import numpy as np

NUM_CHIRPS = 1024
NUM_RX = 8
NUM_SAMPLES = 2048
RX_PER_HALF = 4
FRAME_BYTES = NUM_CHIRPS * NUM_RX * NUM_SAMPLES * 2

def load_frame(filepath, frame_idx=0):
    offset = frame_idx * FRAME_BYTES
    with open(filepath, 'rb') as f:
        f.seek(offset)
        data = f.read(FRAME_BYTES)
    arr = np.frombuffer(data, dtype=np.int16).astype(np.float32)
    half_elems = len(arr) // 2
    half0 = arr[:half_elems].reshape(NUM_CHIRPS, RX_PER_HALF, NUM_SAMPLES)
    half1 = arr[half_elems:2*half_elems].reshape(NUM_CHIRPS, RX_PER_HALF, NUM_SAMPLES)
    return half0, half1

def main():
    import os
    ctrx0 = os.path.expanduser("/home/zhengyuanliu/Desktop/Orin-ROS/data/ctrx0_raw.bin")
    ctrx1 = os.path.expanduser("/home/zhengyuanliu/Desktop/Orin-ROS/data/ctrx1_raw.bin")

    h0_0, h1_0 = load_frame(ctrx0, 0)
    h0_1, h1_1 = load_frame(ctrx1, 0)

    # 合并为完整 8-RX 数据: ctrx0 half0 (RX0-3), ctrx0 half1???
    # 实际: ctrx0 有 2 个 half, 每个 half 是 4 RX.
    # 需要确认 ctrx0 和 ctrx1 的关系.

    # 先单看 ctrx0 的 half0, chirp#0, RX#0
    chirp = h0_0[0, 0, :].copy()
    chirp_dc = chirp - chirp.mean()

    win = np.hanning(2048).astype(np.float32)
    spec = np.fft.rfft(chirp_dc * win, n=2048)
    power = np.abs(spec) ** 2
    log_power = 10 * np.log10(power + 1e-10)

    print("=" * 70)
    print("  谐波假说验证")
    print("=" * 70)

    # 找基频 (TX 泄漏)
    # 检查 bin 2, 3, 4, 5, 6... 哪个是泄漏的基频
    print("\n  近场 bins 详细功率 (bin 1-20):")
    for i in range(1, 21):
        marker = ""
        if i <= 10:
            # 检查是否是主要泄漏
            if log_power[i] > 120:
                marker = " ← 强泄漏"
        print(f"    bin {i:4d}: {log_power[i]:7.1f} dB  (R@0.1465={i*0.1465:.3f}m){marker}")

    # 谐波分析
    print("\n  谐波分析 (检查 bin 508, 1020 是否是近场 bin 的整数倍):")
    for base_bin in [1, 2, 3, 4]:
        for mult in [127, 254, 255]:
            harmonic_bin = base_bin * mult
            if harmonic_bin < 1025:
                rfft_bin = harmonic_bin if harmonic_bin <= 1024 else 2048 - harmonic_bin
                if rfft_bin < len(log_power):
                    print(f"    base_bin={base_bin} × {mult:3d} = {harmonic_bin:4d} → power={log_power[rfft_bin]:7.1f} dB")

    # 检查 bin 508 和 1020 的精确关系
    print(f"\n  峰值关系:")
    print(f"    bin 508  power = {log_power[508]:.1f} dB")
    print(f"    bin 1020 power = {log_power[1020]:.1f} dB")
    print(f"    bin 1020 / bin 508 = {1020/508:.3f}")
    print(f"    508 的约数: {[i for i in range(1, 21) if 508 % i == 0]}")
    print(f"    1020 的约数: {[i for i in range(1, 21) if 1020 % i == 0]}")

    # =========================================================================
    # 关键测试: 如果我们在时域对 chirp 做低通滤波，去除低频泄漏，谐波会消失吗？
    # =========================================================================
    print("\n" + "=" * 70)
    print("  测试: 高通滤波去除 TX 泄漏后，谐波是否消失")
    print("=" * 70)

    # 方法: 在时域减去低频成分 (模拟高通滤波)
    from numpy.fft import rfft, irfft

    # 频域陷波: 将 bin 0-10 置零
    spec_filtered = spec.copy()
    spec_filtered[0:15] = 0  # 滤除 DC + 近场泄漏

    # 重建时域信号
    chirp_filtered = irfft(spec_filtered, n=2048)

    # 重新做 FFT
    spec2 = rfft(chirp_filtered * win, n=2048)
    power2 = np.abs(spec2) ** 2
    log_power2 = 10 * np.log10(power2 + 1e-10)

    print(f"  滤波后 bin 508  power = {log_power2[508]:.1f} dB (原 {log_power[508]:.1f} dB)")
    print(f"  滤波后 bin 1020 power = {log_power2[1020]:.1f} dB (原 {log_power[1020]:.1f} dB)")

    # 如果滤波后 bin 508 和 1020 的功率大幅下降 → 确认是泄漏谐波
    # 如果基本不变 → 是独立的真实信号

    delta_508 = log_power[508] - log_power2[508]
    delta_1020 = log_power[1020] - log_power2[1020]

    if delta_508 > 10:
        print(f"\n  ★ bin 508 功率下降 {delta_508:.1f} dB → 确认是 TX 泄漏的谐波!")
    else:
        print(f"\n  ★ bin 508 功率仅下降 {delta_508:.1f} dB → 可能是独立信号")

    # =========================================================================
    # 测试: 真实目标应该在什么 bin?
    # 对于 3-5m 暗室角反射器, 理论 beat frequency:
    # f_beat = 2*R*B/(c*T_chirp) = 2*R*S/c
    # 若 B=1GHz, T_chirp=50us → S=2e13 Hz/s
    # f_beat(3m) = 2*3*2e13/3e8 = 400 kHz
    # bin = f_beat * T_chirp * N_fft / N_samples = f_beat * N / fs
    # 若 fs=40.96MHz → bin = 400e3 * 2048 / 40.96e6 = 20
    # =========================================================================

    print("\n" + "=" * 70)
    print("  寻找真实目标 (3-5m 对应 bins 10-50)")
    print("=" * 70)

    # 在滤波后的频谱中寻找
    peaks_filtered = []
    for i in range(6, 100):  # bins 6-100, 跳过 TX 泄漏区
        if log_power2[i] > log_power2[i-1] and log_power2[i] > log_power2[i+1]:
            bg = np.median(log_power2[max(0,i-20):i].tolist() + log_power2[i+1:min(len(log_power2),i+21)].tolist())
            snr = log_power2[i] - bg
            if snr > 3:
                peaks_filtered.append((i, log_power2[i], snr))
    peaks_filtered.sort(key=lambda x: -x[1])

    print(f"  滤波后 bins 6-100 中的峰值 (SNR > 3dB):")
    for bin_idx, pwr, snr in peaks_filtered[:15]:
        r_0146 = bin_idx * 0.1465
        r_001 = bin_idx * 0.01
        print(f"    bin={bin_idx:4d}  power={pwr:7.1f} dB  SNR={snr:.1f} dB  "
              f"(R@0.1465={r_0146:.2f}m, R@0.01={r_001:.2f}m)")

    # =========================================================================
    # 对比 ctrx0 和 ctrx1: 检查异构
    # =========================================================================
    print("\n" + "=" * 70)
    print("  ctrx0 vs ctrx1 对比 (chirp#0, RX#0)")
    print("=" * 70)

    chirp_ctrx1 = h0_1[0, 0, :].copy()
    chirp_ctrx1_dc = chirp_ctrx1 - chirp_ctrx1.mean()
    spec_ctrx1 = rfft(chirp_ctrx1_dc * win, n=2048)
    log_ctrx1 = 10 * np.log10(np.abs(spec_ctrx1)**2 + 1e-10)

    print(f"  ctrx0 bin 508: {log_power[508]:.1f} dB")
    print(f"  ctrx1 bin 508: {log_ctrx1[508]:.1f} dB")
    print(f"  ctrx0 bin 4:   {log_power[4]:.1f} dB")
    print(f"  ctrx1 bin 4:   {log_ctrx1[4]:.1f} dB")
    print(f"  ctrx0 噪声底板: {np.median(log_power):.1f} dB")
    print(f"  ctrx1 噪声底板: {np.median(log_ctrx1):.1f} dB")

    # 相关性检查
    corr = np.corrcoef(log_power[1:100], log_ctrx1[1:100])[0,1]
    print(f"  近场频谱相关系数 (bin 1-100): {corr:.4f}")

    # =========================================================================
    # 重要: 直接检查时间域信号 - 看是否有明显周期
    # =========================================================================
    print("\n" + "=" * 70)
    print("  时域信号分析")
    print("=" * 70)

    # 去直流后看信号
    sig = chirp_dc
    # 自相关找基频周期
    autocorr = np.correlate(sig[:512], sig[:512], mode='same')
    autocorr_center = len(autocorr) // 2
    # 找自相关的峰值 (排除 0 lag)
    ac_peaks = []
    for i in range(autocorr_center + 2, len(autocorr)):
        if autocorr[i] > autocorr[i-1] and autocorr[i] > autocorr[i+1]:
            ac_peaks.append((i - autocorr_center, autocorr[i]))
    ac_peaks.sort(key=lambda x: -x[1])

    print(f"  信号幅度: min={sig.min():.0f}, max={sig.max():.0f}, std={sig.std():.0f}")
    print(f"  自相关前 5 峰值 (lag, corr):")
    for lag, corr_val in ac_peaks[:5]:
        if lag > 0:
            freq_est = 2048 / lag  # 周期对应的谐波 bin
            print(f"    lag={lag:4d} samples  corr={corr_val:.0f}  → 基频约 bin {freq_est:.0f}")

    # =========================================================================
    # 跨多个 chirp 检查 bin 508 的一致性
    # =========================================================================
    print("\n" + "=" * 70)
    print("  跨 chirp 一致性检查 (ctrx0 half0, RX0)")
    print("=" * 70)

    bin508_powers = []
    bin4_powers = []
    bin20_powers = []
    for c in range(0, 512, 64):  # 每 64 chirp 采样
        ch = h0_0[c, 0, :] - h0_0[c, 0, :].mean()
        sp = rfft(ch * win, n=2048)
        pw = np.abs(sp)**2
        bin508_powers.append(10*np.log10(pw[508] + 1e-10))
        bin4_powers.append(10*np.log10(pw[4] + 1e-10))
        bin20_powers.append(10*np.log10(pw[20] + 1e-10))

    print(f"  bin 508 功率: mean={np.mean(bin508_powers):.1f}, std={np.std(bin508_powers):.1f} dB")
    print(f"  bin 4   功率: mean={np.mean(bin4_powers):.1f}, std={np.std(bin4_powers):.1f} dB")
    print(f"  bin 20  功率: mean={np.mean(bin20_powers):.1f}, std={np.std(bin20_powers):.1f} dB")

if __name__ == '__main__':
    main()
