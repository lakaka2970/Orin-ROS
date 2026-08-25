"""
合成 FMCW 原始数据生成器（匹配 readRawBinCasc 的 ctrx*.bin 格式）

用途：在无真实 ADC 数据的情况下，生成若干已知距离/速度的点目标，
用于本地运行 CPU 信号处理链，验证其正确性（距离/多普勒估计）并测量分阶段耗时。

信号模型（复基带，去斜后）：
  目标 (rb, db) 在物理 RX p、经 TX t 到达，出现在距离-多普勒谱的
  原始 bin (rb_raw, db_raw)，其中
    rb_raw = rb + tx_sr[t] + rx_sr[p]
    db_raw = db + tx_ddma[t]*16 + tx_sd[t] + rx_sd[p]
  （与 doppler_cpu.py / peak_detection_cpu.py 中的 RX 半阵与 DDMA 半阵偏移保持一致）

写入格式（与 data_io.readRawBinCasc 严格互逆）：
  ctrx0_raw.bin: int16, 4D 数组 (4, Ns, 2, Nc) Fortran 序  → 通道 0..7 (I0-3, Q0-3)
  ctrx1_raw.bin: int16, 4D 数组 (4, Ns, 2, Nc) Fortran 序  → 通道 8..15 (I4-7, Q4-7)
"""
import numpy as np
from config import RadarConfig


def generate_frame(cfg: RadarConfig, targets, noise_std: float, signal_peak: float = 20000.0):
    """
    Args:
        cfg:        RadarConfig 波形参数
        targets:    列表，每项 (rb, db, amp)，rb/db 为距离门/多普勒门，amp 相对幅度
        noise_std:  高斯噪声标准差（int16 单位，加到 I/Q 上）
        signal_peak: 目标合成信号最大幅度（int16 单位，用于缩放避免溢出）
    Returns:
        x_total: (8, Nc, Ns) complex128，8 个物理 RX 的复信号
    """
    Ns = cfg.n_samples
    Nc = cfg.n_chirps
    tx_ddma = np.asarray(cfg.tx_ddma_idx, dtype=np.int64)   # (16,)
    doppler_step = 16  # n_chirps // n_subbands = 512 // 32

    n = np.arange(Ns, dtype=np.float64)[None, :]   # (1, Ns)
    m = np.arange(Nc, dtype=np.float64)[:, None]   # (Nc, 1)

    # TX/RX 半阵偏移（doppler +4, range +1）
    tx_sd = np.where(np.arange(16) >= 8, 4.0, 0.0)   # doppler 偏移
    tx_sr = np.where(np.arange(16) >= 8, 1.0, 0.0)   # range 偏移
    rx_sd = np.where(np.arange(8) >= 4, 4.0, 0.0)
    rx_sr = np.where(np.arange(8) >= 4, 1.0, 0.0)

    x_total = np.zeros((8, Nc, Ns), dtype=np.complex128)

    for (rb, db, amp) in targets:
        rb = float(rb)
        db = float(db)
        # S[m,n] = sum over 16 TX of exp(j2π((db + tx_ddma[t]*16 + tx_sd[t]) m/Nc + (rb + tx_sr[t]) n/Ns))
        S = np.zeros((Nc, Ns), dtype=np.complex128)
        for t in range(16):
            dop = db + tx_ddma[t] * doppler_step + tx_sd[t]
            rng = rb + tx_sr[t]
            phase = (2.0 * np.pi * dop / Nc) * m + (2.0 * np.pi * rng / Ns) * n
            S += np.exp(1j * phase)
        # 每个 RX p： x_total[p] += amp * S * exp(j2π(rx_sd[p] m/Nc + rx_sr[p] n/Ns))
        for p in range(8):
            phase_p = (2.0 * np.pi * rx_sd[p] / Nc) * m + (2.0 * np.pi * rx_sr[p] / Ns) * n
            x_total[p] += amp * S * np.exp(1j * phase_p)

    # 缩放信号峰值，避免 int16 溢出
    peak = np.max(np.abs(x_total))
    if peak > 0:
        x_total = x_total * (signal_peak / peak)
    return x_total


def write_frame(x_total, cfg: RadarConfig, outdir: str, noise_std: float, seed: int = 0):
    """
    将 (8, Nc, Ns) 复信号按 I/Q 拆分为 16 通道，写入 ctrx0_raw.bin / ctrx1_raw.bin。
    """
    Ns = cfg.n_samples
    Nc = cfg.n_chirps
    rng = np.random.default_rng(seed)

    # x_total[p, m, n] -> I[p, n, m], Q[p, n, m]
    I = np.transpose(x_total.real, (0, 2, 1))   # (8, Ns, Nc)
    Q = np.transpose(x_total.imag, (0, 2, 1))   # (8, Ns, Nc)

    # 加噪声
    I = I + rng.normal(0.0, noise_std, size=I.shape)
    Q = Q + rng.normal(0.0, noise_std, size=Q.shape)

    # 量化到 int16
    I = np.clip(np.round(I), -32768, 32767).astype(np.int16)
    Q = np.clip(np.round(Q), -32768, 32767).astype(np.int16)

    # ctrx0: 物理 RX 0..3 (I -> 通道 0..3, Q -> 通道 4..7)
    data0 = np.zeros((4, Ns, 2, Nc), dtype=np.int16)
    data0[:, :, 0, :] = I[0:4]   # I of RX0..3
    data0[:, :, 1, :] = Q[0:4]   # Q of RX0..3
    # ctrx1: 物理 RX 4..7
    data1 = np.zeros((4, Ns, 2, Nc), dtype=np.int16)
    data1[:, :, 0, :] = I[4:8]
    data1[:, :, 1, :] = Q[4:8]

    import os
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "ctrx0_raw.bin"), "wb") as f:
        f.write(data0.reshape(-1, order='F').tobytes())
    with open(os.path.join(outdir, "ctrx1_raw.bin"), "wb") as f:
        f.write(data1.reshape(-1, order='F').tobytes())

    nbytes = data0.nbytes
    print(f"[gen] ctrx0_raw.bin / ctrx1_raw.bin 各 {nbytes} 字节, 帧尺寸符合 {4*Ns*2*Nc*2} 字节")


if __name__ == "__main__":
    cfg = RadarConfig()
    # 4 个已知点目标：(rb, db, amp)，对应 R = rb*0.3593 m, v = db*0.0823 m/s
    targets = [
        (80,  100, 1.0),   # R≈28.7m  v≈8.23 m/s
        (139, 130, 1.0),   # R≈49.9m  v≈10.70 m/s
        (139, 210, 0.7),   # R≈49.9m  v≈17.28 m/s
        (250, 200, 0.5),   # R≈89.8m  v≈16.46 m/s
    ]
    x_total = generate_frame(cfg, targets, noise_std=4000.0, signal_peak=20000.0)
    write_frame(x_total, cfg, ".", noise_std=4000.0, seed=42)
    print("[gen] 完成。目标真值 (rb, db):", [(t[0], t[1]) for t in targets])
    print("[gen] R =", [round(t[0]*cfg.range_resolution, 2) for t in targets], "m")
    print("[gen] v =", [round(t[1]*cfg.doppler_resolution, 2) for t in targets], "m/s")
