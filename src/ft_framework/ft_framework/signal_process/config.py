"""
雷达系统参数配置模块：包含波形参数、信号处理阈值、绘图选项等。
"""

import numpy as np

C0 = 299792458.0          # 光速 [m/s]


class RadarConfig:
    """雷达系统与信号处理参数（集中管理，便于调优）"""

    # ----- 波形参数 -----
    n_samples = 2048          # 每个 Chirp 的采样点数
    n_chirps = 512            # 一帧内的 Chirp 数量
    n_rx = 16                 # 接收天线数
    n_subbands = 32           # DDMA 子带数（多普勒域分割数）

    # ----- 处理参数 -----
    threshold_scale = 8       # 干扰检测门限系数（相对于差分均值）
    noise_est_ratio = 50      # 噪声估计百分位数（例如 50% 中位数）
    ps_scale = 19.0           # 峰值搜索门限缩放因子（倍乘噪声基底）
    max_peaks_per_rb = 12     # 每个距离门最多保留的峰值数
    max_total_peaks = 1024    # 全局最多保留的峰值总数

    # DDMA 发射天线索引（前8个master CTRX0，后8个slave CTRX1）
    tx_ddma_idx = np.array([
        0,  1,  2,  3,  4,  9, 10, 12,       # master  (phase_deg / 11.25)
        14, 16, 19, 23, 24, 27, 29, 30        # slave
    ], dtype=np.int64)

    # ----- 波形具体参数（需与硬件/固件一致）-----
    freq_start_hz = 77.5e9                       # 起始频率 [Hz]
    freq_slope_hz_per_s = 10.18524e12            # 调频斜率 [Hz/s]
    time_ramp_end_s = 40.96e-6                  # 有效调频时间 [s]
    time_idle_s = 4.94e-6                       # Chirp 空闲时间 [s]
    step_time = 4                               #跳频倍数

    # 派生参数（自动计算）
    bandwidth_hz = freq_slope_hz_per_s * time_ramp_end_s          # 带宽 [Hz]
    wavelength = C0 / freq_start_hz                               # 波长 [m]
    chirp_interval_s = time_ramp_end_s + time_idle_s              # Chirp 周期 [s]

    # 距离/速度分辨率与最大不模糊速度
    range_resolution = C0 / (2 * bandwidth_hz)                    # 距离分辨率 [m]
    doppler_resolution = wavelength / (2 * n_chirps * chirp_interval_s)  # 多普勒分辨率 [m/s]
    ambgt = wavelength / (2 * chirp_interval_s)                   # 最大不模糊速度 [m/s]

    # ----- 绘图开关（调试/可视化用）-----
    enable_plots = False
    plot_range_profile = True
    plot_range_chirp = True
    plot_rd_cube = True
    plot_rx_nci = True
    plot_noise = True
    plot_vch_nci = True
    plot_max_subband = True
    plot_peaks = True
    save_plots = False
    plot_save_dir = "./plots"