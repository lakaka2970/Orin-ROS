"""
雷达原始数据预处理模块
包含：DC去除、干扰抑制、距离维加窗及FFT
"""
import numpy as np

# 静态配置：采样点数、接收天线数、Chirp数（可根据实际修改）
_N_SAMPLES = 2048
_N_RX = 16
_N_CHIRPS = 512

# 距离维汉宁窗 (1, 1, n_samples)
WIN_RANGE = np.hanning(_N_SAMPLES).astype(np.float32).reshape(1, 1, -1)


def radar_signal_process_final(
    adc_data_np: np.ndarray,
    n_samples: int,
    n_rx: int,
    n_chirps: int,
    threshold_scale: float = 6.0
) -> tuple:
    """
    雷达信号处理主流程（纯NumPy实现）

    Args:
        adc_data_np: 原始ADC数据，形状 (n_chirps, n_rx, n_samples)
        n_samples: 每个Chirp的采样点数
        n_rx: 接收天线数
        n_chirps: Chirp数量
        threshold_scale: 干扰抑制的阈值缩放系数

    Returns:
        radarcube: 距离FFT结果，形状 (n_rx, n_chirps, range_bins)
        dc_estimated: 估计的直流分量，形状 (n_rx,)
        status: 状态码（0表示成功）
    """
    # 1. 确保数据类型为int16（复数原始数据按需调整）
    adc_data = adc_data_np.astype(np.int16)

    # 2. DC分量估计与去除（逐天线，沿chirp和样本维平均）
    dc_estimated = np.mean(adc_data.astype(np.float32), axis=(0, 2)).astype(np.int16)
    adc_data = adc_data - dc_estimated.reshape(1, n_rx, 1)

    # 3. 干扰抑制：计算相邻样本差值的平均能量作为阈值
    diff = np.abs(adc_data[:, :, 1:] - adc_data[:, :, :-1])        # (chirp, rx, samp-1)
    diff_avg = np.sum(diff, axis=2) // n_samples                   # (chirp, rx)
    threshold = (diff_avg * threshold_scale)[:, :, np.newaxis]     # (chirp, rx, 1)

    # 4. 超过阈值的点置零
    mask = np.abs(adc_data) > threshold
    adc_data = np.where(mask, 0, adc_data)

    # 5. 转换为浮点并加距离维窗
    adc_float = adc_data.astype(np.float32) * WIN_RANGE

    # 6. 距离维FFT（沿样本轴）
    radarcube = np.fft.rfft(adc_float, axis=2)                     # (chirp, rx, range_bins)

    # 7. 转置为 (rx, chirp, range_bins) 以兼容下游
    radarcube = np.transpose(radarcube, (1, 0, 2))

    return radarcube, dc_estimated, 0