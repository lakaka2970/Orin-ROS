"""
雷达信号预处理模块：包含去直流、时域干扰抑制、汉宁窗加窗及距离维 FFT。
全程 GPU 加速，使用预分配窗函数和常量。
"""

import torch
import numpy as np

DEVICE = torch.device('cuda')

# 静态参数（与 RadarConfig 保持一致）
_n_samples = 2048
_n_rx = 16
_n_chirps = 512

# 距离维汉宁窗（预分配到 GPU）
WIN_RANGE_GPU = torch.hann_window(_n_samples, periodic=False, dtype=torch.float32, device=DEVICE)[None, None, :]

# 常量零（用于干扰抑制时的掩码填充）
ZERO_INT16 = torch.tensor(0, dtype=torch.int16, device=DEVICE)


@torch.inference_mode()
def radar_signal_process_final(adc_data_np: np.ndarray, n_samples: int, n_rx: int,
                               n_chirps: int, threshold_scale: int = 6):
    """
    雷达信号预处理主函数。

    参数:
        adc_data_np: 原始 ADC 数据，numpy 数组，形状 (n_chirps, n_rx, n_samples)，数据类型 int16
        n_samples: 采样点数
        n_rx: 接收天线数
        n_chirps: Chirp 数量
        threshold_scale: 干扰检测门限系数（倍数）

    返回:
        radarcube: 距离 FFT 后的数据立方，形状 (n_rx, n_chirps, n_range_bins)，复数
        dc_estimated: 估计的直流分量 (n_rx,)
        status: 状态码（0 表示正常）
    """
    # ----- 1. 数据转移到 GPU（非阻塞异步传输）-----
    adc_data = torch.from_numpy(adc_data_np).to(DEVICE, non_blocking=True)   # (chirp, rx, samp)

    # ----- 2. 去直流：计算每个接收天线在所有 chirp 和采样点上的均值，然后减去-----
    dc_estimated = torch.mean(adc_data.to(torch.float32), dim=(0, 2)).to(torch.int16)  # (rx,)
    adc_data = adc_data - dc_estimated[None, :, None]   # 广播减去直流

    # ----- 3. 时域干扰抑制：检测相邻采样点突变-----
    # 差分绝对值
    diff = torch.abs(adc_data[:, :, 1:] - adc_data[:, :, :-1])          # (chirp, rx, samp-1)
    diff_ave = torch.sum(diff, dim=2) // n_samples                      # (chirp, rx)
    threshold = (diff_ave * threshold_scale)[:, :, None]                # (chirp, rx, 1)
    # 超过阈值的采样点置零
    mask = adc_data > threshold
    #adc_data = torch.where(mask, ZERO_INT16, adc_data)

    # ----- 4. 转换为浮点并加窗 -----
    adc_float = adc_data.to(torch.float32) * WIN_RANGE_GPU              # (chirp, rx, samp)

    # ----- 5. 距离维 FFT（实信号 -> 单边谱）-----
    radarcube = torch.fft.rfft(adc_float, dim=2)                        # (chirp, rx, range_bins)
    # 转置为 (rx, chirp, range_bins) 以兼容后续多普勒处理
    radarcube = radarcube.permute(1, 0, 2)

    return radarcube, dc_estimated, 0