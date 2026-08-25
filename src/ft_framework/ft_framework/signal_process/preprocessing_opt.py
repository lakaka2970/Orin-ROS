"""
雷达信号预处理模块（优化版）：去直流、汉宁窗加窗及距离维 FFT。

【策略变更记录】
  - 原版 radar_signal_process_final 中的"时域干扰抑制"整段（diff/sum/threshold/mask）
    计算结果 mask 从未被使用（后续 `torch.where(mask, ...)` 被注释掉）。
    profile_hotspots.py 实测该段耗时 ~35 ms，占 preprocess 一半以上，属纯死代码。
  - 本版删除该段及其依赖的 ZERO_INT16 常量，其余逻辑与原版逐字一致，数值结果不变。
"""
import torch
import numpy as np

DEVICE = torch.device('cuda')

# 距离维汉宁窗（预分配到 GPU）
WIN_RANGE_GPU = torch.hann_window(2048, periodic=False, dtype=torch.float32, device=DEVICE)[None, None, :]


@torch.inference_mode()
def radar_signal_process_final(adc_data_input, n_samples: int, n_rx: int,
                               n_chirps: int, threshold_scale: int = 6):
    """去直流 -> 加窗 -> 距离维 FFT。与原版相比仅移除死代码干扰抑制段。"""
    if isinstance(adc_data_input, torch.Tensor):
        adc_data = adc_data_input.to(DEVICE, non_blocking=True)
    else:
        adc_data = torch.from_numpy(adc_data_input).to(DEVICE, non_blocking=True)
    adc_float_all = adc_data.float() if adc_data.dtype != torch.float32 else adc_data

    # 去直流
    dc_estimated = torch.mean(adc_float_all, dim=(0, 2))
    adc_data = adc_float_all - dc_estimated[None, :, None]

    # 加窗
    adc_float = adc_data.to(torch.float32) * WIN_RANGE_GPU

    # 距离维 FFT
    radarcube = torch.fft.rfft(adc_float, dim=2)
    radarcube = radarcube.permute(1, 0, 2)

    return radarcube, dc_estimated, 0
