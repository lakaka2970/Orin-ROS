"""
DOA 估计模块：利用方位/俯仰分离子阵，通过 FFT 与峰值插值获得精确角度。
实现完全 GPU 化，包含环形峰值检索与二次插值。
"""

import torch
import numpy as np
import matplotlib.pyplot as plt
from ft_framework.signal_process.multTarget import MultiTargetEVT_GPU
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


class DOA_Ultra_Initializer:
    """
    DOA 处理环境初始化器（全局单例）。
    预先分配汉明窗、锁页内存，并缓存阵列映射索引。
    """

    def __init__(self, fft_len: int = 256, max_targets: int = 2):
        """
        参数:
            fft_len: 角度 FFT 点数（映射到 sin(θ) 域的网格数）
            max_targets: 每维度最多检测的目标数
        """
        self.fft_len = fft_len
        self.max_targets = max_targets

        # 汉明窗（实数，固化在显存）
        self.window_gpu = torch.hamming_window(fft_len, periodic=False, dtype=torch.float32, device=DEVICE)

        # 锁页内存（pinned memory）用于异步拷贝结果，减少 CPU-GPU 传输延迟
        self.azi_buffer_pinned = torch.empty((max_targets, 5), dtype=torch.float32, pin_memory=True)
        self.ele_buffer_pinned = torch.empty((max_targets, 5), dtype=torch.float32, pin_memory=True)

        self.is_initialized = False

    def prepare_mapping_indices(self, Array_Azi, Array_Ele):
        """
        预计算方位/俯仰子阵元素到 FFT 网格的映射索引（线性映射，基于最小/最大值）。
        此方法需在首次 DOA 处理前调用一次。
        """
        def get_map_indices_gpu(positions):
            """将一维阵元位置线性映射到 [0, fft_len-1] 的整数索引。"""
            if not isinstance(positions, torch.Tensor):
                positions = torch.from_numpy(positions).to(DEVICE)
            else:
                positions = positions.to(DEVICE)
            if positions.numel() == 0:
                return torch.tensor([], dtype=torch.int64, device=DEVICE)
            min_pos, max_pos = torch.min(positions), torch.max(positions)
            if max_pos == min_pos:
                return torch.full_like(positions, self.fft_len // 2, dtype=torch.int64)
            return ((positions - min_pos) / (max_pos - min_pos) * (self.fft_len - 1)).to(torch.int64)

        self.azi_indices = get_map_indices_gpu(Array_Azi[0, :])   # 方位子阵 x 坐标
        self.ele_indices = get_map_indices_gpu(Array_Ele[1, :])   # 俯仰子阵 y 坐标
        self.is_initialized = True


# 全局 DOA 环境（单例）
doa_env = DOA_Ultra_Initializer(fft_len=256, max_targets=2)


@torch.inference_mode()
def doa_main_ultra_separated(snap_data, Array_Azi, AziIdx_Select, Array_Ele, EleIdx_Select, threshold_db=6.0):
    """
    单快拍 DOA 估计（方位与俯仰独立解算，不进行交叉配对）。

    参数:
        snap_data: 复基带数据，形状 (N_virtual,) 或 (N_total,)
        Array_Azi: 方位子阵坐标 (3, n_azi)
        AziIdx_Select: 方位子阵在虚拟阵列中的索引
        Array_Ele: 俯仰子阵坐标 (3, n_ele)
        EleIdx_Select: 俯仰子阵在虚拟阵列中的索引
        threshold_db: 峰值检测门限（相对于最大峰值，单位 dB）

    返回:
        azi_results: NumPy 数组，形状 (n_azi_detected, 5)，每行 [valid_flag, bin_idx, power_dB, interp_bin, angle_deg]
        ele_results: NumPy 数组，形状 (n_ele_detected, 5)
    """
    global doa_env
    if not doa_env.is_initialized:
        # 第一次调用时计算映射索引
        doa_env.prepare_mapping_indices(Array_Azi, Array_Ele)

    fft_len = doa_env.fft_len

    # ----- 提取子阵数据并映射到FFT谱 -----
    src_azi = snap_data[AziIdx_Select].to(dtype=torch.complex64)
    src_ele = snap_data[EleIdx_Select].to(dtype=torch.complex64)

    # 方位子阵FFT谱构造
    spec_azi = torch.zeros(fft_len, dtype=torch.complex64, device=DEVICE)
    # PyTorch 的 clip 对应 clamp，索引推荐使用 torch.int64 (LongTensor)
    azi_indices = torch.clamp(Array_Azi[0, :].to(torch.int64), 0, fft_len - 1)
    spec_azi[azi_indices] = src_azi

    # 使用 torch.fft 模块，注意 PyTorch 的 fft 默认行为与 numpy 一致
    fft_azi = torch.fft.fft(spec_azi * doa_env.window_gpu)
    mag_azi_db = 20.0 * torch.log10(torch.abs(fft_azi) + 1e-12)

    # 俯仰子阵FFT谱构造
    spec_ele = torch.zeros(fft_len, dtype=torch.complex64, device=DEVICE)
    ele_indices = torch.clamp(Array_Ele[1, :].to(torch.int64), 0, fft_len - 1)
    spec_ele[ele_indices] = src_ele

    fft_ele = torch.fft.fft(spec_ele * doa_env.window_gpu)
    mag_ele_db = 20.0 * torch.log10(torch.abs(fft_ele) + 1e-12)


    # （可选绘图，默认关闭）
    if False:
        azi_np = mag_azi_db.cpu().numpy()
        ele_np = mag_ele_db.cpu().numpy()
        plt.figure()
        plt.subplot(1,2,1); plt.plot(azi_np); plt.title("Azimuth Spectrum")
        plt.subplot(1,2,2); plt.plot(ele_np); plt.title("Elevation Spectrum")
        plt.show()

    # ----- 4. 环形峰值检索（一维）-----
    # 方位轴环形比较（左右邻域）
    p_left_a = torch.roll(mag_azi_db, 1, 0)
    p_right_a = torch.roll(mag_azi_db, -1, 0)
    p_left_a[0], p_right_a[-1] = mag_azi_db[0], mag_azi_db[-1]   # 边界修正
    mask_a = (mag_azi_db >= p_left_a) & (mag_azi_db >= p_right_a) & \
             (mag_azi_db >= (torch.max(mag_azi_db) - threshold_db))
    idx_a = torch.where(mask_a)[0]

    # 俯仰轴类似
    p_left_e = torch.roll(mag_ele_db, 1, 0)
    p_right_e = torch.roll(mag_ele_db, -1, 0)
    p_left_e[0], p_right_e[-1] = mag_ele_db[0], mag_ele_db[-1]
    mask_e = (mag_ele_db >= p_left_e) & (mag_ele_db >= p_right_e) & \
             (mag_ele_db >= (torch.max(mag_ele_db) - threshold_db))
    idx_e = torch.where(mask_e)[0]

    # 限制最大目标数量
    n_azi = min(idx_a.numel(), doa_env.max_targets)
    n_ele = min(idx_e.numel(), doa_env.max_targets)

    res_azi_np = np.zeros((0, 5), dtype=np.float32)
    res_ele_np = np.zeros((0, 5), dtype=np.float32)

    # ----- 5. 二次插值（抛物线拟合）与角度解算 -----
    if n_azi > 0:
        idx_c = torch.clamp(idx_a[:n_azi], 1, fft_len - 2)               # 确保有左右邻域
        denom = mag_azi_db[idx_c - 1] - 2.0 * mag_azi_db[idx_c] + mag_azi_db[idx_c + 1]
        offset = torch.where(denom != 0,
                             (mag_azi_db[idx_c - 1] - mag_azi_db[idx_c + 1]) / (2.0 * denom),
                             0.0)
        interp_bin = idx_a[:n_azi].to(torch.float32) + torch.clamp(offset, -0.5, 0.5)

        # 物理角度： sinθ = (2 * shift) / fft_len, shift 以 fft_len/2 为界正负
        shift = torch.where(interp_bin < (fft_len / 2), interp_bin, interp_bin - fft_len)
        sin_theta = torch.clamp((2.0 * shift) / fft_len, -1.0, 1.0)
        azi_deg = torch.rad2deg(torch.asin(sin_theta))

        # 组装结果 [valid, bin_idx, power, interp_bin, angle_deg]
        res_azi = torch.stack([
            torch.ones(n_azi, device=DEVICE),
            idx_a[:n_azi].to(torch.float32),
            mag_azi_db[idx_a[:n_azi]],
            interp_bin,
            azi_deg
        ], dim=1)
        doa_env.azi_buffer_pinned[:n_azi].copy_(res_azi, non_blocking=True)
        res_azi_np = doa_env.azi_buffer_pinned[:n_azi].numpy()

    if n_ele > 0:
        idx_c = torch.clamp(idx_e[:n_ele], 1, fft_len - 2)
        denom = mag_ele_db[idx_c - 1] - 2.0 * mag_ele_db[idx_c] + mag_ele_db[idx_c + 1]
        offset = torch.where(denom != 0,
                             (mag_ele_db[idx_c - 1] - mag_ele_db[idx_c + 1]) / (2.0 * denom),
                             0.0)
        interp_bin = idx_e[:n_ele].to(torch.float32) + torch.clamp(offset, -0.5, 0.5)

        shift = torch.where(interp_bin < (fft_len / 2), interp_bin, interp_bin - fft_len)
        sin_theta = torch.clamp((2.0 * shift) / fft_len, -1.0, 1.0)
        ele_deg = torch.rad2deg(torch.asin(sin_theta))

        res_ele = torch.stack([
            torch.ones(n_ele, device=DEVICE),
            idx_e[:n_ele].to(torch.float32),
            mag_ele_db[idx_e[:n_ele]],
            interp_bin,
            ele_deg
        ], dim=1)
        doa_env.ele_buffer_pinned[:n_ele].copy_(res_ele, non_blocking=True)
        res_ele_np = doa_env.ele_buffer_pinned[:n_ele].numpy()

    if False:
        azm_bin_interp = res_azi[0, 3]                     # 插值后的bin (Tensor)
        two_pi_div_nfft = 2 * np.pi / doa_env.fft_len       # 标量常数
        phase_input = two_pi_div_nfft * azm_bin_interp

        # 1. 计算通道偏移（全 GPU 算子）
        azi_positions = Array_Azi[0, :]
        diffs = torch.diff(azi_positions)

        # 用 torch.cat 拼接，替代 np.concatenate
        zero_padding = torch.tensor([0.0], dtype=torch.float32, device=DEVICE)
        channel_offsets = torch.cat((zero_padding, diffs)).to(dtype=torch.float32)

        # 2. 实例化 GPU 检测器 (高频调用时，建议把实例化移到外层循环外以极大提升速度)
        evt_detector = MultiTargetEVT_GPU(
            channel_offsets=channel_offsets,
            lambda_coeff=-np.pi,
            nof_mbf_channel=len(AziIdx_Select),
            device='cuda'
        )

        # 3. 提取子阵数据
        ch_data = snap_data[AziIdx_Select].to(dtype=torch.complex64)

        # 4. 运行 GPU 判决
        is_multi, db_val, _, _ = evt_detector.is_multi_target(
            azm_bin_in=phase_input,
            ch_data_in=ch_data,
            is_moving=False
        )

    return res_azi_np, res_ele_np