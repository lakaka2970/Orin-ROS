"""
DOA（到达角）估计模块
基于FFT波束形成，分离方位和俯仰子阵处理
"""
import numpy as np
import matplotlib.pyplot as plt
from ft_framework.signal_process.multTarget_cpu import MultiTargetEVT


class DOAUltraInitializer:
    """DOA处理器初始化：准备FFT窗和子阵索引映射"""

    def __init__(self, fft_len: int = 256, max_targets: int = 2):
        self.fft_len = fft_len
        self.max_targets = max_targets
        self.window = np.hamming(fft_len).astype(np.float32)
        self.is_initialized = False

    def prepare_mapping_indices(self, Array_Azi: np.ndarray, Array_Ele: np.ndarray) -> None:
        """
        根据子阵位置计算FFT索引映射
        Args:
            Array_Azi: 方位子阵坐标 [3, n_azi]
            Array_Ele: 俯仰子阵坐标 [3, n_ele]
        """
        def get_indices(positions: np.ndarray) -> np.ndarray:
            if positions.size == 0:
                return np.array([], dtype=np.int64)
            min_pos, max_pos = positions.min(), positions.max()
            if max_pos == min_pos:
                return np.full_like(positions, self.fft_len // 2, dtype=np.int64)
            return ((positions - min_pos) / (max_pos - min_pos) * (self.fft_len - 1)).astype(np.int64)

        self.azi_indices = get_indices(Array_Azi[0, :])   # 方位子阵使用x坐标
        self.ele_indices = get_indices(Array_Ele[1, :])   # 俯仰子阵使用y坐标
        self.is_initialized = True


# 全局DOA环境（单例）
doa_env = DOAUltraInitializer(fft_len=256, max_targets=2)


def doa_main_ultra_separated(
    snap_data: np.ndarray,
    Array_Azi: np.ndarray,
    AziIdx_Select: np.ndarray,
    Array_Ele: np.ndarray,
    EleIdx_Select: np.ndarray,
    threshold_db: float = 6.0
) -> tuple:
    """
    分离的方位/俯仰DOA估计

    Args:
        snap_data: 单快拍复数数据，形状 (N_total_elements,)
        Array_Azi: 方位子阵坐标 [3, n_azi]
        AziIdx_Select: 方位子阵在原阵列中的索引
        Array_Ele: 俯仰子阵坐标 [3, n_ele]
        EleIdx_Select: 俯仰子阵在原阵列中的索引
        threshold_db: 峰值检测的相对门限(dB)

    Returns:
        res_azi_np: 方位结果数组，每行 [flag, bin_idx, mag_db, interp_bin, angle_deg]
        res_ele_np: 俯仰结果数组，格式同上
    """
    global doa_env
    if not doa_env.is_initialized:
        doa_env.prepare_mapping_indices(Array_Azi, Array_Ele)

    fft_len = doa_env.fft_len

    # ----- 提取子阵数据并映射到FFT谱 -----
    src_azi = snap_data[AziIdx_Select].astype(np.complex64)
    src_ele = snap_data[EleIdx_Select].astype(np.complex64)

    # 方位子阵FFT谱构造
    spec_azi = np.zeros(fft_len, dtype=np.complex64)
    azi_indices = np.clip(Array_Azi[0, :].astype(np.int64), 0, fft_len - 1)
    spec_azi[azi_indices] = src_azi
    fft_azi = np.fft.fft(spec_azi * doa_env.window)
    mag_azi_db = 20.0 * np.log10(np.abs(fft_azi) + 1e-12)

    # 俯仰子阵FFT谱构造
    spec_ele = np.zeros(fft_len, dtype=np.complex64)
    ele_indices = np.clip(Array_Ele[1, :].astype(np.int64), 0, fft_len - 1)
    spec_ele[ele_indices] = src_ele
    fft_ele = np.fft.fft(spec_ele * doa_env.window)
    mag_ele_db = 20.0 * np.log10(np.abs(fft_ele) + 1e-12)

    # 可选绘图
    if False:   # 改为False避免干扰，如需绘图置True
        plt.figure(figsize=(12, 5))
        plt.subplot(1, 2, 1); plt.plot(mag_azi_db); plt.title('Azimuth Spectrum'); plt.grid(True)
        plt.subplot(1, 2, 2); plt.plot(mag_ele_db); plt.title('Elevation Spectrum'); plt.grid(True)
        plt.show()

    # ----- 峰值检测（环形邻域）-----
    def find_peaks(mag: np.ndarray) -> np.ndarray:
        left = np.roll(mag, 1); left[0] = mag[0]
        right = np.roll(mag, -1); right[-1] = mag[-1]
        mask = (mag >= left) & (mag >= right) & (mag >= (mag.max() - threshold_db))
        return np.where(mask)[0]

    def sin_snr_lin(mag: np.ndarray, idx: np.ndarray) -> np.uint16:
        """最强/次强峰值比 (dB) * 1000, clamp 到 uint16"""
        if len(idx) < 2:
            return np.uint16(0)
        peaks = mag[idx]
        top2 = np.sort(peaks)[-2:]  # 升序取最后2个
        diff_db = top2[1] - top2[0]
        return np.uint16(np.clip(diff_db * 1000.0, 0, 65535))

    idx_a = find_peaks(mag_azi_db)
    idx_e = find_peaks(mag_ele_db)
    azi_snr_lin = sin_snr_lin(mag_azi_db, idx_a)
    ele_snr_lin = sin_snr_lin(mag_ele_db, idx_e)

    n_azi = min(len(idx_a), doa_env.max_targets)
    n_ele = min(len(idx_e), doa_env.max_targets)

    # ----- 方位角插值与解算 -----
    if n_azi > 0:
        res_azi = np.empty((n_azi, 5), dtype=np.float32)
        idx_c = np.clip(idx_a[:n_azi], 1, fft_len - 2)
        denom = mag_azi_db[idx_c - 1] - 2 * mag_azi_db[idx_c] + mag_azi_db[idx_c + 1]
        offset = np.zeros_like(idx_c, dtype=np.float32)
        mask = denom != 0
        offset[mask] = (mag_azi_db[idx_c[mask] - 1] - mag_azi_db[idx_c[mask] + 1]) / (2.0 * denom[mask])
        offset = np.clip(offset, -0.5, 0.5)
        interp_bin = idx_a[:n_azi].astype(np.float32) + offset
        # 转换为角度：方位角满足 sin(azi) = 2 * (bin_index - N/2) / N
        shift = np.where(interp_bin < fft_len / 2, interp_bin, interp_bin - fft_len)
        sin_azi = np.clip(2.0 * shift / fft_len, -1.0, 1.0)
        azi_deg = np.rad2deg(np.arcsin(sin_azi))

        res_azi[:, 0] = 1.0                     # 有效标志
        res_azi[:, 1] = idx_a[:n_azi]           # 整数bin索引
        res_azi[:, 2] = mag_azi_db[idx_a[:n_azi]]  # 峰值幅度(dB)
        res_azi[:, 3] = interp_bin              # 插值后bin
        res_azi[:, 4] = azi_deg                 # 角度(deg)
    else:
        res_azi = np.empty((0, 5), dtype=np.float32)

    # ----- 俯仰角插值与解算 -----
    if n_ele > 0:
        res_ele = np.empty((n_ele, 5), dtype=np.float32)
        idx_c = np.clip(idx_e[:n_ele], 1, fft_len - 2)
        denom = mag_ele_db[idx_c - 1] - 2 * mag_ele_db[idx_c] + mag_ele_db[idx_c + 1]
        offset = np.zeros_like(idx_c, dtype=np.float32)
        mask = denom != 0
        offset[mask] = (mag_ele_db[idx_c[mask] - 1] - mag_ele_db[idx_c[mask] + 1]) / (2.0 * denom[mask])
        offset = np.clip(offset, -0.5, 0.5)
        interp_bin = idx_e[:n_ele].astype(np.float32) + offset
        shift = np.where(interp_bin < fft_len / 2, interp_bin, interp_bin - fft_len)
        sin_ele = np.clip(2.0 * shift / fft_len, -1.0, 1.0)
        ele_deg = np.rad2deg(np.arcsin(sin_ele))

        res_ele[:, 0] = 1.0
        res_ele[:, 1] = idx_e[:n_ele]
        res_ele[:, 2] = mag_ele_db[idx_e[:n_ele]]
        res_ele[:, 3] = interp_bin
        res_ele[:, 4] = ele_deg
    else:
        res_ele = np.empty((0, 5), dtype=np.float32)

    # ----- 多目标检测（仅使用第一个方位目标test）-----
    if len(res_azi) > 0:
        azm_bin_interp = res_azi[0, 3]                     # 插值后的bin
        two_pi_div_nfft = 2 * np.pi / doa_env.fft_len
        phase_input = two_pi_div_nfft * azm_bin_interp

        # 计算通道偏移（用于EVT检测器）
        azi_positions = Array_Azi[0, :]
        diffs = np.diff(azi_positions)
        channel_offsets = np.concatenate(([0.0], diffs)).astype(np.float32)
        evt_detector = MultiTargetEVT(channel_offsets, lambda_coeff=-np.pi, nof_mbf_channel=len(AziIdx_Select))

        ch_data = snap_data[AziIdx_Select].astype(np.complex64)
        is_multi, db_val, _, _ = evt_detector.is_multi_target(
            azm_bin_in=phase_input,
            ch_data_in=ch_data,
            is_moving=False
        )
        #print(f"多目标标志: {is_multi}, 比值(dB): {db_val:.2f}")

    return res_azi, res_ele, azi_snr_lin, ele_snr_lin