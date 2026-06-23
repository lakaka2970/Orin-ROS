"""
基于EVT（特征向量）的多目标检测器
"""
import numpy as np


class MultiTargetEVT:
    """
    EVT多目标检测器
    利用预先计算的 steering 向量矩阵，对相位补偿后的数据进行残差分析，判断是否存在多目标
    """

    def __init__(self, channel_offsets: np.ndarray, lambda_coeff: float = -np.pi, nof_mbf_channel: int = None):
        """
        Args:
            channel_offsets: 通道位置差（波长或虚拟索引差），形状 (n_channels,)
            lambda_coeff: 相位计算公式中的系数 (例如 -π)
            nof_mbf_channel: 使用的通道数，默认全部
        """
        self.channel_offsets = np.asarray(channel_offsets, dtype=np.float32)
        self.nof_mbf_channel = nof_mbf_channel if nof_mbf_channel is not None else len(self.channel_offsets)
        # 预定义的离线角度偏移（弧度），用于生成4个steering方向
        self.lut_der = np.array([-0.0768, -0.0471], dtype=np.float32)
        self.lambda_coeff = lambda_coeff
        self.steering_vecs_conj = self._compute_steering_vectors()

    def _compute_steering_vectors(self) -> np.ndarray:
        """生成4个steering向量的共轭，形状 (4, n_channels)"""
        n = self.nof_mbf_channel
        offsets = self.channel_offsets[:n]
        steering = np.zeros((4, n), dtype=np.complex64)
        for i, sin_theta in enumerate(np.sin(self.lut_der)):
            phase = self.lambda_coeff * sin_theta * offsets
            sv_conj = np.exp(-1j * phase).astype(np.complex64)
            steering[i, :] = sv_conj
            steering[3 - i, :] = np.conj(sv_conj)   # 对称方向
        return steering

    def steer_ch_vect(self, azm_bin_in: float, ch_data_in: np.ndarray) -> np.ndarray:
        """
        根据方位bin对通道数据进行相位补偿

        Args:
            azm_bin_in: 插值后的FFT bin索引（已乘以2π/Nfft）
            ch_data_in: 原始通道数据 (n_channels,)

        Returns:
            相位补偿后的数据
        """
        n = self.nof_mbf_channel
        offsets = self.channel_offsets[:n]
        rotation = np.exp(1j * azm_bin_in * offsets).astype(np.complex64)
        return ch_data_in[:n] * rotation

    def is_multi_target(
        self,
        azm_bin_in: float,
        ch_data_in: np.ndarray,
        noise_var: float = None,
        is_moving: bool = False,
        threshold_db: float = 1.0
    ) -> tuple:
        """
        多目标判决

        Args:
            azm_bin_in: 插值后的FFT bin索引（已乘以2π/Nfft）
            ch_data_in: 原始通道数据 (>= nof_mbf_channel)
            noise_var: 噪声方差（未使用，保留接口）
            is_moving: 是否运动场景（影响阈值）
            threshold_db: 判决阈值(dB)

        Returns:
            is_multi: 是否为多目标
            db_value: 实际比值(dB)
            evt_db: 残差功率(dB)
            main_pow_db: 主目标功率(dB)
        """
        n = self.nof_mbf_channel
        # 1. 相位补偿
        ch_proc = self.steer_ch_vect(azm_bin_in, ch_data_in[:n])

        # 2. 复数均值（主目标分量）
        mean_cx = np.mean(ch_proc)

        # 3. 减去均值（得到残差）
        ch_proc -= mean_cx

        # 4. 计算四个 steering 方向上的投影功率
        evt_steered = np.dot(self.steering_vecs_conj, ch_proc)          # (4,)
        evt_pow = np.abs(evt_steered) ** 2
        max_evt_pow = np.max(evt_pow)

        # 5. 主目标功率
        main_pow = np.abs(mean_cx) ** 2

        # 6. 计算比值(dB)
        if main_pow > 0:
            db_val = 10.0 * np.log10(max_evt_pow / main_pow)
        else:
            db_val = -np.inf

        is_multi = (db_val > threshold_db)
        return is_multi, db_val, 10.0 * np.log10(max_evt_pow), 10.0 * np.log10(main_pow)