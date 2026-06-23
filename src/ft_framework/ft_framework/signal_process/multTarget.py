import torch
import numpy as np

class MultiTargetEVT_GPU:
    """
    基于 PyTorch GPU 加速的 EVT 多目标检测器
    """
    def __init__(self, channel_offsets: torch.Tensor, lambda_coeff: float = -np.pi, nof_mbf_channel: int = None, device: str = 'cuda'):
        """
        Args:
            channel_offsets: 通道位置差 Tensor，形状 (n_channels,)，必须已经在 GPU 上
            lambda_coeff: 相位计算公式中的系数
            nof_mbf_channel: 使用的通道数
            device: 运算设备
        """
        self.device = torch.device(device)
        self.lambda_coeff = lambda_coeff

        # 确保 channel_offsets 在指定的 GPU 上
        self.channel_offsets = channel_offsets.to(device=self.device, dtype=torch.float32)
        self.nof_mbf_channel = nof_mbf_channel if nof_mbf_channel is not None else len(self.channel_offsets)

        # 离线角度偏移转换成 GPU Tensor
        self.lut_der = torch.tensor([-0.0768, -0.0471], dtype=torch.float32, device=self.device)

        # 预计算 steering 向量（完全矩阵化，无循环）
        self.steering_vecs_conj = self._compute_steering_vectors()

    def _compute_steering_vectors(self) -> torch.Tensor:
        """利用广播机制在 GPU 上并行生成 4 个 steering 向量的共轭，形状 (4, n_channels)"""
        n = self.nof_mbf_channel
        offsets = self.channel_offsets[:n]  # (n,)

        sin_theta = torch.sin(self.lut_der)  # (2,)

        # 利用广播机制计算 2 个基础方向的 phase: (2, 1) * (n,) -> (2, n)
        phase = self.lambda_coeff * sin_theta.unsqueeze(1) * offsets.unsqueeze(0)
        sv_conj = torch.exp(-1j * phase.to(torch.complex64))

        # 构造 4 个方向的矩阵
        steering = torch.zeros((4, n), dtype=torch.complex64, device=self.device)
        steering[0:2, :] = sv_conj
        steering[2:4, :] = torch.conj(sv_conj).flip(dims=[0])  # 对称方向映射

        return steering

    def steer_ch_vect(self, azm_bin_in: torch.Tensor, ch_data_in: torch.Tensor) -> torch.Tensor:
        """通道数据相位补偿"""
        n = self.nof_mbf_channel
        offsets = self.channel_offsets[:n]
        # azm_bin_in 应当是个 0 维 Tensor 或者是标量
        rotation = torch.exp(1j * azm_bin_in * offsets)
        return ch_data_in[:n] * rotation

    def is_multi_target(
        self,
        azm_bin_in: torch.Tensor,
        ch_data_in: torch.Tensor,
        noise_var: float = None,
        is_moving: bool = False,
        threshold_db: float = 1.0
    ) -> tuple:
        """
        多目标判决 (全 Tensor 化，无 CPU 交互)
        """
        n = self.nof_mbf_channel

        # 1. 相位补偿
        ch_proc = self.steer_ch_vect(azm_bin_in, ch_data_in[:n])

        # 2. 复数均值（主目标分量）
        mean_cx = torch.mean(ch_proc)

        # 3. 减去均值（得到残差），非原地减法
        ch_proc_res = ch_proc - mean_cx

        # 4. 计算四个 steering 方向上的投影功率
        evt_steered = torch.mv(self.steering_vecs_conj, ch_proc_res)   # (4,)
        evt_pow = torch.absolute(evt_steered) ** 2
        max_evt_pow = torch.max(evt_pow)

        # 5. 主目标功率
        main_pow = torch.absolute(mean_cx) ** 2

        # 6. 计算比值(dB)，加极小值防除零、log(0)
        db_val = 10.0 * torch.log10(max_evt_pow / (main_pow + 1e-12) + 1e-12)

        # 主功率趋近于0时强制赋值负无穷
        db_val = torch.where(main_pow > 1e-12, db_val, torch.tensor(-float('inf'), device=self.device))

        is_multi = (db_val > threshold_db)

        # 返回GPU张量结果，外部调用.item()即可转为Python数值
        return is_multi, db_val, 10.0 * torch.log10(max_evt_pow + 1e-12), 10.0 * torch.log10(main_pow + 1e-12)
