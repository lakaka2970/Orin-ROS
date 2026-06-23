import numpy as np
import struct
import os
from typing import Dict, List, Any, Optional, Tuple

# ============================================================
# 物理常数
# ============================================================
C0 = 299792458.0  # 光速 m/s

# ============================================================
# 归一化函数 (全局版本)
# ============================================================
def global_normalisation(data: np.ndarray, max_abs: float = 1.0) -> np.ndarray:
    """
    将整个数组线性缩放到最大绝对值 = max_abs
    """
    max_val = np.max(np.abs(data))
    if max_val > 1e-12:
        data = data / (max_val / max_abs)
    return data

# ============================================================
# 时间轴生成 (修正偏移)
# ============================================================
def create_timeline(param: Dict[str, Any]) -> np.ndarray:
    """
    生成每个 ADC 采样点的绝对时间向量 (第一个采样点从 adc_start_delay 开始)
    """
    nof_adc_sample_per_chirp = param['WFP']['nof_adc_sample_per_chirp']
    nof_chirp = param['WFP']['nof_chirp']
    sample_rate_hz = param['WFP']['sample_rate_kHz'] * 1e3
    adc_start_delay = param['MMIC']['adcStartTimeConst_profileCfg'] * 1e-8
    time_idle = param['WFP']['time_idle_us'] * 1e-6
    time_ramp_end = param['WFP']['time_ramp_end_us'] * 1e-6

    Ts = 1.0 / sample_rate_hz
    # 单个 chirp 内采样点时间（相对该 chirp 起始） 修正：从0开始
    time_per_chirp = np.arange(nof_adc_sample_per_chirp) * Ts + adc_start_delay

    timeline = np.zeros(nof_adc_sample_per_chirp * nof_chirp)
    for chirp_idx in range(nof_chirp):
        start = chirp_idx * nof_adc_sample_per_chirp
        end = start + nof_adc_sample_per_chirp
        timeline[start:end] = time_per_chirp
        # 每个 chirp 结束后增加空闲时间和 ramp 结束时间
        time_per_chirp = time_per_chirp + time_ramp_end + time_idle

    return timeline

# ============================================================
# 合成数据生成器 (修正版)
# ============================================================
class SynDataGenerator:
    def __init__(self, scenario: Dict[str, Any], param: Dict[str, Any]):
        self.scenario = scenario
        self.param = param
        self.num_tx = len(param['ANTP']['tx_pos']) // 3
        self.num_rx = len(param['ANTP']['rx_pos']) // 3
        self.num_targets = scenario['num_target']
        self.project_name = param['project_name']
        self.lambda_factor = param['ANTP']['channel_step_lambda']

        # 波长 (使用起始频率)
        self.lambda0 = C0 / param['WFP']['freq_start_Hz']

        # 转换天线位置到米 (使用减法约定，天线坐标可为正负)
        self.rx_x_m = np.array(param['ANTP']['rx_pos'][:self.num_rx]) * (self.lambda0 * self.lambda_factor)
        self.rx_y_m = np.array(param['ANTP']['rx_pos'][self.num_rx:2*self.num_rx]) * (self.lambda0 * self.lambda_factor)
        self.rx_z_m = np.array(param['ANTP']['rx_pos'][2*self.num_rx:3*self.num_rx]) * (self.lambda0 * self.lambda_factor)
        self.tx_x_m = np.array(param['ANTP']['tx_pos'][:self.num_tx]) * (self.lambda0 * self.lambda_factor)
        self.tx_y_m = np.array(param['ANTP']['tx_pos'][self.num_tx:2*self.num_tx]) * (self.lambda0 * self.lambda_factor)
        self.tx_z_m = np.array(param['ANTP']['tx_pos'][2*self.num_tx:3*self.num_tx]) * (self.lambda0 * self.lambda_factor)

        # 波形参数
        self.freq_start_hz = param['WFP']['freq_start_Hz']
        self.freq_slope_hz_per_s = param['WFP']['freq_slope_MHzPus'] * 1e12
        self.sample_rate_hz = param['WFP']['sample_rate_kHz'] * 1e3
        self.nof_adc_sample_per_chirp = param['WFP']['nof_adc_sample_per_chirp']
        self.nof_chirp = param['WFP']['nof_chirp']
        self.all_sample = self.nof_adc_sample_per_chirp * self.nof_chirp

        # 频率步进（如果存在）
        self.freq_band_stepped_hz = param['WFP'].get('freq_band_stepped_Hz', 0.0)

        # PSK 调制参数
        self.rotation_speed = param['WFP']['rotation_speed']  # 列表，每个 TX 一个
        self.nof_ddma_period = param['WFP'].get('nof_ddma_period', 1)

        # 信噪比 (dB)，新增参数，若未提供则使用默认 30 dB
        self.snr_dB = param.get('snr_dB', 30.0)

        # 生成时间轴
        self.timeline = create_timeline(param)

        # 准备频率数组 (每个采样点的瞬时发射频率)
        self.freq_array = self._build_freq_array()

        # 准备 ramp 索引数组 (用于 PSK)
        self.ramp_index_arr = self._build_ramp_index()

        # 输出数据容器 (预分配)
        if self.project_name in ["CR4T4R", "FLR4T4R","16T16R"]:
            self.sigMxArr = np.zeros((self.num_rx, self.nof_adc_sample_per_chirp, self.nof_chirp, param['Number_of_Frames']),
                                     dtype=np.float32)
        else:  # 复数情况
            self.sigMxArr = np.zeros((self.num_rx, self.nof_adc_sample_per_chirp, self.nof_chirp, param['Number_of_Frames']),
                                     dtype=np.complex64)

        # 存储每帧目标真值
        self.gt_xyz = np.full((self.param['RSPP']['max_nof_det_per_frame'], 3, param['Number_of_Frames']), np.nan)

    def _build_freq_array(self) -> np.ndarray:
        """构建每个采样点的发射频率 (修正时间轴偏移)"""
        freq = np.zeros(self.all_sample)
        adc_start_delay = self.param['MMIC']['adcStartTimeConst_profileCfg'] * 1e-8
        # 第一个 chirp 第一个采样点的频率
        freq[0] = self.freq_start_hz + self.freq_slope_hz_per_s * adc_start_delay
        for i in range(1, self.nof_adc_sample_per_chirp):
            dt = self.timeline[i] - self.timeline[i-1]
            freq[i] = freq[i-1] + dt * self.freq_slope_hz_per_s
        # 复制到所有 chirp，并加上频率步进
        for chirp in range(1, self.nof_chirp):
            start = chirp * self.nof_adc_sample_per_chirp
            end = start + self.nof_adc_sample_per_chirp
            freq[start:end] = freq[:self.nof_adc_sample_per_chirp] + chirp * self.freq_band_stepped_hz / (self.nof_chirp - 1)
        return freq

    def _build_ramp_index(self) -> np.ndarray:
        """构建每个采样点对应的 ramp 序号（0 起始）"""
        ramp_idx = np.zeros(self.all_sample, dtype=int)
        for chirp in range(self.nof_chirp):
            start = chirp * self.nof_adc_sample_per_chirp
            end = start + self.nof_adc_sample_per_chirp
            ramp_idx[start:end] = chirp
        return ramp_idx

    def _update_target_positions(self, frame_time: float):
        """根据帧间时间更新所有目标位置 (匀速/匀加速运动)"""
        for t in range(self.num_targets):
            trg = self.scenario['target'][t]
            dt = frame_time
            trg['xPos_m'] += dt * (0.5 * dt * trg['xAcc_mpss'] + trg['xVel_mps'])
            trg['yPos_m'] += dt * (0.5 * dt * trg['yAcc_mpss'] + trg['yVel_mps'])

    def generate_frame(self, frame_id: int) -> np.ndarray:
        """
        生成一帧数据，返回 sigMxArr 对应帧的数据 (rx, samp, chirp)
        修正：减法距离，正确相位，全局归一化，按 SNR 添加噪声
        """
        # 第一步：无噪声生成信号（用于计算信号功率和归一化）
        # 形状 (num_rx, samp, chirp)
        clean_signal = np.zeros((self.num_rx, self.nof_adc_sample_per_chirp, self.nof_chirp),
                                dtype=np.float32 if self.project_name in ["CR4T4R", "FLR4T4R","16T16R"] else np.complex64)

        # 分别计算每个接收通道的无噪声信号
        for rx_idx in range(self.num_rx):
            # 初始化该通道的累加器
            if self.project_name in ["CR4T4R", "FLR4T4R","16T16R"]:
                rx_clean = np.zeros((self.nof_adc_sample_per_chirp, self.nof_chirp), dtype=np.float32)
            else:
                rx_clean = np.zeros((self.nof_adc_sample_per_chirp, self.nof_chirp), dtype=np.complex64)

            for tx_idx in range(self.num_tx):
                # PSK 相位
                psk_phase = 2 * np.pi * self.ramp_index_arr * self.rotation_speed[tx_idx] / self.nof_ddma_period

                if self.project_name in ["CR4T4R", "FLR4T4R","16T16R"]:
                    sig_fm = np.zeros(self.all_sample, dtype=np.float32)
                else:
                    sig_fm = np.zeros(self.all_sample, dtype=np.complex64)

                for trg_idx in range(self.num_targets):
                    trg = self.scenario['target'][trg_idx]
                    # 目标随时间的位置 (匀加速运动)
                    x_trg = trg['xPos_m'] + self.timeline * (0.5 * self.timeline * trg['xAcc_mpss'] + trg['xVel_mps'])
                    y_trg = trg['yPos_m'] + self.timeline * (0.5 * self.timeline * trg['yAcc_mpss'] + trg['yVel_mps'])
                    z_trg = trg['zPos_m']

                    # 检查是否在 FOV 内 (方位角绝对值小于 FOV/2 且 x>0)
                    azim_deg = np.abs(np.degrees(np.arctan2(y_trg, x_trg)))
                    valid = (azim_deg <= self.scenario['sensor']['FOV'] / 2) & (x_trg >= 0)
                    if not np.any(valid):
                        continue

                    # 对无效点置 NaN，避免计算距离
                    x_trg_valid = x_trg.copy()
                    y_trg_valid = y_trg.copy()
                    x_trg_valid[~valid] = np.nan
                    y_trg_valid[~valid] = np.nan

                    # 距离计算: 减法（天线坐标在雷达坐标系中）
                    range_tx2trg = np.sqrt((x_trg_valid - self.tx_x_m[tx_idx])**2 +
                                           (y_trg_valid - self.tx_y_m[tx_idx])**2 +
                                           (z_trg - self.tx_z_m[tx_idx])**2)
                    range_trg2rx = np.sqrt((x_trg_valid - self.rx_x_m[rx_idx])**2 +
                                           (y_trg_valid - self.rx_y_m[rx_idx])**2 +
                                           (z_trg - self.rx_z_m[rx_idx])**2)
                    total_range = range_tx2trg + range_trg2rx
                    tof = total_range / C0
                    rf = self.freq_array + self.freq_slope_hz_per_s  * tof
                    # 修正的 FMCW 中频相位
                    phase = 2*np.pi*tof*rf + psk_phase;
                    # phase = 2π ( f_tx(t) * τ - 0.5 * S * τ² )
                    #phase = 2 * np.pi * (self.freq_array * tof - 0.5 * self.freq_slope_hz_per_s * tof**2) + psk_phase

                    # 幅度 (默认所有目标幅度为1，可扩展)
                    amp = 1.0

                    if self.project_name in ["CR4T4R", "FLR4T4R","16T16R"]:
                        video = amp * np.cos(phase)
                        video = np.nan_to_num(video)
                        sig_fm += video
                    else:
                        video = amp * np.exp(1j * phase)
                        video = np.nan_to_num(video)
                        sig_fm += video

                # 叠加当前 TX 贡献到接收通道
                transposed = sig_fm.reshape(self.nof_adc_sample_per_chirp, self.nof_chirp,order='F')
                if self.project_name in ["CR4T4R", "FLR4T4R","16T16R"]:
                    rx_clean += transposed
                else:
                    rx_clean += transposed

            clean_signal[rx_idx, :, :] = rx_clean

        # 第二步：计算信号功率并添加噪声（按 SNR）
        signal_power = np.mean(clean_signal**2)  # 对所有元素求平均功率
        snr_linear = 10**(self.snr_dB / 10.0)
        noise_power = signal_power / snr_linear
        sigma_noise = np.sqrt(noise_power)

        # 生成噪声
        if self.project_name in ["CR4T4R", "FLR4T4R","16T16R"]:
            noise = sigma_noise * np.random.randn(self.num_rx, self.nof_adc_sample_per_chirp, self.nof_chirp)
            signal_with_noise = clean_signal + noise
        else:
            noise_re = sigma_noise * np.random.randn(self.num_rx, self.nof_adc_sample_per_chirp, self.nof_chirp)
            noise_im = sigma_noise * np.random.randn(self.num_rx, self.nof_adc_sample_per_chirp, self.nof_chirp)
            noise = noise_re + 1j * noise_im
            signal_with_noise = clean_signal + noise

        # 第三步：全局归一化 (所有通道一起) 并缩放到 1000 范围
        max_abs = np.max(np.abs(signal_with_noise))
        if max_abs > 1e-12:
            signal_scaled = signal_with_noise / max_abs * 1000.0
        else:
            signal_scaled = signal_with_noise

        # 返回 int16 兼容的浮点数据（写入时会 clip）
        return signal_scaled.astype(np.float32 if self.project_name in ["CR4T4R", "FLR4T4R","16T16R"] else np.complex64)

    def generate_all_frames(self):
        """生成所有帧"""
        for frame_id in range(self.param['Number_of_Frames']):
            # 记录当前帧目标真值 (位置更新前)
            for trg_idx in range(self.num_targets):
                trg = self.scenario['target'][trg_idx]
                if trg_idx < self.param['RSPP']['max_nof_det_per_frame']:
                    # 存储顺序: x, y, z (符合常规)
                    self.gt_xyz[trg_idx, 0, frame_id] = trg['xPos_m']
                    self.gt_xyz[trg_idx, 1, frame_id] = trg['yPos_m']
                    self.gt_xyz[trg_idx, 2, frame_id] = trg['zPos_m']

            # 生成当前帧数据
            frame_data = self.generate_frame(frame_id)
            self.sigMxArr[:, :, :, frame_id] = frame_data

            # 更新目标位置，准备下一帧 (帧间时间)
            frame_duration = self.timeline[-1] + self.param['WFP']['time_idle_us'] * 1e-6
            self._update_target_positions(frame_duration)

        return self.sigMxArr, self.gt_xyz


# ============================================================
# 二进制文件写入 (修正：采样点- chirp-通道交织)
# ============================================================
def num2uint32_le(num: int) -> Tuple[int, int]:
    low = num & 0xFFFF
    high = (num >> 16) & 0xFFFF
    return low, high

def write_syn_data_bin(sigMxArr: np.ndarray, param: Dict[str, Any], 
                       output_filename: str, 
                       ppar_data: Optional[np.ndarray] = None,
                       apar_data: Optional[np.ndarray] = None,
                       rpar_data: Optional[np.ndarray] = None):
    """
    将合成数据写入二进制文件，格式兼容 data_plot.py 和 data_analysis.m
    数据排列: 每帧内按 (采样点, chirp, 接收通道) 顺序连续存储 (C顺序)
    """
    num_frames = sigMxArr.shape[3] if sigMxArr.ndim == 4 else 1


    with open(output_filename, 'wb') as fid:
        for frame in range(num_frames):
            # 注释掉头部写入（保持与之前相同，不写头部）
            # header.tofile(fid)

            # 获取当前帧数据 (rx, samp, chirp)
            frame_data = sigMxArr[:, :, :, frame]
            # 重排为 (samp, chirp, rx) 顺序 (C 顺序展平)
            # ===================== 【关键：生成递增数字】 =====================
            #total = num_rx * 2048 * 256
            #test_seq = np.arange(1, total + 1, dtype=np.int16)  # 1,2,3,4,...
            
            # 重排成和真实数据完全一样的顺序：[samp, chirp, rx]
            #frame_data = test_seq.reshape(num_rx, 2048, 256)  # (16,2048,256)
            frame_data = np.transpose(frame_data, (1, 2, 0))  # (samp, chirp, rx)
            flat = frame_data.flatten()  # C 顺序: samp0_chirp0_rx0, samp0_chirp0_rx1, ..., samp0_chirp1_rx0, ...

            if np.iscomplexobj(sigMxArr):
                # 复数: 每两个 int16 表示一个复数，顺序 (imag, real) 与之前保持一致
                frame_adc = []
                for val in flat:
                    i = int(np.clip(np.imag(val), -32768, 32767))
                    r = int(np.clip(np.real(val), -32768, 32767))
                    frame_adc.extend([i, r])
            else:
                frame_adc = [int(np.clip(np.round(val), -32768, 32767)) for val in flat]

            frame_adc = np.array(frame_adc, dtype=np.int16)
            frame_adc.tofile(fid)

    print(f"Binary file saved to {output_filename}")


# ============================================================
# 使用示例
# ============================================================
if __name__ == "__main__":
        # 参数配置 (同原示例，但增加 snr_dB)
    tx_pos = np.array([
        [0, 0, 0], [4,0, 0], [8,0, 0], [12,0, 0],
        [16,0, 0], [20,0, 0], [24,0, 0], [28,0, 0],
        [38,0, 0], [45,0, 0], [53,0, 0], [53,7, 0],
        [53,14, 0], [53,27, 0], [53,34, 0], [53,41, 0]
    ]).T   # 此时 tx_pos[0] 是 x 坐标列表，tx_pos[1] 是 y，tx_pos[2] 是 z

    rx_pos = np.array([
        [0,0, 0],[5,40, 0],[10,40, 0],[15,40, 0],
        [19,40, 0],[24,40, 0],[28,40, 0],[33,40, 0],
        [38,40, 0],[46,40, 0],[52,40, 0],[0,8, 0],
        [0,16, 0],[0,24, 0],[0,32, 0],[0,40, 0]
    ]).T

    # 转换为原有列表格式
    tx_pos_list = tx_pos[0].tolist() + tx_pos[1].tolist() + tx_pos[2].tolist()
    rx_pos_list = rx_pos[0].tolist() + rx_pos[1].tolist() + rx_pos[2].tolist()

    param_example = {
        'project_name': '16T16R',
        'Number_of_Frames': 1,
        'snr_dB': 30.0,                     # 新增信噪比参数
        'WFP': {
            'freq_start_Hz': 77e9,
            'freq_slope_MHzPus': 14.0,
            'sample_rate_kHz': 60000.0,
            'nof_adc_sample_per_chirp': 2048,
            'nof_chirp': 512,
            'time_idle_us': 5.0,
            'time_ramp_end_us': 35.0,
            'freq_band_stepped_Hz': 0.0,
            'rotation_speed': [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15],     # 16个TX
            'nof_ddma_period': 32,
        },
        'MMIC': {
            'adcStartTimeConst_profileCfg': 0.0,
        },
        'ANTP': {
            'tx_pos':tx_pos_list,  # 简单示例，实际应设间距
            'rx_pos':rx_pos_list,
            'channel_step_lambda': 0.5,
        },
        'RSPP': {
            'max_nof_det_per_frame': 10,
        },
    }

    scenario_example = {
        'num_target': 1,
        'target': [
            {
                'xPos_m': 50.0, 'yPos_m': 0.0, 'zPos_m': 0.0,
                'xVel_mps': 10.0, 'yVel_mps': 0.0,
                'xAcc_mpss': 0.0, 'yAcc_mpss': 0.0,
                'snr_dB': 60.0,
            },
            {
                'xPos_m': 30.0, 'yPos_m': 5.0, 'zPos_m': 1.0,
                'xVel_mps': -5.0, 'yVel_mps': 1.0,
                'xAcc_mpss': 0.0, 'yAcc_mpss': 0.0,
                'snr_dB': 25.0,
            }
        ],
        'sensor': {'FOV': 90.0}
    }

    # 生成数据
    generator = SynDataGenerator(scenario_example, param_example)
    sigMxArr, gt_xyz = generator.generate_all_frames()
    print("Generated signal shape:", sigMxArr.shape)

    # 写入 bin 文件
    write_syn_data_bin(sigMxArr, param_example, "synthetic_data_fixed.bin")