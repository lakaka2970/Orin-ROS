"""
虚拟阵列生成与双目标DOA仿真数据构造模块
"""
import numpy as np
import time
from dataclasses import dataclass, asdict
from scipy.io import savemat
from ft_framework.signal_process.doa_proc_cpu import doa_main_ultra_separated


@dataclass
class DOAData:
    """存储单次DOA仿真结果的数据类"""
    power_diff_dB: float = 0.0
    target1_az: float = 0.0
    target1_el: float = 0.0
    target2_az: float = 0.0
    target2_el: float = 0.0
    snr_dB: float = 0.0
    complex_data: np.ndarray = None


class RadarArrayInitializer:
    """雷达阵列初始化：定义收发天线坐标，构造虚拟阵列并分离方位/俯仰子阵"""

    def __init__(self):
        # 发射天线坐标 [3, N_tx]
        self.tx_pos = np.array([
            [24, 0, 0], [28, 0, 0], [20, 0, 0], [16, 0, 0], [12, 0, 0],
            [8, 0, 0], [4, 0, 0], [0, 0, 0], [53, 34, 0], [53, 41, 0],
            [53, 27, 0], [53, 14, 0], [53, 7, 0], [53, 0, 0], [45, 0, 0], [38, 0, 0]
        ], dtype=np.float32).T

        # 接收天线坐标 [3, N_rx]
        self.rx_pos = np.array([
            [10, 40, 0], [5, 40, 0], [0, 40, 0], [0, 32, 0], [0, 24, 0],
            [0, 16, 0], [0, 0, 0], [0, 8, 0], [52, 40, 0], [46, 40, 0],
            [38, 40, 0], [33, 40, 0], [28, 40, 0], [24, 40, 0], [15, 40, 0], [19, 40, 0]
        ], dtype=np.float32).T

        # 合成虚拟阵列 (3, N_tx * N_rx)
        self.virtual_array_np = (self.tx_pos[:, :, None] + self.rx_pos[:, None, :]).reshape(3, -1)
        self._separate_subarrays()

    def _separate_subarrays(self):
        """
        从虚拟阵列中分离方位子阵和俯仰子阵：
        - 方位子阵：选取 y,z 重复最多的坐标组，Tx→Rx 顺序遍历，x 先到先得去重
        - 俯仰子阵：选取 x,z 重复最多的坐标组，Tx→Rx 顺序遍历，y 先到先得去重
        """
        arr = self.virtual_array_np
        n_tx = self.tx_pos.shape[1]
        n_rx = self.rx_pos.shape[1]

        # ---- 方位子阵：y=40, z=0 ----
        yz_hash = arr[1, :] * 10000.0 + arr[2, :]
        unique_yz, counts = np.unique(yz_hash, return_counts=True)
        best_yz = unique_yz[np.argmax(counts)]

        seen_x = set()
        azi_ch_list = []
        for tx_idx in range(n_tx):
            for rx_idx in range(n_rx):
                ch = tx_idx * n_rx + rx_idx
                if yz_hash[ch] == best_yz:
                    x_val = arr[0, ch]
                    if x_val not in seen_x:
                        seen_x.add(x_val)
                        azi_ch_list.append(ch)
        self.AziIdx_Select = np.array(azi_ch_list, dtype=np.int64)
        self.Array_Azi = arr[:, self.AziIdx_Select]

        # ---- 俯仰子阵：x=53, z=0 ----
        xz_hash = arr[0, :] * 10000.0 + arr[2, :]
        unique_xz, counts_ele = np.unique(xz_hash, return_counts=True)
        best_xz = unique_xz[np.argmax(counts_ele)]

        seen_y = set()
        elv_ch_list = []
        for tx_idx in range(n_tx):
            for rx_idx in range(n_rx):
                ch = tx_idx * n_rx + rx_idx
                if xz_hash[ch] == best_xz:
                    y_val = arr[1, ch]
                    if y_val not in seen_y:
                        seen_y.add(y_val)
                        elv_ch_list.append(ch)
        self.EleIdx_Select = np.array(elv_ch_list, dtype=np.int64)
        self.Array_Ele = arr[:, self.EleIdx_Select]


def gen_doa_sim_data_numpy(array_init: RadarArrayInitializer, aziFlag: bool) -> list:
    """
    生成用于DOA评估的双目标仿真数据（包含噪声）
    
    Args:
        array_init: 已初始化的阵列对象
        aziFlag: True表示第二个目标变化方位角，False表示变化俯仰角
    
    Returns:
        DOAData对象列表
    """
    v_arr = array_init.virtual_array_np
    n_elem = v_arr.shape[1]
    num_samples = 100                     # 每个快拍内的采样数

    power_diffs = np.arange(0, 30, 5)     # 功率差(dB) 0,5,...,25
    snr_list = [40, 35, 30, 25, 20, 15, 10]

    # 第二个目标的角度变化范围（方位或俯仰）
    t2_angles = np.array([0.2, 3, 4]) if aziFlag else np.array([-10, 0, 10])

    # 目标1导向矢量 (az=0, el=0)
    sv1 = np.exp(-1j * np.pi * v_arr[2, :])

    doa_data_list = []

    for pdB in power_diffs:
        amp_ratio = 10.0 ** (-pdB / 20.0)          # 幅度比 (A2/A1)
        for angle in t2_angles:
            az2, el2 = (angle, 0.0) if aziFlag else (0.0, angle)
            az2_rad, el2_rad = np.deg2rad(az2), np.deg2rad(el2)

            # 目标2导向矢量
            phase = 1j * np.pi * (
                v_arr[0] * np.sin(az2_rad) * np.cos(el2_rad) +
                v_arr[1] * np.sin(el2_rad) +
                v_arr[2] * np.cos(az2_rad) * np.cos(el2_rad)
            )
            sv2 = np.exp(-phase)

            signal = sv1 + amp_ratio * sv2
            sig_power = np.mean(np.abs(signal) ** 2)

            for snr in snr_list:
                noise_power = sig_power * (10.0 ** (-snr / 10.0))
                noise = np.sqrt(noise_power / 2.0) * (
                    np.random.randn(n_elem, num_samples) +
                    1j * np.random.randn(n_elem, num_samples)
                ).astype(np.complex64)
                data = signal[:, np.newaxis] + noise

                doa_data_list.append(DOAData(
                    power_diff_dB=float(pdB),
                    target1_az=0.0, target1_el=0.0,
                    target2_az=float(az2), target2_el=float(el2),
                    snr_dB=float(snr),
                    complex_data=data
                ))

    # 保存为npz和mat文件
    name = 'doa_simulation_data_direct_azi.npz' if aziFlag else 'doa_simulation_data_direct_ele.npz'
    dicts = [asdict(d) for d in doa_data_list]
    np.savez(name, doa_data=dicts)
    savemat(name.replace('.npz', '.mat'), {'doa_data': dicts})
    print(f"✅ 仿真数据生成完成: {name} | 总数: {len(doa_data_list)}")
    return doa_data_list


if __name__ == '__main__':
    t0 = time.perf_counter()
    array_env = RadarArrayInitializer()
    print(f"阵列初始化耗时: {(time.perf_counter() - t0) * 1000:.2f} ms")

    aziFlag = True
    t_gen = time.perf_counter()
    doa_data = gen_doa_sim_data_numpy(array_env, aziFlag)
    print(f"数据生成耗时: {(time.perf_counter() - t_gen) * 1000:.2f} ms")

    # 测试单帧测角
    snap = doa_data[0].complex_data[:, 1]        # 取第二帧
    azi_res, ele_res = doa_main_ultra_separated(
        snap, array_env.Array_Azi, array_env.AziIdx_Select,
        array_env.Array_Ele, array_env.EleIdx_Select
    )
    print(f"方位探测结果数: {len(azi_res)}, 俯仰探测结果数: {len(ele_res)}")