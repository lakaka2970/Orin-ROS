import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat, loadmat
from dataclasses import dataclass, asdict
import os
import doa_proc
# 数据结构（等价 MATLAB struct）
@dataclass
class DOAData:
    power_diff_dB: float = 0.0
    target1_az: float = 0.0
    target1_el: float = 0.0
    target2_az: float = 0.0
    target2_el: float = 0.0
    snr_dB: float = 0.0
    complex_data: np.ndarray = None

# ==========================
# 阵列配置
# ==========================
def layoutCfg():
    # 发射阵列（半波长）
    tx_pos = np.array([
        [0, 0, 0], [4,0, 0], [8,0, 0], [12,0, 0],
        [16,0, 0], [20,0, 0], [24,0, 0], [28,0, 0],
        [38,0, 0], [45,0, 0], [53,0, 0], [53,7, 0],
        [53,14, 0], [53,27, 0], [53,34, 0], [53,41, 0]
    ]).T

    # 接收阵列
    rx_pos = np.array([
        [0,0, 0],[5,40, 0],[10,40, 0],[15,40, 0],
        [19,40, 0],[24,40, 0],[28,40, 0],[33,40, 0],
        [38,40, 0],[46,40, 0],[52,40, 0],[0,8, 0],
        [0,16, 0],[0,24, 0],[0,32, 0],[0,40, 0]
    ]).T
    # 虚拟阵列（MIMO）
    virtual = []
    for tx in tx_pos.T:
        for rx in rx_pos.T:
            virtual.append(tx + rx)
    virtual_array = np.array(virtual).T

    # 绘图
    plt.figure(figsize=(8,6))
    plt.plot(tx_pos[0], tx_pos[1], 'ro', markersize=10, linewidth=2, label='TX')
    plt.plot(rx_pos[0], rx_pos[1], 'bs', markersize=10, linewidth=2, label='RX')
    plt.plot(virtual_array[0], virtual_array[1], 'g.', markersize=10, label='Virtual')
    plt.title('Array Geometry')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()

    return virtual_array

def Array_Seprate(Array_Virtual, showPattern=False, showArray=False):
    """
    从 MIMO 虚拟阵列中分离方位子阵和俯仰子阵。

    参数:
        Array_Virtual: ndarray, shape (3, N) - 虚拟阵元坐标，第一行 x (方位)，第二行 y (俯仰)，第三行 z
        showPattern, showArray: 保留参数，本函数未实现绘图

    返回:
        Array_Azi: ndarray, (3, M_azi) - 方位子阵（所有阵元具有相同的 y 和 z，按 x 排序）
        AziIdx_Select: ndarray, 在原始 Array_Virtual 中的列索引
        Array_Ele: ndarray, (3, M_ele) - 俯仰子阵（所有阵元具有相同的 x 和 z，按 y 排序）
        EleIdx_Select: ndarray, 在原始 Array_Virtual 中的列索引
    """
    # ---------- 方位子阵 (Azimuth) ----------
    # 统计每个 (y, z) 组合的出现次数
    yz_pairs = np.ascontiguousarray(Array_Virtual[[1, 2], :].T)  # (N, 2)
    _, unique_indices, counts = np.unique(yz_pairs, axis=0, return_inverse=True, return_counts=True)
    # 找到出现次数最多的组合的索引
    max_count_idx = np.argmax(counts)
    # 获取该组合对应的原始列索引
    mask = (unique_indices == max_count_idx)
    AziIdx_Select = np.where(mask)[0]   # 这些是满足条件的列索引
    Array_Azi = Array_Virtual[:, AziIdx_Select]

    # 按 x 坐标排序
    sort_idx = np.argsort(Array_Azi[0, :])
    Array_Azi = Array_Azi[:, sort_idx]
    AziIdx_Select = AziIdx_Select[sort_idx]

    # 去重（如果有完全重复的阵元位置，保留第一个）
    _, unique_azi_idx = np.unique(Array_Azi.T, axis=0, return_index=True)
    unique_azi_idx = np.sort(unique_azi_idx)
    Array_Azi = Array_Azi[:, unique_azi_idx]
    AziIdx_Select = AziIdx_Select[unique_azi_idx]

    # ---------- 俯仰子阵 (Elevation) ----------
    # 统计每个 (x, z) 组合的出现次数
    xz_pairs = np.ascontiguousarray(Array_Virtual[[0, 2], :].T)  # (N, 2)
    _, unique_indices_ele, counts_ele = np.unique(xz_pairs, axis=0, return_inverse=True, return_counts=True)
    max_count_idx_ele = np.argmax(counts_ele)
    mask_ele = (unique_indices_ele == max_count_idx_ele)
    EleIdx_Select = np.where(mask_ele)[0]
    Array_Ele = Array_Virtual[:, EleIdx_Select]

    # 按 y 坐标排序
    sort_idx_ele = np.argsort(Array_Ele[1, :])
    Array_Ele = Array_Ele[:, sort_idx_ele]
    EleIdx_Select = EleIdx_Select[sort_idx_ele]

    # 去重
    _, unique_ele_idx = np.unique(Array_Ele.T, axis=0, return_index=True)
    unique_ele_idx = np.sort(unique_ele_idx)
    Array_Ele = Array_Ele[:, unique_ele_idx]
    EleIdx_Select = EleIdx_Select[unique_ele_idx]

    return Array_Azi, AziIdx_Select, Array_Ele, EleIdx_Select
# ==========================
# 仿真数据生成
# ==========================
def genDoaSimData(virtual_array, aziFlag):
    target1_az, target1_el = 0, 0
    num_samples = 100
    target1_amp = 1.0
    power_diff_dB = np.arange(0, 30, 5)
    snr_dB = [40,35,30,25,20,15,10]

    if aziFlag:
        target2_el = 0
        t2 = np.concatenate([np.arange(0.1,0.5,0.1), np.arange(0.5,20.5,0.5)])
    else:
        target2_az = 0
        t2 = np.arange(1,10.5,0.5)

    def steer(az, el):
        az_r = np.deg2rad(az)
        el_r = np.deg2rad(el)
        phase = 1j * np.pi * (
            virtual_array[0] * np.sin(az_r) * np.cos(el_r) +
            virtual_array[1] * np.sin(el_r) +
            virtual_array[2] * np.cos(az_r) * np.cos(el_r)
        )
        return np.exp(-phase)

    doa_data = []
    for pdB in power_diff_dB:
        a2 = target1_amp * (10 ** (-pdB/20))
        if aziFlag:
            for az2 in t2:
                for snr in snr_dB:
                    sv1 = steer(target1_az, target1_el)
                    sv2 = steer(az2, target2_el)
                    sig = target1_amp*sv1 + a2*sv2
                    sig_pwr = np.mean(np.abs(sig)**2)
                    noise_pwr = sig_pwr * (10**(-snr/10))

                    noise = np.sqrt(noise_pwr/2) * (
                        np.random.randn(len(sig), num_samples) +
                        1j * np.random.randn(len(sig), num_samples)
                    )
                    data = sig[:,None] + noise

                    doa_data.append(DOAData(
                        power_diff_dB=pdB,
                        target1_az=target1_az, target1_el=target1_el,
                        target2_az=az2, target2_el=target2_el,
                        snr_dB=snr, complex_data=data
                    ))
        else:
            for el2 in t2:
                for snr in snr_dB:
                    sv1 = steer(target1_az, target1_el)
                    sv2 = steer(target2_az, el2)
                    sig = target1_amp*sv1 + a2*sv2
                    sig_pwr = np.mean(np.abs(sig)**2)
                    noise_pwr = sig_pwr * (10**(-snr/10))

                    noise = np.sqrt(noise_pwr/2) * (
                        np.random.randn(len(sig), num_samples) +
                        1j * np.random.randn(len(sig), num_samples)
                    )
                    data = sig[:,None] + noise

                    doa_data.append(DOAData(
                        power_diff_dB=pdB,
                        target1_az=target1_az, target1_el=target1_el,
                        target2_az=target2_az, target2_el=el2,
                        snr_dB=snr, complex_data=data
                    ))

    # 保存
    name = 'doa_simulation_data_direct_azi.npz' if aziFlag else 'doa_simulation_data_direct_ele.npz'
    dicts = [asdict(d) for d in doa_data]
    np.savez(name, doa_data=dicts)
    savemat(name.replace('.npz','.mat'), {'doa_data': dicts})
    print(f"保存成功：{name}")
    print(f"总数据量：{len(doa_data)}")
    return doa_data

# ==========================
# 主脚本
# ==========================
if __name__ == '__main__':
    # 阵列配置
    virtual_array = layoutCfg()
    Array_Azi, AziIdx_Select, Array_Ele, EleIdx_Select= Array_Seprate(virtual_array)
    print("=" * 50)
    print("AziIdx_Select (shape: {}) :".format(AziIdx_Select.shape), AziIdx_Select)
    print("Array_Azi (shape: {}) :\n".format(Array_Azi.shape), Array_Azi)
    print("EleIdx_Select (shape: {}) :".format(EleIdx_Select.shape), EleIdx_Select)
    print("Array_Ele (shape: {}) :\n".format(Array_Ele.shape), Array_Ele)
    print("=" * 50)
    # 仿真开关
    aziFlag = 1
    genFlag = 1

    if genFlag:
        doa_data = genDoaSimData(virtual_array, aziFlag)
    else:
        # 加载（可选）
        pass

    # 测角循环
    for d in doa_data:
        cmplx = d['complex_data'] if isinstance(d,dict) else d.complex_data
        snr = d['snr_dB'] if isinstance(d,dict) else d.snr_dB
        noise_level = -snr

        # 前2帧
        snap_data = cmplx[:,1]
        peakInfo = doa_proc.doa_main(snap_data, virtual_array, Array_Azi, AziIdx_Select, Array_Ele, EleIdx_Select,noise_level)
        print(peakInfo)