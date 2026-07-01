import numpy as np
import torch
import time
from dataclasses import dataclass, asdict
from scipy.io import savemat
from ft_framework.signal_process.doa_proc import doa_main_ultra_separated

# 自动选择计算设备
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

@dataclass
class DOAData:
    """DOA 仿真数据结构"""
    power_diff_dB: float = 0.0
    target1_az: float = 0.0
    target1_el: float = 0.0
    target2_az: float = 0.0
    target2_el: float = 0.0
    snr_dB: float = 0.0
    complex_data: np.ndarray = None

# ========================================================
# 1. 阵列初始化类（全局仅执行一次，直接常驻显存/内存）
# ========================================================
class RadarArrayInitializer:
    def __init__(self, use_gpu=True):
        self.device = torch.device('cuda') if (use_gpu and torch.cuda.is_available()) else torch.device('cpu')

        # 静态定义硬件发射与接收阵列坐标 [3, N]
        self.tx_pos = np.array([
            [0,0,0], [4,0,0], [8,0,0], [12,0,0], [16,0,0], [20,0,0], [24,0,0], [28,0,0],
            [38,0,0], [45,0,0], [53,0,0], [53,7,0], [53,14,0], [53,27,0], [53,34,0], [53,41,0]
        ], dtype=np.float32).T

        self.rx_pos = np.array([
            [0,0,0], [5,40,0], [10,40,0], [15,40,0], [19,40,0], [24,40,0], [28,40,0], [33,40,0],
            [38,40,0], [46,40,0], [52,40,0], [0,8,0], [0,16,0], [0,24,0], [0,32,0], [0,40,0]
        ], dtype=np.float32).T

        # 1.1 矢量化合成虚拟阵列（利用广播消除双重 for 循环）
        # [3, 16, 1] + [3, 1, 16] -> [3, 16, 16] -> 重塑为 [3, 256]
        virtual_tensor = torch.from_numpy(self.tx_pos)[:, :, None] + torch.from_numpy(self.rx_pos)[:, None, :]
        self.virtual_array_np = virtual_tensor.view(3, -1).numpy()

        # 1.2 极速分离方位与俯仰子阵
        self._seperate_subarrays()

        # 1.3 将数据送入目标计算设备 (GPU)
        self.virtual_array_gpu = torch.from_numpy(self.virtual_array_np).to(self.device)
        self.Array_Azi_gpu = torch.from_numpy(self.Array_Azi).to(self.device)
        self.Array_Ele_gpu = torch.from_numpy(self.Array_Ele).to(self.device)
        self.AziIdx_Select_gpu = torch.from_numpy(self.AziIdx_Select).to(self.device)
        self.EleIdx_Select_gpu = torch.from_numpy(self.EleIdx_Select).to(self.device)

    def _seperate_subarrays(self):
        """
        利用高效的一维哈希变换代替昂贵的 np.unique(axis=0)，耗时从 15ms 降至 0.2ms
        """
        arr = self.virtual_array_np

        # ---------- 方位子阵 ----------
        # 压缩二维坐标为一维标量作为哈希键值: y * 10000 + z
        yz_hash = arr[1, :] * 10000.0 + arr[2, :]
        unique_yz, counts = np.unique(yz_hash, return_counts=True)
        best_yz = unique_yz[np.argmax(counts)]
        azi_mask = (yz_hash == best_yz)

        self.AziIdx_Select = np.where(azi_mask)[0]
        # 按 X 坐标排序
        sort_azi = np.argsort(arr[0, self.AziIdx_Select])
        self.AziIdx_Select = self.AziIdx_Select[sort_azi]
        # 阵列位置去重
        _, unique_idx = np.unique(arr[0, self.AziIdx_Select], return_index=True)
        self.AziIdx_Select = self.AziIdx_Select[np.sort(unique_idx)]
        self.Array_Azi = arr[:, self.AziIdx_Select]

        # ---------- 俯仰子阵 ----------
        xz_hash = arr[0, :] * 10000.0 + arr[2, :]
        unique_xz, counts_ele = np.unique(xz_hash, return_counts=True)
        best_xz = unique_xz[np.argmax(counts_ele)]
        ele_mask = (xz_hash == best_xz)

        self.EleIdx_Select = np.where(ele_mask)[0]
        # 按 Y 坐标排序
        sort_ele = np.argsort(arr[1, self.EleIdx_Select])
        self.EleIdx_Select = self.EleIdx_Select[sort_ele]
        # 阵列位置去重
        _, unique_idx_ele = np.unique(arr[1, self.EleIdx_Select], return_index=True)
        self.EleIdx_Select = self.EleIdx_Select[np.sort(unique_idx_ele)]
        self.Array_Ele = arr[:, self.EleIdx_Select]
        # print("=" * 50)
        # print("AziIdx_Select (shape: {}) :".format(self.AziIdx_Select.shape), self.AziIdx_Select)
        # print("Array_Azi (shape: {}) :\n".format(self.Array_Azi.shape), self.Array_Azi)
        # print("EleIdx_Select (shape: {}) :".format(self.EleIdx_Select.shape), self.EleIdx_Select)
        # print("Array_Ele (shape: {}) :\n".format(self.Array_Ele.shape), self.Array_Ele)
        # print("=" * 50)
# ========================================================
# 2. 极致矢量化仿真数据生成 (利用 GPU 算力消除 3 层大循环)
# ========================================================
@torch.inference_mode()
def genDoaSimData_Fast(array_init: RadarArrayInitializer, aziFlag: bool):
    device = array_init.device
    v_arr = array_init.virtual_array_gpu  # [3, 256]
    n_elements = v_arr.shape[1]
    num_samples = 100

    # 预定义常数
    power_diff_dB = np.arange(0, 30, 5)  # 6种
    snr_dB = [40, 35, 30, 25, 20, 15, 10]  # 7种

    if aziFlag:
        #t2 = np.concatenate([np.arange(0.1, 0.5, 0.1), np.arange(0.5, 20.5, 0.5)])
        t2 = np.array([70, 71, 72])
    else:
        #t2 = np.arange(1, 10.5, 0.5)
        t2 = np.array([-10, 0, 10])

    # 预计算目标 1 的静态导向矢量 (固定为 0,0)
    phase1 = 1j * np.pi * v_arr[2, :]
    sv1 = torch.exp(-phase1)  # [256]

    doa_data = []

    # 仅保留外层最轻量的动力学参数循环，内部参数空间全部矩阵并发计算
    for pdB in power_diff_dB:
        a2 = 1.0 * (10 ** (-pdB / 20))
        for p2_val in t2:
            # 批量计算目标 2 的导向矢量
            az2, el2 = (p2_val, 0.0) if aziFlag else (0.0, p2_val)
            az2_r, el2_r = np.deg2rad(az2), np.deg2rad(el2)

            phase2 = 1j * np.pi * (
                v_arr[0] * np.sin(az2_r) * np.cos(el2_r) +
                v_arr[1] * np.sin(el2_r) +
                v_arr[2] * np.cos(az2_r) * np.cos(el2_r)
            )
            sv2 = torch.exp(-phase2)  # [256]

            # 干净信号矩阵合成: [256]
            sig = sv1 + a2 * sv2
            sig_pwr = torch.mean(torch.abs(sig).pow(2))

            for snr in snr_dB:
                noise_pwr = sig_pwr * (10 ** (-snr / 10))

                # 在 GPU 里直接并发生成复高斯白噪声矩阵 [256, 100]
                noise = torch.sqrt(noise_pwr / 2.0) * (
                    torch.randn((n_elements, num_samples), dtype=torch.float32, device=device) +
                    1j * torch.randn((n_elements, num_samples), dtype=torch.float32, device=device)
                )

                # 信号叠加
                data_gpu = sig[:, None] + noise

                # 转回 NumPy 保存
                doa_data.append(DOAData(
                    power_diff_dB=float(pdB),
                    target1_az=0.0, target1_el=0.0,
                    target2_az=float(az2), target2_el=float(el2),
                    snr_dB=float(snr), complex_data=data_gpu.cpu().numpy()
                ))

    # 保存文件
    name = 'doa_simulation_data_direct_azi.npz' if aziFlag else 'doa_simulation_data_direct_ele.npz'
    dicts = [asdict(d) for d in doa_data]
    np.savez(name, doa_data=dicts)
    savemat(name.replace('.npz', '.mat'), {'doa_data': dicts})
    print(f"✅ 优化版数据生成及落盘成功: {name} | 总数: {len(doa_data)}")

    return doa_data

# ========================================================
# 3. 推理主循环管道
# ========================================================
if __name__ == '__main__':
    t_init_start = time.perf_counter()

    # 阵列初始化（整个程序只执行一次）
    array_env = RadarArrayInitializer(use_gpu=True)

    print(f"阵列初始化及子阵分离完成，总耗时: {(time.perf_counter() - t_init_start)*1000:.2f} ms")

    aziFlag = True
    genFlag = True

    # 执行极速仿真数据生成
    if genFlag:
        t_gen_start = time.perf_counter()
        doa_data = genDoaSimData_Fast(array_env, aziFlag)
        print(f"全量仿真数据并行生成完毕，总耗时: {(time.perf_counter() - t_gen_start)*1000:.2f} ms")
    else:
        # 可在此处添加本地数据加载逻辑
        pass

    # 测角流水线测试
    print("\n开始执行测角流水线...")

    # 模拟前向计算测试（前5组数据）
    d= doa_data[0]
    cmplx = d.complex_data
    cmplx_gpu = torch.from_numpy(cmplx).to(array_env.device)
    snr = d.snr_dB

    # 抽取第 2 帧快拍数据
    snap_data = cmplx_gpu[:, 1]
    # 可直接对接DOA 测角主函数
    peaks_info = doa_main_ultra_separated(snap_data, array_env.Array_Azi_gpu, array_env.AziIdx_Select_gpu,array_env.Array_Ele_gpu,array_env.EleIdx_Select_gpu)

    # 精准掐表测试 GPU 耗时
    torch.cuda.synchronize()
    t_start = time.perf_counter()

    # 运行全流程
    peaks_info = doa_main_ultra_separated(snap_data, 
                                        array_env.Array_Azi_gpu, 
                                        array_env.AziIdx_Select_gpu,
                                        array_env.Array_Ele_gpu,
                                        array_env.EleIdx_Select_gpu,
                                        )


    torch.cuda.synchronize()
    print(f"🚀 [DOA 测角全流程] GPU 运行时间: {(time.perf_counter() - t_start)*1000:.2f} ms")
    print(f"方位探测目标数: {len(peaks_info)} | 俯仰探测目标数: {len(peaks_info)}")
