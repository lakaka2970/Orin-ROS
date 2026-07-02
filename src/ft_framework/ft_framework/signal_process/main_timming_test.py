"""
主程序入口 — GPU 版本参数对齐 rsp_cuda, 用于单独验证信号处理流水线。
"""

import time
import numpy as np
import torch
import matplotlib.pyplot as plt

from config import RadarConfig
from data_io import readRawBinCasc
from preprocessing import radar_signal_process_final
from doppler import doppler_processing_gpu
from peak_detection import peak_search_gpu
from doa_proc import doa_main_batch, doa_main_ultra_separated, doa_env

DEVICE = torch.device('cuda')


def main():
    cfg = RadarConfig()

    # ---- 天线阵列定义（与 rsp_cuda 一致）----
    AzmChUse = np.array([0,1,2,8,9,10,11,12,13,14,15,16,17,18,24,25,26,27,28,31,32,33,34,40,
                         42,43,45,46,48,49,50,56,58,59,61,62,64,65,66,72,74,75,77,78,80,81,82,
                         88,90,91,93,94,96,97,98,106,107,110,112,113,114,126,208,216,217,218,219,
                         220,221,224,232,234,235,236,237,248,249,251], dtype=np.int64)
    AzmPosUse = np.array([34,29,24,76,70,62,57,52,48,39,43,38,33,28,80,74,66,61,56,47,30,25,20,
                          72,58,53,44,35,26,21,16,68,54,49,40,31,22,17,12,64,50,45,36,27,18,13,8,
                          60,46,41,32,23,14,9,4,42,37,19,10,5,0,15,63,105,99,91,86,81,77,55,97,83,
                          78,73,69,90,84,71], dtype=np.float32)
    ElvChUse = np.array([43,130,131,132,133,134,135,146,147,148,149,150,151,162,163,164,165,166,167,
                         178,179,180,181,182,183,194,195,196,197,198,199,211,212,213,214,215], dtype=np.int64)
    ElvPosUse = np.array([40,74,66,58,50,34,42,81,73,65,57,41,49,67,59,51,43,27,35,54,46,38,30,
                          14,22,47,39,31,23,7,15,32,24,16,0,8], dtype=np.float32)

    n_azi = len(AzmPosUse)
    n_ele = len(ElvPosUse)
    Array_Azi = np.zeros((3, n_azi), dtype=np.float32)
    Array_Azi[0, :] = AzmPosUse
    Array_Ele = np.zeros((3, n_ele), dtype=np.float32)
    Array_Ele[1, :] = ElvPosUse

    # GPU tensor 版本
    AziIdx_Select_gpu = torch.from_numpy(AzmChUse).to(DEVICE)
    EleIdx_Select_gpu = torch.from_numpy(ElvChUse).to(DEVICE)
    Array_Azi_gpu = torch.from_numpy(Array_Azi).to(DEVICE)
    Array_Ele_gpu = torch.from_numpy(Array_Ele).to(DEVICE)

    # 初始化 DOA 环境
    if not doa_env.is_initialized:
        doa_env.prepare_mapping_indices(Array_Azi, Array_Ele)
        doa_env.cache_selection_indices(AziIdx_Select_gpu, EleIdx_Select_gpu)

    # ----- 1. 读取数据 -----
    raw_data = readRawBinCasc(
        ".", frameNr=0,
        nSamples=cfg.n_samples, nRamps=cfg.n_chirps, nChannels=cfg.n_rx)
    print(f"原始数据形状: {raw_data.shape}")

    # ----- 2. GPU 预热 -----
    for _ in range(3):
        cube, dc, _ = radar_signal_process_final(
            raw_data, cfg.n_samples, cfg.n_rx, cfg.n_chirps,
            threshold_scale=cfg.threshold_scale)
        rd, rx, noise, vch, ms_idx, mv = doppler_processing_gpu(
            cube, cfg.n_rx, cfg.n_chirps, cube.shape[2],
            cfg.tx_ddma_idx, cfg.n_subbands, cfg.noise_est_ratio)
        peaks = peak_search_gpu(
            rd, mv, ms_idx, rx, noise,
            cfg.tx_ddma_idx, cube.shape[2], cfg.n_chirps, cfg.n_subbands,
            ps_scale=cfg.ps_scale, max_peaks_per_rb=cfg.max_peaks_per_rb,
            max_total_peaks=cfg.max_total_peaks)
        if peaks:
            ch_batch = torch.stack([p['channel_gpu'] for p in peaks])
            doa_main_batch(ch_batch, 28.0)
    torch.cuda.synchronize()

    # ----- 3. 正式处理 + 计时 -----
    t0 = time.perf_counter()

    cube, dc, _ = radar_signal_process_final(
        raw_data, cfg.n_samples, cfg.n_rx, cfg.n_chirps,
        threshold_scale=cfg.threshold_scale)
    rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci = \
        doppler_processing_gpu(
            cube, cfg.n_rx, cfg.n_chirps, cube.shape[2],
            cfg.tx_ddma_idx, cfg.n_subbands, cfg.noise_est_ratio)
    peaks = peak_search_gpu(
        rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise_est,
        cfg.tx_ddma_idx, cube.shape[2], cfg.n_chirps, cfg.n_subbands,
        ps_scale=cfg.ps_scale, max_peaks_per_rb=cfg.max_peaks_per_rb,
        max_total_peaks=cfg.max_total_peaks)

    # ====== 4a. 新方法: 批量 DOA (doa_main_batch) ======
    doa_threshold_db = 28.0
    range_res = cfg.range_resolution
    doppler_res = cfg.doppler_resolution
    ambgt = cfg.ambgt

    t_batch = time.perf_counter()
    channel_batch = torch.stack([p['channel_gpu'] for p in peaks])  # (N, 256)
    # Debug: 找出 rb==16, db==63 的 peak 索引
    debug_idx = [i for i, p in enumerate(peaks) if p['rb'] == 16 and p['db'] == 63]
    all_azi, all_ele, azi_snr, ele_snr = doa_main_batch(
        channel_batch, doa_threshold_db, debug_indices=debug_idx)
    torch.cuda.synchronize()
    t_batch = (time.perf_counter() - t_batch) * 1000.0

    points_batch = []
    for i, peak in enumerate(peaks):
        azi_results = all_azi[i]
        ele_results = all_ele[i]
        if len(azi_results) == 0 or len(ele_results) == 0:
            continue

        rb, db = peak['rb'], peak['db']
        rng = rb * range_res
        vel = db * doppler_res
        pow_linear = peak['f32PeakPowVchNci_Q7dB']
        noise_val = peak['noise']
        snr_db = (20.0 * np.log10(pow_linear / noise_val) if noise_val > 0 else 0.0)
        rcs_db = 20.0 * np.log10(pow_linear)

        valid_azi = [t for t in azi_results if t[0] == 1]
        valid_ele = [t for t in ele_results if t[0] == 1]
        valid_azi.sort(key=lambda t: t[2], reverse=True)
        valid_ele.sort(key=lambda t: t[2], reverse=True)

        if len(valid_azi) == 2 and len(valid_ele) == 2:
            pairs = [(valid_azi[0], valid_ele[0]), (valid_azi[1], valid_ele[1])]
        else:
            pairs = [(a, e) for a in valid_azi for e in valid_ele]

        n_pairs = len(pairs)
        for p_idx, (a_target, e_target) in enumerate(pairs):
            azi_rad, ele_rad = np.deg2rad(a_target[4]), np.deg2rad(e_target[4])
            x = rng * np.cos(ele_rad) * np.cos(azi_rad)
            y = rng * np.cos(ele_rad) * np.sin(azi_rad)
            z = rng * np.sin(ele_rad)

            points_batch.append({
                'x': x, 'y': y, 'z': z,
                'range': rng,
                'azimuth': azi_rad, 'elevation': ele_rad,
                'rcs': rcs_db, 'snr': snr_db, 'power_db': rcs_db,
                'ambgt': ambgt,
                'exist_prob': 100, 'multi_tgt_prob': 100, 'ambgt_prob': 100,
                'raw_doppler': vel,
                'doppler_idx': int(db),
                'azimuth_idx': int(a_target[1]),
                'elevation_idx': int(e_target[1]),
                'obj_same_rv': (p_idx + 1) if n_pairs > 1 else 0,
                'rd_cell_idx': 0, 'range_idx': rb,
                'peak_val': int(np.clip(pow_linear, 0, 65535)),
                'sin_azim_snr_lin': int(azi_snr[i]) if i < len(azi_snr) else 0,
                'sin_elev_snr_lin': int(ele_snr[i]) if i < len(ele_snr) else 0,
                'vel_amb_fac': 0, 'det_ambig_state': 0, 'det_motion_pat': 0,
            })

    # ====== 4b. 旧方法: 逐峰 DOA (doa_main_ultra_separated) ======
    t_sep = time.perf_counter()
    points_sep = []
    for peak in peaks:
        rb, db = peak['rb'], peak['db']
        channel_data = peak['channel_gpu']

        azi_results, ele_results = doa_main_ultra_separated(
            channel_data, Array_Azi_gpu, AziIdx_Select_gpu,
            Array_Ele_gpu, EleIdx_Select_gpu,rb, db, doa_threshold_db)

        if azi_results.shape[0] == 0 or ele_results.shape[0] == 0:
            continue

        rng = rb * range_res
        vel = db * doppler_res
        pow_linear = peak['f32PeakPowVchNci_Q7dB']
        noise_val = peak['noise']
        snr_db = (20.0 * np.log10(pow_linear / noise_val) if noise_val > 0 else 0.0)
        rcs_db = 20.0 * np.log10(pow_linear)

        for a_target in azi_results:
            if a_target[0] != 1:
                continue
            for e_target in ele_results:
                if e_target[0] != 1:
                    continue
                azi_rad, ele_rad = np.deg2rad(a_target[4]), np.deg2rad(e_target[4])
                x = rng * np.cos(ele_rad) * np.cos(azi_rad)
                y = rng * np.cos(ele_rad) * np.sin(azi_rad)
                z = rng * np.sin(ele_rad)

                points_sep.append({
                    'x': x, 'y': y, 'z': z,
                    'range': rng,
                    'azimuth': azi_rad, 'elevation': ele_rad,
                    'rcs': rcs_db, 'snr': snr_db, 'power_db': rcs_db,
                    'ambgt': ambgt,
                    'exist_prob': 100, 'multi_tgt_prob': 100, 'ambgt_prob': 100,
                    'raw_doppler': vel,
                    'doppler_idx': int(db),
                    'azimuth_idx': int(a_target[1]),
                    'elevation_idx': int(e_target[1]),
                    'obj_same_rv': 0,
                    'rd_cell_idx': 0, 'range_idx': rb,
                    'peak_val': int(np.clip(pow_linear, 0, 65535)),
                    'sin_azim_snr_lin': 0,
                    'sin_elev_snr_lin': 0,
                    'vel_amb_fac': 0, 'det_ambig_state': 0, 'det_motion_pat': 0,
                })
    torch.cuda.synchronize()
    t_sep = (time.perf_counter() - t_sep) * 1000.0

    t_total = (time.perf_counter() - t0) * 1000.0

    # ---- 对比输出 ----
    print(f"\n{'='*60}")
    print(f"峰值数: {len(peaks)}")
    print(f"新方法 (批量 batch):   {len(points_batch):4d} 个点云,  DOA 耗时 {t_batch:.1f} ms")
    print(f"旧方法 (逐峰 separated): {len(points_sep):4d} 个点云,  DOA 耗时 {t_sep:.1f} ms")
    print(f"GPU 总耗时: {t_total:.1f} ms")
    print(f"{'='*60}\n")

    # ---- 保存 PCD ----
    timestamp_us = int(time.time() * 1e6)
    if points_batch:
        _save_pcd(points_batch, f"radar_pointcloud_batch_{timestamp_us}.pcd")
    if points_sep:
        _save_pcd(points_sep, f"radar_pointcloud_sep_{timestamp_us}.pcd")


def _save_pcd(points, filepath):
    """保存为 PCD v0.7 ASCII 格式 (22 字段)。"""
    fields = [
        ('u32TimeStamp',     4, 'U'),
        ('u16FrameID',       2, 'U'),
        ('u16DetObjNum',     2, 'U'),
        ('f32XPos',          4, 'F'),
        ('f32YPos',          4, 'F'),
        ('f32ZPos',          4, 'F'),
        ('f32Range',         4, 'F'),
        ('f32Speed',         4, 'F'),
        ('f32AzimuthAng',    4, 'F'),
        ('f32EleAng',        4, 'F'),
        ('f32SNRdB',         4, 'F'),
        ('f32RcsdB',         4, 'F'),
        ('f32PowerdB',       4, 'F'),
        ('u32ObjSameRV',     4, 'U'),
        ('u16RdCellIdx',     2, 'U'),
        ('u16RangeIdx',      2, 'U'),
        ('u16DopplerIdx',    2, 'U'),
        ('u8AzimuthIdx',     1, 'U'),
        ('u8ElevationIdx',   1, 'U'),
        ('u16PeakVal',       2, 'U'),
        ('u16SinAzimSNRLin', 2, 'U'),
        ('u16SinElevSNRLin', 2, 'U'),
    ]
    fields_str = ' '.join(f[0] for f in fields)
    size_str   = ' '.join(str(f[1]) for f in fields)
    type_str   = ' '.join(f[2] for f in fields)
    n = len(points)

    keys = ['x', 'y', 'z', 'range', 'raw_doppler', 'azimuth', 'elevation',
            'snr', 'rcs', 'power_db', 'obj_same_rv', 'rd_cell_idx', 'range_idx',
            'doppler_idx', 'azimuth_idx', 'elevation_idx', 'peak_val',
            'sin_azim_snr_lin', 'sin_elev_snr_lin']

    with open(filepath, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                f"FIELDS {fields_str}\n"
                f"SIZE {size_str}\n"
                f"TYPE {type_str}\n"
                "COUNT " + " ".join("1" for _ in fields) + "\n"
                f"WIDTH {n}\nHEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {n}\nDATA ascii\n")
        for p in points:
            row = (
                f"0 0 {n} "   # timestamp, frame_id, det_obj_num
                f"{p['x']:.6f} {p['y']:.6f} {p['z']:.6f} "
                f"{p['range']:.6f} {abs(p['raw_doppler']):.6f} "
                f"{p['azimuth']:.6f} {p['elevation']:.6f} "
                f"{p['snr']:.6f} {p['rcs']:.6f} {p.get('power_db', 0):.6f} "
                f"{p.get('obj_same_rv', 0)} "
                f"{p.get('rd_cell_idx', 0)} {p.get('range_idx', 0)} {p.get('doppler_idx', 0)} "
                f"{p.get('azimuth_idx', 0)} {p.get('elevation_idx', 0)} "
                f"{p.get('peak_val', 0)} "
                f"{p.get('sin_azim_snr_lin', 0)} {p.get('sin_elev_snr_lin', 0)}\n"
            )
            f.write(row)
    print(f"PCD 已保存: {filepath}")


if __name__ == "__main__":
    main()
