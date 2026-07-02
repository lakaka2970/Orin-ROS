"""
主程序入口：完整雷达信号处理流水线，包含数据读取、距离处理、多普勒处理、
峰值检测、DOA 估计，最终生成点云并保存为 PCD/CSV 文件。
"""

import time
import numpy as np
import torch

from config import RadarConfig
from data_io import readRawBinCasc
from preprocessing import radar_signal_process_final
from doppler import doppler_processing_gpu
from peak_detection import peak_search_gpu
from arraySim import RadarArrayInitializer
from doa_proc import doa_main_ultra_separated, doa_env
from plotting import (plot_range_profile, plot_range_chirp_energy,
                      plot_rd_cube, plot_rx_nci, plot_noise_estimation,
                      plot_vch_nci, plot_max_subband, plot_peaks_on_vch)

DEVICE = torch.device('cuda')


def save_pointcloud_pcd(points: list, filename: str, timestamp_us: int):
    """将点云保存为 PCD (Point Cloud Data) 格式 (ASCII)。"""
    if not points:
        print("无有效点云，跳过 PCD 保存")
        return
    pcd_path = f"{filename}_{timestamp_us}.pcd"
    with open(pcd_path, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z range azimuth elevation RCS SNR ambgt exist_prob multi_tgt_prob ambgt_prob raw_doppler idx\n")
        f.write("SIZE 4 4 4 4 4 4 4 4 4 1 1 1 4 1\n")
        f.write("TYPE F F F F F F F F F U U U F U\n")
        f.write("COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p['x']:.6f} {p['y']:.6f} {p['z']:.6f} "
                    f"{p['range']:.6f} {p['azimuth']:.6f} {p['elevation']:.6f} "
                    f"{p['RCS']:.2f} {p['SNR']:.2f} {p['ambgt']:.3f} "
                    f"{p['exist_prob']} {p['multi_tgt_prob']} {p['ambgt_prob']} "
                    f"{p['raw_doppler']:.3f} {p['idx']}\n")
    print(f"PCD 已保存: {pcd_path}")


def save_pointcloud_csv(points: list, filename: str, timestamp_us: int):
    """将点云保存为 CSV 格式。"""
    if not points:
        return
    csv_path = f"{filename}_{timestamp_us}.csv"
    with open(csv_path, 'w') as f:
        f.write("x,y,z,range,azimuth,elevation,RCS,SNR,ambgt,exist_prob,multi_tgt_prob,ambgt_prob,raw_doppler,idx\n")
        for p in points:
            f.write(f"{p['x']:.6f},{p['y']:.6f},{p['z']:.6f},"
                    f"{p['range']:.6f},{p['azimuth']:.6f},{p['elevation']:.6f},"
                    f"{p['RCS']:.2f},{p['SNR']:.2f},{p['ambgt']:.3f},"
                    f"{p['exist_prob']},{p['multi_tgt_prob']},{p['ambgt_prob']},"
                    f"{p['raw_doppler']:.3f},{p['idx']}\n")
    print(f"CSV 已保存: {csv_path}")


def main():
    """主处理流程。"""
    cfg = RadarConfig()
    array_env = RadarArrayInitializer(use_gpu=True)

    total_start = time.perf_counter()

    # ----- 1. 读取二进制数据 -----
    raw_data = readRawBinCasc(
        ".",
        frameNr=0,
        nSamples=cfg.n_samples,
        nRamps=cfg.n_chirps,
        nChannels=cfg.n_rx
    )
    print(f"原始数据形状: {raw_data.shape}")

    # ----- 2. GPU 预热（执行三次，消除首次编译/显存分配开销）-----
    for _ in range(3):
        cube, dc, nof = radar_signal_process_final(
            raw_data, cfg.n_samples, cfg.n_rx, cfg.n_chirps, threshold_scale=cfg.threshold_scale
        )
        rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci = doppler_processing_gpu(
            cube, cfg.n_rx, cfg.n_chirps, 1025, cfg.tx_ddma_idx, cfg.n_subbands, noise_est_ratio=cfg.noise_est_ratio
        )
        peaks = peak_search_gpu(
            rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise_est,
            cfg.tx_ddma_idx, 1025, cfg.n_chirps, cfg.n_subbands,
            ps_scale=cfg.ps_scale, max_peaks_per_rb=cfg.max_peaks_per_rb,
            max_total_peaks=cfg.max_total_peaks
        )
        for peak in peaks:
            channel_data = torch.from_numpy(peak['channel']).to(DEVICE, non_blocking=True)
            _ = doa_main_ultra_separated(
                channel_data,
                array_env.Array_Azi_gpu,
                array_env.AziIdx_Select_gpu,
                array_env.Array_Ele_gpu,
                array_env.EleIdx_Select_gpu
            )
    torch.cuda.synchronize()

    # ----- 3. 正式处理并计时 -----
    t_start = time.perf_counter()

    # 距离处理 + 多普勒处理 + 峰值检测
    cube, dc, nof = radar_signal_process_final(
        raw_data, cfg.n_samples, cfg.n_rx, cfg.n_chirps, threshold_scale=cfg.threshold_scale
    )
    rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci = doppler_processing_gpu(
        cube, cfg.n_rx, cfg.n_chirps, 1025, cfg.tx_ddma_idx, cfg.n_subbands, noise_est_ratio=cfg.noise_est_ratio
    )
    peaks = peak_search_gpu(
        rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise_est,
        cfg.tx_ddma_idx, 1025, cfg.n_chirps, cfg.n_subbands,
        ps_scale=cfg.ps_scale, max_peaks_per_rb=cfg.max_peaks_per_rb,
        max_total_peaks=cfg.max_total_peaks
    )

    # ----- 4. DOA 估计与点云生成 -----

    doa_threshold_db = 28.0
    range_res = cfg.range_resolution
    doppler_res = cfg.doppler_resolution
    ambgt = cfg.ambgt

    points = []
    doa_env.prepare_mapping_indices(array_env.Array_Azi_gpu, array_env.Array_Ele_gpu)

    for peak in peaks:
        rb = peak['rb']
        db = peak['db']
        channel_data = torch.from_numpy(peak['channel']).to(DEVICE, non_blocking=True)

        azi_results, ele_results = doa_main_ultra_separated(
            channel_data, array_env.Array_Azi_gpu, array_env.AziIdx_Select_gpu, array_env.Array_Ele_gpu, array_env.EleIdx_Select_gpu, doa_threshold_db
        )

        if azi_results.shape[0] == 0 or ele_results.shape[0] == 0:
            continue

        rng = rb * range_res
        vel = db * doppler_res
        pow_linear = peak['f32PeakPowVchNci_Q7dB']
        snr_db = 10 * np.log10(pow_linear / peak['noise']) if peak['noise'] > 0 else 0
        rcs_db = 10 * np.log10(pow_linear)

        # 交叉配对方位与俯仰（简单笛卡尔积）
        for a_target in azi_results:
            if a_target[0] != 1:
                continue
            azi_deg = a_target[4]
            azi_rad = np.deg2rad(azi_deg)
            for e_target in ele_results:
                if e_target[0] != 1:
                    continue
                ele_deg = e_target[4]
                ele_rad = np.deg2rad(ele_deg)

                x = rng * np.cos(ele_rad) * np.cos(azi_rad)
                y = rng * np.cos(ele_rad) * np.sin(azi_rad)
                z = rng * np.sin(ele_rad)

                points.append({
                    'x': x, 'y': y, 'z': z,
                    'range': rng,
                    'azimuth': azi_rad,
                    'elevation': ele_rad,
                    'RCS': rcs_db,
                    'SNR': snr_db,
                    'ambgt': ambgt,
                    'exist_prob': 100,
                    'multi_tgt_prob': 100,
                    'ambgt_prob': 100,
                    'raw_doppler': vel,
                    'idx': 128 if vel != 0 else 0
                })

    timestamp_us = int(time.time() * 1e6)
    torch.cuda.synchronize()
    t_end = time.perf_counter()

    print(f"GPU 纯运算耗时: {(t_end - t_start) * 1000:.2f} ms")
    print(f"检测到 {len(peaks)} 个峰值")

    if points:
        save_pointcloud_pcd(points, "radar_pointcloud", timestamp_us)
        save_pointcloud_csv(points, "radar_pointcloud", timestamp_us)
    else:
        print("DOA 未产生有效点云，跳过保存")


if __name__ == "__main__":
    main()