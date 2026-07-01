"""
主程序：完整雷达信号处理流程 + 点云生成与计时
"""
import time
import numpy as np
from config import RadarConfig
from data_io import readRawBinCasc
from preprocessing_cpu import radar_signal_process_final
from doppler_cpu import doppler_processing_numpy
from peak_detection_cpu import peak_search_numpy
from arraySim_cpu import RadarArrayInitializer
from doa_proc_cpu import doa_main_ultra_separated, doa_env
from plotting_cpu import (plot_range_profile, plot_range_chirp_energy,
                          plot_rd_cube, plot_rx_nci, plot_noise_estimation,
                          plot_vch_nci, plot_max_subband, plot_peaks_on_vch,
                          plot_adc_chirp, plot_single_chirp_range)


def save_pointcloud_pcd(points: list, filename: str, timestamp_us: int) -> None:
    """将点云保存为PCD文件（ASCII格式）"""
    if not points:
        print("没有点可保存")
        return
    pcd_path = f"{filename}_{timestamp_us}.pcd"
    with open(pcd_path, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        f.write("FIELDS x y z range azimuth elevation RCS SNR ambgt exist_prob multi_tgt_prob ambgt_prob raw_doppler idx\n")
        f.write("SIZE 4 4 4 4 4 4 4 4 4 1 1 1 4 1\n")
        f.write("TYPE F F F F F F F F F U U U F U\n")
        f.write("COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\nDATA ascii\n")
        for p in points:
            f.write(
                f"{p['x']:.6f} {p['y']:.6f} {p['z']:.6f} "
                f"{p['range']:.6f} {p['azimuth']:.6f} {p['elevation']:.6f} "
                f"{p['RCS']:.2f} {p['SNR']:.2f} {p['ambgt']:.3f} "
                f"{p['exist_prob']} {p['multi_tgt_prob']} {p['ambgt_prob']} "
                f"{p['raw_doppler']:.3f} {p['idx']}\n"
            )
    print(f"PCD保存: {pcd_path}")


def save_pointcloud_csv(points: list, filename: str, timestamp_us: int) -> None:
    """将点云保存为CSV文件"""
    if not points:
        return
    csv_path = f"{filename}_{timestamp_us}.csv"
    with open(csv_path, 'w') as f:
        f.write("x,y,z,range,azimuth,elevation,RCS,SNR,ambgt,exist_prob,multi_tgt_prob,ambgt_prob,raw_doppler,idx\n")
        for p in points:
            f.write(
                f"{p['x']:.6f},{p['y']:.6f},{p['z']:.6f},"
                f"{p['range']:.6f},{p['azimuth']:.6f},{p['elevation']:.6f},"
                f"{p['RCS']:.2f},{p['SNR']:.2f},{p['ambgt']:.3f},"
                f"{p['exist_prob']},{p['multi_tgt_prob']},{p['ambgt_prob']},"
                f"{p['raw_doppler']:.3f},{p['idx']}\n"
            )
    print(f"CSV保存: {csv_path}")


def main():
    cfg = RadarConfig()
    array_env = RadarArrayInitializer()

    total_start = time.perf_counter()

    # ----- 读取数据 -----
    raw_data = readRawBinCasc(
        ".",
        frameNr=0,
        nSamples=cfg.n_samples,
        nRamps=cfg.n_chirps,
        nChannels=cfg.n_rx
    )
    print(f"原始数据形状: {raw_data.shape}")

    # ----- 预热（运行一次，确保缓存就绪）-----
    cube, _, _ = radar_signal_process_final(
        raw_data, cfg.n_samples, cfg.n_rx, cfg.n_chirps, cfg.threshold_scale
    )
    rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci = doppler_processing_numpy(
        cube, cfg.n_rx, cfg.n_chirps, cube.shape[2],
        cfg.tx_ddma_idx, cfg.n_subbands, cfg.noise_est_ratio
    )
    peaks_warm = peak_search_numpy(
        rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise_est,
        cfg.tx_ddma_idx, cube.shape[2], cfg.n_chirps, cfg.n_subbands,
        cfg.ps_scale, cfg.max_peaks_per_rb, cfg.max_total_peaks
    )
    if peaks_warm:
        # 预热DOA环境（准备映射）
        doa_main_ultra_separated(
            peaks_warm[0]['channel'],
            array_env.Array_Azi, array_env.AziIdx_Select,
            array_env.Array_Ele, array_env.EleIdx_Select
        )

    # ----- 正式计时处理 -----
    t_start = time.perf_counter()

    cube, dc_est, _ = radar_signal_process_final(
        raw_data, cfg.n_samples, cfg.n_rx, cfg.n_chirps, cfg.threshold_scale
    )
    rd_cube, rx_nci, noise_est, vch_nci, max_subband_idx, max_vch_nci = doppler_processing_numpy(
        cube, cfg.n_rx, cfg.n_chirps, cube.shape[2],
        cfg.tx_ddma_idx, cfg.n_subbands, cfg.noise_est_ratio
    )
    peaks = peak_search_numpy(
        rd_cube, max_vch_nci, max_subband_idx, rx_nci, noise_est,
        cfg.tx_ddma_idx, cube.shape[2], cfg.n_chirps, cfg.n_subbands,
        cfg.ps_scale, cfg.max_peaks_per_rb, cfg.max_total_peaks
    )

    # ----- 中间结果绘图 -----
    if cfg.enable_plots:
        import os as _os
        _save_dir = cfg.plot_save_dir if cfg.save_plots else None
        if _save_dir:
            _os.makedirs(_save_dir, exist_ok=True)

        if cfg.plot_adc:
            plot_adc_chirp(raw_data, chirp_idx=1, rx_idx=2,
                           save_path=_os.path.join(_save_dir, "adc_chirp.png") if _save_dir else None)
        if cfg.plot_range_profile:
            plot_range_profile(cube, rx_idx=0,
                               save_path=_os.path.join(_save_dir, "range_profile.png") if _save_dir else None)
            plot_single_chirp_range(cube, rx_idx=0, chirp_idx=0,
                                    save_path=_os.path.join(_save_dir, "single_chirp_range.png") if _save_dir else None)
        if cfg.plot_range_chirp:
            plot_range_chirp_energy(cube, save_path=_os.path.join(_save_dir, "range_chirp.png") if _save_dir else None)
        if cfg.plot_rd_cube:
            plot_rd_cube(rd_cube, rx_idx=0, vmin=None,
                               save_path=_os.path.join(_save_dir, "rd_cube.png") if _save_dir else None)
        if cfg.plot_rx_nci:
            plot_rx_nci(rx_nci, save_path=_os.path.join(_save_dir, "rx_nci.png") if _save_dir else None)
        if cfg.plot_noise:
            plot_noise_estimation(noise_est, save_path=_os.path.join(_save_dir, "noise_est.png") if _save_dir else None)
        if cfg.plot_vch_nci:
            plot_vch_nci(vch_nci, save_path=_os.path.join(_save_dir, "vch_nci.png") if _save_dir else None)
        if cfg.plot_max_subband:
            plot_max_subband(max_vch_nci, max_subband_idx, save_path=_os.path.join(_save_dir, "max_subband.png") if _save_dir else None)
        if cfg.plot_peaks and peaks:
            plot_peaks_on_vch(vch_nci, peaks, save_path=_os.path.join(_save_dir, "peaks.png") if _save_dir else None)

    # ----- 点云生成 -----
    range_res = cfg.range_resolution
    doppler_res = cfg.doppler_resolution
    ambgt = cfg.ambgt
    doa_threshold_db = 28.0

    # 确保DOA环境已初始化
    if not doa_env.is_initialized:
        doa_env.prepare_mapping_indices(array_env.Array_Azi, array_env.Array_Ele)

    points = []
    for peak in peaks:
        rb = peak['rb']
        db = peak['db']
        channel_data = peak['channel']           # 形状 (256,)

        azi_results, ele_results = doa_main_ultra_separated(
            channel_data,
            array_env.Array_Azi, array_env.AziIdx_Select,
            array_env.Array_Ele, array_env.EleIdx_Select,
            doa_threshold_db
        )
        if len(azi_results) == 0 or len(ele_results) == 0:
            continue

        rng = rb * range_res
        vel = db * doppler_res
        pow_linear = peak['pow_vch']
        snr_db = 10 * np.log10(pow_linear / peak['noise']) if peak['noise'] > 0 else 0.0
        rcs_db = 10 * np.log10(pow_linear)

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

                # 直角坐标
                x = rng * np.cos(ele_rad) * np.cos(azi_rad)
                y = rng * np.cos(ele_rad) * np.sin(azi_rad)
                z = rng * np.sin(ele_rad)

                point = {
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
                }
                points.append(point)

    timestamp_us = int(time.time() * 1e6)
    t_end = time.perf_counter()

    print(f"纯NumPy运算耗时: {(t_end - t_start) * 1000:.2f} ms")
    print(f"检测到的峰值数: {len(peaks)}")

    if points:
        save_pointcloud_pcd(points, "radar_pointcloud", timestamp_us)
        save_pointcloud_csv(points, "radar_pointcloud", timestamp_us)
    else:
        print("DOA后无有效点，跳过保存")


if __name__ == "__main__":
    main()