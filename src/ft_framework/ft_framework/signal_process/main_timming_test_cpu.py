"""
主程序：完整雷达信号处理流程 + 点云生成与计时
"""
import os
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

    # ---- 天线阵列定义（用户指定）----
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
    Array_Azi_test = np.zeros((3, n_azi), dtype=np.float32)
    Array_Azi_test[0, :] = AzmPosUse                                  # AzmPosUse → Array_Azi[0, :]
    Array_Ele_test = np.zeros((3, n_ele), dtype=np.float32)
    Array_Ele_test[1, :] = ElvPosUse                                  # ElvPosUse → Array_Ele[1, :]
    # ------------------------------------------

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
            Array_Azi_test, AzmChUse,
            Array_Ele_test, ElvChUse, 0, 0,0
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
    doa_threshold_db = 3.0

    # 确保DOA环境已初始化
    if not doa_env.is_initialized:
        doa_env.prepare_mapping_indices(Array_Azi_test, Array_Ele_test)

    points = []
    for peak in peaks:
        rb = peak['rb']
        db = peak['db']
        channel_data = peak['channel']           # 形状 (256,)

        azi_results, ele_results = doa_main_ultra_separated(
            channel_data,
            Array_Azi_test, AzmChUse,
            Array_Ele_test, ElvChUse, rb, db,
            doa_threshold_db
        )
        if len(azi_results) == 0 or len(ele_results) == 0:
            continue

        rng = rb * range_res
        vel = (db - rb*4)* doppler_res
        pow_linear = peak['pow_vch']
        snr_db = 10 * np.log10(pow_linear / peak['noise']) if peak['noise'] > 0 else 0.0
        rcs_db = 10 * np.log10(pow_linear)

        # 筛选有效目标 (flag==1)，按能量降序排列
        valid_azi = [t for t in azi_results if t[0] == 1]
        valid_ele = [t for t in ele_results if t[0] == 1]
        valid_azi.sort(key=lambda t: t[2], reverse=True)   # t[2]=mag_db
        valid_ele.sort(key=lambda t: t[2], reverse=True)

        if len(valid_azi) == 2 and len(valid_ele) == 2:
            # 两组：能量强配强、弱配弱
            pairs = [(valid_azi[0], valid_ele[0]),
                     (valid_azi[1], valid_ele[1])]
        else:
            # 其他情况：笛卡尔积
            pairs = [(a, e) for a in valid_azi for e in valid_ele]

        for a_target, e_target in pairs:
            azi_deg = a_target[4]
            azi_rad = np.deg2rad(azi_deg)
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