"""
RSPS 雷达点云可视化主程序
读取 ctrx0_raw.bin (V4L2 RG12 原始捕获) 并生成雷达点云可视化

用法:
    python RSPS_main.py                          # 默认: 帧0, 单帧
    python RSPS_main.py --frame 100              # 读取第100帧
    python RSPS_main.py --frame 0 --nframes 8    # 拼接8帧 (更多chirp → 更好多普勒分辨率)
    python RSPS_main.py --no-3d                  # 跳过3D图, 仅2D
"""

import argparse
import os
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 注册 '3d' projection

import rlib

# =============================================================================
# 参数配置
# =============================================================================

# V4L2 帧参数 (CTRX sensor, RG12 格式)
V4L2_N_SAMPLES = 2048      # 每行采样点数 (chirp 内采样)
V4L2_N_ROWS = 512          # 帧行数
V4L2_N_RX = 4              # 单颗 CTRX 接收通道数

# 派生参数
N_CHIRPS_PER_FRAME = V4L2_N_ROWS // V4L2_N_RX  # 128 chirps/帧

# 雷达物理参数 (77GHz)
C0 = 299792458.0
FREQ_START = 77e9                          # 起始频率 [Hz]
FREQ_SLOPE = 14.0e12                       # 调频斜率 [Hz/s]
TIME_RAMP_END = 35.0e-6                    # 有效调频时间 [s]
WAVELENGTH = C0 / FREQ_START               # 波长 [m]
BANDWIDTH = FREQ_SLOPE * TIME_RAMP_END     # 带宽 [Hz]
RANGE_RES = C0 / (2 * BANDWIDTH)           # 距离分辨率 [m]
D_RX = WAVELENGTH / 2                      # RX 天线间距 [m]

# 数据文件路径 (相对于脚本所在目录)
DATA_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BIN_FILE = os.path.join(DATA_DIR, "output", "ctrx0_raw.bin")

# 备用路径: 脚本同目录下的 output/
if not os.path.exists(DEFAULT_BIN_FILE):
    DEFAULT_BIN_FILE = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "output", "ctrx0_raw.bin"
    )


# =============================================================================
# 可视化函数
# =============================================================================

def plot_nci_heatmap(NCI, peaks_2d=None, title="NCI Range-Doppler Map", save_path=None):
    """绘制 NCI 距离-多普勒热力图，可选叠加峰值点"""
    NCI_dB = 10 * np.log10(NCI + 1e-10)
    Ndopp, Nrang = NCI.shape

    # 坐标轴转换为物理单位
    range_axis = np.arange(Nrang) * RANGE_RES
    doppler_axis = np.arange(Ndopp)

    fig, ax = plt.subplots(figsize=(12, 8))
    im = ax.imshow(
        NCI_dB,
        aspect='auto',
        origin='lower',
        extent=[range_axis[0], range_axis[-1], doppler_axis[0], doppler_axis[-1]],
        cmap='jet'
    )
    plt.colorbar(im, ax=ax, label='Energy [dB]')

    if peaks_2d and len(peaks_2d) > 0:
        peak_ranges = [p[0] * RANGE_RES for p in peaks_2d]
        peak_dopplers = [p[1] for p in peaks_2d]
        ax.scatter(peak_ranges, peak_dopplers, c='white', marker='x', s=30,
                   linewidths=0.8, label=f'{len(peaks_2d)} peaks')

    ax.set_xlabel('Range [m]')
    ax.set_ylabel('Doppler Bin')
    ax.set_title(title)
    if peaks_2d:
        ax.legend()
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"热力图已保存: {save_path}")
    return fig


def plot_nci_3d_surface(NCI, title="NCI 3D Surface", save_path=None):
    """绘制 NCI 3D 曲面图"""
    NCI_dB = 10 * np.log10(NCI + 1e-10)
    Z = np.transpose(NCI_dB)

    x, y = rlib.axis(Z)

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, Z, cmap='jet')
    ax.view_init(-90, 0, 0)
    ax.set_proj_type('ortho')
    ax.set_xlabel('Range Bin')
    ax.set_ylabel('Doppler Bin')
    ax.set_zlabel('Energy [dB]')
    ax.set_title(title)
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"3D曲面图已保存: {save_path}")
    return fig


def plot_pointcloud_2d(peaks_with_azimuth, title="Radar Point Cloud (Range-Doppler)",
                       save_path=None):
    """
    2D 点云散点图: 距离 vs 多普勒, 颜色编码能量
    """
    if not peaks_with_azimuth:
        print("无峰值点，跳过2D点云绘制")
        return None

    ranges = [p[0] * RANGE_RES for p in peaks_with_azimuth]
    dopplers = [p[1] for p in peaks_with_azimuth]
    energies = [p[2] for p in peaks_with_azimuth]

    fig, ax = plt.subplots(figsize=(12, 8))
    scatter = ax.scatter(ranges, dopplers, c=energies, cmap='jet',
                         s=20, alpha=0.7, edgecolors='black', linewidth=0.3)
    plt.colorbar(scatter, ax=ax, label='Energy [dB]')

    ax.set_xlabel('Range [m]')
    ax.set_ylabel('Doppler Bin')
    ax.set_title(f'{title}\n({len(peaks_with_azimuth)} points)')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"2D点云已保存: {save_path}")
    return fig


def plot_pointcloud_3d(peaks_with_azimuth, title="Radar 3D Point Cloud",
                       save_path=None):
    """
    3D 点云散点图: X-Y-Z (笛卡尔坐标), 颜色编码能量

    将 (range, azimuth) 转换为 (x, y, z):
        x = range * cos(azimuth)
        y = range * sin(azimuth)
        z = 0 (无俯仰角信息时)
    """
    if not peaks_with_azimuth:
        print("无峰值点，跳过3D点云绘制")
        return None

    # peaks_with_azimuth: (range_bin, doppler_bin, energy_dB, azimuth_deg, azimuth_rad)
    xs, ys, zs, energies, azimuths = [], [], [], [], []
    for p in peaks_with_azimuth:
        r = p[0] * RANGE_RES
        az_rad = p[4]  # azimuth in radians
        xs.append(r * np.cos(az_rad))
        ys.append(r * np.sin(az_rad))
        zs.append(0.0)
        energies.append(p[2])
        azimuths.append(p[3])

    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)
    energies = np.array(energies)

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(xs, ys, zs, c=energies, cmap='jet',
                         s=15, alpha=0.8, marker='o')

    # 添加原点标记 (雷达位置)
    ax.scatter([0], [0], [0], c='red', marker='^', s=200, label='Radar')

    # 添加坐标轴参考线
    max_range = np.max(np.abs(xs)) if len(xs) > 0 else 10
    ax.plot([0, max_range], [0, 0], [0, 0], 'r--', alpha=0.3, linewidth=0.5)
    ax.plot([0, 0], [-max_range, max_range], [0, 0], 'g--', alpha=0.3, linewidth=0.5)
    ax.plot([0, 0], [0, 0], [-1, 1], 'b--', alpha=0.3, linewidth=0.5)

    plt.colorbar(scatter, ax=ax, label='Energy [dB]', shrink=0.6)

    ax.set_xlabel('X [m] (forward)')
    ax.set_ylabel('Y [m] (lateral)')
    ax.set_zlabel('Z [m] (height)')
    ax.set_title(f'{title}\n({len(peaks_with_azimuth)} points)')
    ax.legend()

    # 设置等比例坐标轴
    limit = max(max_range * 1.1, 2.0)
    ax.set_xlim(0, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit / 4, limit / 4)

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"3D点云已保存: {save_path}")
    return fig


def plot_combined(peaks_with_azimuth, NCI, title="Radar Point Cloud - Combined View",
                  save_path=None):
    """
    组合视图: 2×2 子图
    - 左上: NCI 热力图 + 峰值叠加
    - 右上: 2D 点云 (距离 vs 多普勒)
    - 左下: 3D 点云 (X-Y-Z 笛卡尔坐标)
    - 右下: 方位角分布直方图
    """
    NCI_dB = 10 * np.log10(NCI + 1e-10)
    Ndopp, Nrang = NCI.shape

    fig = plt.figure(figsize=(18, 14))

    # --- 左上: NCI 热力图 + 峰值 ---
    ax1 = fig.add_subplot(2, 2, 1)
    im = ax1.imshow(NCI_dB, aspect='auto', origin='lower', cmap='jet')
    if peaks_with_azimuth:
        peak_r = [p[0] for p in peaks_with_azimuth]
        peak_d = [p[1] for p in peaks_with_azimuth]
        ax1.scatter(peak_r, peak_d, c='white', marker='x', s=20,
                    linewidths=0.5)
    ax1.set_xlabel('Range Bin')
    ax1.set_ylabel('Doppler Bin')
    ax1.set_title(f'NCI + {len(peaks_with_azimuth)} Peaks')
    plt.colorbar(im, ax=ax1, label='dB')

    # --- 右上: 2D 点云 ---
    ax2 = fig.add_subplot(2, 2, 2)
    if peaks_with_azimuth:
        ranges = [p[0] * RANGE_RES for p in peaks_with_azimuth]
        dopplers = [p[1] for p in peaks_with_azimuth]
        energies = [p[2] for p in peaks_with_azimuth]
        sc = ax2.scatter(ranges, dopplers, c=energies, cmap='jet',
                         s=15, alpha=0.7, edgecolors='black', linewidth=0.2)
        plt.colorbar(sc, ax=ax2, label='Energy [dB]')
    ax2.set_xlabel('Range [m]')
    ax2.set_ylabel('Doppler Bin')
    ax2.set_title('2D Point Cloud (Range-Doppler)')
    ax2.grid(True, alpha=0.3)

    # --- 左下: 3D 点云 (笛卡尔坐标) ---
    ax3 = fig.add_subplot(2, 2, 3, projection='3d')
    if peaks_with_azimuth:
        xs, ys, zs, energies = [], [], [], []
        for p in peaks_with_azimuth:
            r = p[0] * RANGE_RES
            az_rad = p[4]
            xs.append(r * np.cos(az_rad))
            ys.append(r * np.sin(az_rad))
            zs.append(0.0)
            energies.append(p[2])

        sc3 = ax3.scatter(xs, ys, zs, c=energies, cmap='jet',
                          s=12, alpha=0.8)
        plt.colorbar(sc3, ax=ax3, label='Energy [dB]', shrink=0.6)

        max_r = max(np.max(np.abs(xs)), np.max(np.abs(ys)), 2.0)
        ax3.set_xlim(0, max_r * 1.1)
        ax3.set_ylim(-max_r * 1.1, max_r * 1.1)

    ax3.scatter([0], [0], [0], c='red', marker='^', s=150, label='Radar')
    ax3.set_xlabel('X [m]')
    ax3.set_ylabel('Y [m]')
    ax3.set_zlabel('Z [m]')
    ax3.set_title('3D Point Cloud (Cartesian)')
    ax3.legend()

    # --- 右下: 方位角分布 ---
    ax4 = fig.add_subplot(2, 2, 4)
    if peaks_with_azimuth:
        az_deg = [p[3] for p in peaks_with_azimuth]
        ax4.hist(az_deg, bins=36, color='steelblue', edgecolor='black',
                 alpha=0.7)
        ax4.axvline(x=0, color='red', linestyle='--', alpha=0.5)
        ax4.set_xlabel('Azimuth [deg]')
        ax4.set_ylabel('Count')
        ax4.set_title(f'Azimuth Distribution ({len(az_deg)} points)')
    else:
        ax4.text(0.5, 0.5, 'No points', ha='center', va='center',
                 transform=ax4.transAxes)
    ax4.grid(True, alpha=0.3)

    fig.suptitle(title, fontsize=14, fontweight='bold')
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"组合视图已保存: {save_path}")
    return fig


# =============================================================================
# 主处理流程
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="CTRX 雷达点云可视化 — 读取 ctrx0_raw.bin 并生成点云"
    )
    parser.add_argument('--file', type=str, default=DEFAULT_BIN_FILE,
                        help=f'二进制文件路径 (默认: {DEFAULT_BIN_FILE})')
    parser.add_argument('--frame', type=int, default=0,
                        help='起始帧编号 (默认: 0)')
    parser.add_argument('--nframes', type=int, default=8,
                        help='拼接帧数, 更多帧=更好多普勒分辨率 (默认: 8)')
    parser.add_argument('--nrx', type=int, default=V4L2_N_RX,
                        help=f'接收通道数 (默认: {V4L2_N_RX})')
    parser.add_argument('--nrows', type=int, default=V4L2_N_ROWS,
                        help=f'V4L2帧行数 (默认: {V4L2_N_ROWS})')
    parser.add_argument('--nsamples', type=int, default=V4L2_N_SAMPLES,
                        help=f'每行采样点数 (默认: {V4L2_N_SAMPLES})')
    parser.add_argument('--threshold-db', type=float, default=9,
                        help='CFAR 门限 [dB] (默认: 9)')
    parser.add_argument('--no-3d', action='store_true',
                        help='跳过3D曲面图')
    parser.add_argument('--no-combined', action='store_true',
                        help='跳过组合视图')
    parser.add_argument('--save', action='store_true',
                        help='保存图像到文件')
    parser.add_argument('--save-dir', type=str, default='./plots',
                        help='图像保存目录 (默认: ./plots)')
    parser.add_argument('--doa', action='store_true', default=True,
                        help='启用DOA方位角估计 (默认: 开启)')
    parser.add_argument('--no-doa', action='store_true',
                        help='禁用DOA方位角估计')
    args = parser.parse_args()

    if args.no_doa:
        args.doa = False

    # 检查文件是否存在
    if not os.path.exists(args.file):
        print(f"错误: 文件不存在: {args.file}")
        print("请确认 ctrx0_raw.bin 文件路径")
        return

    # 获取文件总帧数
    frame_bytes = args.nrows * args.nsamples * 2
    total_bytes = os.path.getsize(args.file)
    total_frames = total_bytes // frame_bytes
    print(f"文件: {args.file}")
    print(f"文件大小: {total_bytes / 1024**3:.2f} GB")
    print(f"每帧: {frame_bytes / 1024:.1f} KB ({args.nsamples}×{args.nrows} uint16)")
    print(f"总帧数: {total_frames}")
    print(f"起始帧: {args.frame}, 拼接帧数: {args.nframes}")
    print(f"输出: {args.nsamples} samples × {args.nrx} RX × {args.nframes * N_CHIRPS_PER_FRAME} chirps")

    if args.frame + args.nframes > total_frames:
        print(f"警告: 请求帧超出范围, 调整为 {total_frames - args.frame} 帧")
        args.nframes = max(1, total_frames - args.frame)

    # =========================================================================
    # 1. 读取数据
    # =========================================================================
    print("\n[1/5] 读取原始数据...")
    if args.nframes > 1:
        Raw = rlib.readRawBinV4L2Multi(
            args.file, args.frame, args.nframes,
            args.nsamples, args.nrows, args.nrx
        )
    else:
        Raw = rlib.readRawBinV4L2(
            args.file, args.frame,
            args.nsamples, args.nrows, args.nrx
        )

    nSamples, nRx, nRamps = Raw.shape
    print(f"  数据形状: (nSamples={nSamples}, nRx={nRx}, nRamps={nRamps})")

    # =========================================================================
    # 2. 距离-多普勒处理
    # =========================================================================
    print("[2/5] 距离-多普勒 FFT...")
    RD = rlib.rdFft(Raw)
    nDopp, nRang, nRx_out = RD.shape
    print(f"  RD cube: (nDopp={nDopp}, nRange={nRang}, nRx={nRx_out})")

    # =========================================================================
    # 3. NCI + 峰值检测
    # =========================================================================
    print("[3/5] 非相干积累 + 峰值检测...")
    NCI = rlib.nci(RD)
    LMAP = rlib.localMax(NCI)
    TMAP = rlib.thresholding(NCI, args.threshold_db)
    DMAP = LMAP & TMAP

    peaks_2d = rlib.getPeaks(DMAP)
    peaks_with_energy = rlib.getPeaksWithEnergy(DMAP, NCI)
    print(f"  检测到 {len(peaks_2d)} 个峰值 (局部最大 + CFAR门限 {args.threshold_db}dB)")

    # =========================================================================
    # 4. DOA 方位角估计
    # =========================================================================
    peaks_with_azimuth = []
    if args.doa and len(peaks_with_energy) > 0:
        print("[4/5] DOA 方位角估计...")
        peaks_with_azimuth = rlib.estimateAzimuth(
            RD, peaks_with_energy,
            wavelength=WAVELENGTH,
            d_rx=D_RX
        )
        az_min = min(p[3] for p in peaks_with_azimuth) if peaks_with_azimuth else 0
        az_max = max(p[3] for p in peaks_with_azimuth) if peaks_with_azimuth else 0
        print(f"  方位角范围: [{az_min:.1f}°, {az_max:.1f}°]")
        print(f"  有效DOA点数: {len(peaks_with_azimuth)}")
    else:
        print("[4/5] 跳过 DOA 估计")
        # 用原始峰值填充 (无方位角)
        peaks_with_azimuth = [(p[0], p[1], p[2], 0.0, 0.0)
                              for p in peaks_with_energy]

    # =========================================================================
    # 5. 可视化
    # =========================================================================
    print("[5/5] 生成可视化...")

    save_dir = args.save_dir if args.save else None
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    frame_label = f"frame_{args.frame}" if args.nframes == 1 else \
                  f"frames_{args.frame}_{args.frame + args.nframes - 1}"

    # 物理距离轴信息
    max_range_m = nRang * RANGE_RES
    print(f"\n  系统参数:")
    print(f"    距离分辨率: {RANGE_RES:.3f} m")
    print(f"    最大距离:   {max_range_m:.2f} m")
    print(f"    波长:       {WAVELENGTH*1000:.2f} mm")
    print(f"    RX间距:     {D_RX*1000:.2f} mm")

    # --- 图1: NCI 热力图 + 峰值 ---
    save_nci = os.path.join(save_dir, f"nci_heatmap_{frame_label}.png") if save_dir else None
    plot_nci_heatmap(NCI, peaks_2d,
                     title=f"NCI Range-Doppler Map ({frame_label})\n"
                           f"{nRamps} chirps, {len(peaks_2d)} peaks",
                     save_path=save_nci)

    # --- 图2: 2D 点云 ---
    save_pc2d = os.path.join(save_dir, f"pointcloud2d_{frame_label}.png") if save_dir else None
    plot_pointcloud_2d(peaks_with_azimuth,
                       title=f"2D Point Cloud ({frame_label})",
                       save_path=save_pc2d)

    # --- 图3: 3D 点云 (笛卡尔坐标) ---
    if args.doa and peaks_with_azimuth:
        save_pc3d = os.path.join(save_dir, f"pointcloud3d_{frame_label}.png") if save_dir else None
        plot_pointcloud_3d(peaks_with_azimuth,
                           title=f"3D Point Cloud - Cartesian ({frame_label})",
                           save_path=save_pc3d)

    # --- 图4: 组合视图 ---
    if not args.no_combined:
        save_combined = os.path.join(save_dir, f"combined_{frame_label}.png") if save_dir else None
        plot_combined(peaks_with_azimuth, NCI,
                      title=f"Radar Point Cloud Analysis - {frame_label}",
                      save_path=save_combined)

    # --- 图5: NCI 3D 曲面 (可选) ---
    if not args.no_3d:
        save_3d = os.path.join(save_dir, f"nci_3d_{frame_label}.png") if save_dir else None
        plot_nci_3d_surface(NCI,
                            title=f"NCI 3D Surface ({frame_label})",
                            save_path=save_3d)

    print("\n=== 可视化完成 ===")
    print(f"峰值总数: {len(peaks_2d)}")
    print(f"DOA有效点数: {len(peaks_with_azimuth)}")
    if args.save:
        print(f"图像保存目录: {save_dir}")

    plt.show()


if __name__ == "__main__":
    main()
