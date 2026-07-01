# plotting_cpu.py — 纯NumPy版绘图，无torch依赖
import os
import numpy as np
import matplotlib.pyplot as plt


def _ensure_numpy(x):
    """确保输入为numpy数组（已是numpy则原样返回）。"""
    return np.asarray(x)


def plot_range_profile(radarcube, rx_idx=0, vmin=-80, vmax=0, save_path=None):
    """绘制单个通道的 Range-Chirp 能量热力图。

    Args:
        radarcube: 形状 (n_rx, n_chirps, range_bins)，距离FFT复数结果
        rx_idx: 要绘制的接收通道索引
    """
    radarcube = _ensure_numpy(radarcube)
    ch_data = np.abs(radarcube[rx_idx])                    # (n_chirps, range_bins)
    ch_db = 20 * np.log10(ch_data + 1e-10)
    ch_db -= np.max(ch_db)
    plt.figure(figsize=(12, 6))
    # x=range_bin, y=chirp, 颜色=能量
    plt.imshow(ch_db, aspect='auto', cmap='jet', vmin=vmin, vmax=vmax)
    plt.colorbar(label='Normalized Power (dB)')
    plt.title(f"Range-Chirp Heatmap (RX {rx_idx})")
    plt.xlabel("Range Bin")
    plt.ylabel("Chirp Index")
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()


def plot_single_chirp_range(radarcube, rx_idx=0, chirp_idx=0, save_path=None):
    """绘制单个通道、单个chirp的一维距离向幅度曲线。

    Args:
        radarcube: 形状 (n_rx, n_chirps, range_bins)
        rx_idx: 通道索引
        chirp_idx: chirp索引
    """
    radarcube = _ensure_numpy(radarcube)
    profile = np.abs(radarcube[rx_idx, chirp_idx, :])    # (range_bins,)
    profile_db = 20 * np.log10(profile + 1e-10)
    plt.figure(figsize=(10, 4))
    plt.plot(profile_db, linewidth=1.0)
    plt.title(f"Range Profile — RX {rx_idx}, Chirp {chirp_idx}")
    plt.xlabel("Range Bin")
    plt.ylabel("Normalized Power (dB)")
    plt.ylim([0, 150])
    plt.grid(True, linestyle='--', alpha=0.6)
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()


def plot_range_chirp_energy(radarcube, vmin=-80, vmax=0, save_path=None):
    radarcube = _ensure_numpy(radarcube)
    energy = np.mean(np.abs(radarcube), axis=0)
    energy_db = 20 * np.log10(energy + 1e-10)
    energy_db -= np.max(energy_db)
    plt.figure(figsize=(12, 6))
    plt.imshow(energy_db.T, aspect='auto', cmap='jet', vmin=vmin, vmax=vmax)
    plt.colorbar(label='Normalized Power (dB)')
    plt.title("Range-Chirp Energy Heatmap")
    plt.xlabel("Chirp Index")
    plt.ylabel("Range Bin")
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()


def plot_rd_cube(rd_cube, rx_idx=0, slice_rb=16, vmin=-80, vmax=0, save_path=None):
    """绘制 Range-Doppler Map + 固定 range_bin 的 Doppler 切片。

    Args:
        rd_cube: 形状 (n_rx, n_doppler, n_range_bins)，复数
        rx_idx: 接收通道索引
        slice_rb: 切片的 range bin 索引
        vmin/vmax: 热力图 dB 范围，None 则自适应
    """
    rd_cube = _ensure_numpy(rd_cube)
    rd_energy = np.abs(rd_cube[rx_idx])                         # (n_doppler, n_range_bins)
    rd_db = 20 * np.log10(rd_energy + 1e-10)
    rd_db -= np.max(rd_db)
    # 自适应 vmin: 取 5 分位值，上限 -80 dB
    if vmin is None:
        vmin = max(-80, float(np.percentile(rd_db, 5)))
    if vmax is None:
        vmax = 0

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 6))

    # 左: Range-Doppler 热力图
    im = ax1.imshow(rd_db, aspect='auto', cmap='jet', vmin=vmin, vmax=vmax)
    ax1.set_title(f"Range-Doppler Map (RX {rx_idx})")
    ax1.set_xlabel("Range Bin")
    ax1.set_ylabel("Doppler Bin")
    ax1.axvline(x=slice_rb, color='lime', linestyle='--', linewidth=1.5, label=f'RB={slice_rb}')
    ax1.legend()
    plt.colorbar(im, ax=ax1, label='Normalized Power (dB)')

    # 右: 固定 range_bin 的 512 Doppler 曲线 + 标记前16个最强 local peak
    doppler_slice = rd_db[:, slice_rb]                           # (n_doppler,)
    ax2.plot(doppler_slice, linewidth=0.8)
    # 找 local peaks（比左右两侧能量高），再取前16个最强的
    is_peak = np.ones(len(doppler_slice), dtype=bool)
    is_peak[0], is_peak[-1] = False, False
    is_peak[1:-1] = (doppler_slice[1:-1] > doppler_slice[:-2]) & \
                    (doppler_slice[1:-1] > doppler_slice[2:])
    local_peak_indices = np.where(is_peak)[0]
    local_peak_values = doppler_slice[local_peak_indices]
    top_k = min(16, len(local_peak_indices))
    if top_k > 0:
        take = np.argpartition(local_peak_values, -top_k)[-top_k:]
        take = take[np.argsort(local_peak_values[take])[::-1]]
        top_indices = local_peak_indices[take]
        top_values = local_peak_values[take]
        ax2.scatter(top_indices, top_values, c='red', s=40, marker='x', zorder=5)
        for idx, val in zip(top_indices, top_values):
            ax2.annotate(f'{idx}', (idx, val), textcoords="offset points",
                         xytext=(0, 8), fontsize=7, ha='center', color='red')
    ax2.set_title(f"Doppler Slice at Range Bin {slice_rb}  (top {top_k} local peaks)")
    ax2.set_xlabel("Doppler Bin")
    ax2.set_ylabel("Normalized Power (dB)")
    ax2.set_ylim([vmin, vmax + 10])
    ax2.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()


def plot_rx_nci(rx_nci, save_path=None):
    rx_nci = _ensure_numpy(rx_nci)
    rx_nci_db = 10 * np.log10(rx_nci + 1e-10)
    rx_nci_db -= np.max(rx_nci_db)
    plt.figure(figsize=(12, 6))
    plt.imshow(rx_nci_db.T, aspect='auto', cmap='jet', vmin=-60, vmax=0)
    plt.colorbar(label='Normalized Power (dB)')
    plt.title("RX Non-Coherent Integration")
    plt.xlabel("Doppler Bin")
    plt.ylabel("Range Bin")
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()


def plot_noise_estimation(noise_est, save_path=None):
    noise_est = _ensure_numpy(noise_est)
    plt.figure(figsize=(12, 4))
    plt.plot(noise_est, linewidth=1.5)
    plt.title("Noise Floor Estimation per Doppler Bin")
    plt.xlabel("Doppler Bin")
    plt.ylabel("Noise Power (linear)")
    plt.grid(True, linestyle='--', alpha=0.6)
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()


def plot_vch_nci(vch_nci, vmin=-60, vmax=0, save_path=None):
    vch_nci = _ensure_numpy(vch_nci)
    vch_db = 10 * np.log10(vch_nci + 1e-10)
    vch_db -= np.max(vch_db)
    plt.figure(figsize=(12, 6))
    plt.imshow(vch_db, aspect='auto', cmap='jet', vmin=vmin, vmax=vmax)
    plt.colorbar(label='Normalized Power (dB)')
    plt.title("VCH NCI")
    plt.xlabel("Range Bin")
    plt.ylabel("Doppler Bin")
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()


def plot_max_subband(max_vch_nci, max_subband_idx, save_path=None):
    max_vch_nci = _ensure_numpy(max_vch_nci)
    max_subband_idx = _ensure_numpy(max_subband_idx)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    im1 = ax1.imshow(max_vch_nci, aspect='auto', cmap='hot')
    ax1.set_title("Max VCH NCI per Subband")
    ax1.set_xlabel("Subband Index")
    ax1.set_ylabel("Range Bin")
    plt.colorbar(im1, ax=ax1, label='Power')
    im2 = ax2.imshow(max_subband_idx, aspect='auto', cmap='viridis')
    ax2.set_title("Max Subband Doppler Index")
    ax2.set_xlabel("Subband Index")
    ax2.set_ylabel("Range Bin")
    plt.colorbar(im2, ax=ax2, label='Doppler Bin')
    plt.tight_layout()
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()


def plot_peaks_on_vch(vch_nci, peaks, save_path=None):
    vch_nci = _ensure_numpy(vch_nci)
    vch_db = 10 * np.log10(vch_nci + 1e-10)
    vch_db -= np.max(vch_db)
    plt.figure(figsize=(12, 6))
    plt.imshow(vch_db, aspect='auto', cmap='jet', vmin=-60, vmax=0)
    plt.colorbar(label='Normalized Power (dB)')
    if peaks:
        rbs = [p['rb'] for p in peaks]
        dbs = [p['db'] for p in peaks]
        plt.scatter(rbs, dbs, c='red', s=20, marker='x', label=f'Detected Peaks ({len(peaks)})')
    plt.title("VCH NCI with Detected Peaks")
    plt.xlabel("Range Bin")
    plt.ylabel("Doppler Bin")
    plt.legend()
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()


def plot_adc_chirp(raw_data, chirp_idx=0, rx_idx=0, save_path=None):
    """绘制某一个chirp的ADC原始时域数据。

    Args:
        raw_data: 形状 (n_chirps, n_rx, n_samples)，实部数据
        chirp_idx: 要绘制的chirp索引
        rx_idx: 要绘制的接收通道索引
    """
    raw_data = _ensure_numpy(raw_data)
    adc_samples = raw_data[chirp_idx, rx_idx, :]  # (n_samples,)
    plt.figure(figsize=(14, 5))
    plt.subplot(2, 1, 1)
    plt.plot(adc_samples, linewidth=0.6)
    plt.title(f"ADC Raw Data — Chirp {chirp_idx}, RX {rx_idx}")
    plt.xlabel("Sample Index")
    plt.ylabel("ADC Code")
    plt.grid(True, linestyle='--', alpha=0.5)

    plt.subplot(2, 1, 2)
    adc_db = 20 * np.log10(np.abs(adc_samples) + 1e-10)
    adc_db -= np.max(adc_db)
    plt.plot(adc_db, linewidth=0.6)
    plt.title("ADC Magnitude (dB, normalized)")
    plt.xlabel("Sample Index")
    plt.ylabel("Normalized Power (dB)")
    plt.ylim([-80, 5])
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()
