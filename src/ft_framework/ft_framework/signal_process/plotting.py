# plotting.py
import torch
import numpy as np
import matplotlib.pyplot as plt

def _to_numpy(x):
    if torch.is_tensor(x):
        return x.detach().cpu().numpy()
    return x

def plot_range_profile(radarcube, save_path=None):
    radarcube = _to_numpy(radarcube)
    range_profile = np.mean(np.abs(radarcube), axis=(0, 1))
    range_db = 20 * np.log10(range_profile + 1e-10)
    range_db -= np.max(range_db)
    plt.figure(figsize=(10, 4))
    plt.plot(range_db, linewidth=1.2)
    plt.title("Average Range Profile")
    plt.xlabel("Range Bin")
    plt.ylabel("Normalized Power (dB)")
    plt.ylim([-80, 0])
    plt.grid(True, linestyle='--', alpha=0.6)
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()

def plot_range_chirp_energy(radarcube, vmin=-80, vmax=0, save_path=None):
    radarcube = _to_numpy(radarcube)
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
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()

def plot_rd_cube(rd_cube, rx_idx=0, vmin=-80, vmax=0, save_path=None):
    rd_cube = _to_numpy(rd_cube)   # 注意 rd_cube 是复数
    rd_energy = np.abs(rd_cube[rx_idx])
    rd_db = 20 * np.log10(rd_energy + 1e-10)
    rd_db -= np.max(rd_db)
    plt.figure(figsize=(12, 6))
    plt.imshow(rd_db, aspect='auto', cmap='jet', vmin=vmin, vmax=vmax)
    plt.colorbar(label='Normalized Power (dB)')
    plt.title(f"Range-Doppler Map (RX {rx_idx})")
    plt.xlabel("Range Bin")
    plt.ylabel("Doppler Bin")
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()

def plot_rx_nci(rx_nci, save_path=None):
    rx_nci = _to_numpy(rx_nci)
    rx_nci_db = 10 * np.log10(rx_nci + 1e-10)
    rx_nci_db -= np.max(rx_nci_db)
    plt.figure(figsize=(12, 6))
    plt.imshow(rx_nci_db.T, aspect='auto', cmap='jet', vmin=-60, vmax=0)
    plt.colorbar(label='Normalized Power (dB)')
    plt.title("RX Non-Coherent Integration")
    plt.xlabel("Doppler Bin")
    plt.ylabel("Range Bin")
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()

def plot_noise_estimation(noise_est, save_path=None):
    noise_est = _to_numpy(noise_est)
    plt.figure(figsize=(12, 4))
    plt.plot(noise_est, linewidth=1.5)
    plt.title("Noise Floor Estimation per Doppler Bin")
    plt.xlabel("Doppler Bin")
    plt.ylabel("Noise Power (linear)")
    plt.grid(True, linestyle='--', alpha=0.6)
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()

def plot_vch_nci(vch_nci, vmin=-60, vmax=0, save_path=None):
    vch_nci = _to_numpy(vch_nci)
    vch_db = 10 * np.log10(vch_nci + 1e-10)
    vch_db -= np.max(vch_db)
    plt.figure(figsize=(12, 6))
    plt.imshow(vch_db, aspect='auto', cmap='jet', vmin=vmin, vmax=vmax)
    plt.colorbar(label='Normalized Power (dB)')
    plt.title("VCH NCI")
    plt.xlabel("Range Bin")
    plt.ylabel("Doppler Bin")
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()

def plot_max_subband(max_vch_nci, max_subband_idx, save_path=None):
    max_vch_nci = _to_numpy(max_vch_nci)
    max_subband_idx = _to_numpy(max_subband_idx)
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
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()

def plot_peaks_on_vch(vch_nci, peaks, save_path=None):
    vch_nci = _to_numpy(vch_nci)
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
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()
def plot_all_intermediate(radarcube, rd_cube, rx_nci, noise_est, vch_nci,
                          max_vch_nci, max_subband_idx, peaks=None, save_dir=None):
    """
    一键绘制所有中间结果（用于调试或报告）
    """
    set_plot_style()
    # 1. 平均距离谱
    plot_range_profile(radarcube, save_path=f"{save_dir}/range_profile.png" if save_dir else None)
    # 2. Range-Chirp 热力图
    plot_range_chirp_energy(radarcube, save_path=f"{save_dir}/range_chirp.png" if save_dir else None)
    # 3. 距离-多普勒图（取第一个接收通道）
    plot_rd_cube(rd_cube, rx_idx=0, save_path=f"{save_dir}/rd_cube.png" if save_dir else None)
    # 4. RX NCI
    plot_rx_nci(rx_nci, save_path=f"{save_dir}/rx_nci.png" if save_dir else None)
    # 5. 噪声估计曲线
    plot_noise_estimation(noise_est, save_path=f"{save_dir}/noise_est.png" if save_dir else None)
    # 6. VCH NCI
    plot_vch_nci(vch_nci, save_path=f"{save_dir}/vch_nci.png" if save_dir else None)
    # 7. 子带最大值图
    plot_max_subband(max_vch_nci, max_subband_idx, save_path=f"{save_dir}/max_subband.png" if save_dir else None)
    # 8. 如果提供了峰值，在 VCH 图上标注
    if peaks is not None:
        plot_peaks_on_vch(vch_nci, peaks, save_path=f"{save_dir}/peaks.png" if save_dir else None)