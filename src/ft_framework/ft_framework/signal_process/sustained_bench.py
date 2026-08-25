"""持续负载基准：连续处理 N 帧，测量稳定态单帧耗时与 GPU 时钟/功耗。

目的：区分"单帧冷启动（GPU 降频到 300MHz）"与"连续 15Hz 实时负载（GPU 升频）"的差异。
     真实 ROS 节点按 15Hz 连续处理，GPU 应保持升频；单帧孤立计时会低估性能。
"""
import os, sys, time, threading, subprocess, statistics
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import RadarConfig
from data_io import readRawBinCasc
from preprocessing_opt import radar_signal_process_final
from doppler_opt import doppler_processing_gpu
from peak_detection import peak_search_gpu
from doa_proc import doa_main_batch, doa_env

DEVICE = torch.device('cuda')


def build_antennas(cfg):
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
    Array_Azi = np.zeros((3, len(AzmPosUse)), dtype=np.float32)
    Array_Azi[0, :] = AzmPosUse
    Array_Ele = np.zeros((3, len(ElvPosUse)), dtype=np.float32)
    Array_Ele[1, :] = ElvPosUse
    AziIdx = torch.from_numpy(AzmChUse).to(DEVICE)
    EleIdx = torch.from_numpy(ElvChUse).to(DEVICE)
    Array_Azi_gpu = torch.from_numpy(Array_Azi).to(DEVICE)
    Array_Ele_gpu = torch.from_numpy(Array_Ele).to(DEVICE)
    return Array_Azi, Array_Ele, AziIdx, EleIdx, Array_Azi_gpu, Array_Ele_gpu


def gpu_only(cfg, raw_gpu):
    """仅 GPU 部分：preprocess+doppler，返回耗时。"""
    torch.cuda.synchronize(); t0 = time.perf_counter()
    cube, _, _ = radar_signal_process_final(raw_gpu, cfg.n_samples, cfg.n_rx, cfg.n_chirps, cfg.threshold_scale)
    doppler_processing_gpu(cube, cfg.n_rx, cfg.n_chirps, cube.shape[2],
                           cfg.tx_ddma_idx, cfg.n_subbands, cfg.noise_est_ratio)
    torch.cuda.synchronize()
    return (time.perf_counter() - t0) * 1000.0


def full_pipeline(cfg, raw_gpu, Array_Azi_gpu, AziIdx, Array_Ele_gpu, EleIdx):
    torch.cuda.synchronize(); t0 = time.perf_counter()
    cube, _, _ = radar_signal_process_final(raw_gpu, cfg.n_samples, cfg.n_rx, cfg.n_chirps, cfg.threshold_scale)
    rd, rx, noise, vch, ms, mv = doppler_processing_gpu(
        cube, cfg.n_rx, cfg.n_chirps, cube.shape[2], cfg.tx_ddma_idx, cfg.n_subbands, cfg.noise_est_ratio)
    peaks = peak_search_gpu(rd, mv, ms, rx, noise, cfg.tx_ddma_idx, cube.shape[2],
                            cfg.n_chirps, cfg.n_subbands, ps_scale=cfg.ps_scale,
                            max_peaks_per_rb=cfg.max_peaks_per_rb, max_total_peaks=cfg.max_total_peaks)
    if peaks:
        cb = torch.stack([p['channel_gpu'] for p in peaks])
        doa_main_batch(cb, 28.0)
    torch.cuda.synchronize()
    return (time.perf_counter() - t0) * 1000.0


def sample_clocks_now():
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=clocks.sm,clocks.mem,power.draw,power.limit,utilization.gpu,utilization.memory",
             "--format=csv,noheader,nounits"], text=True, timeout=2).strip()
        p = [x.strip() for x in out.split(",")]
        return int(p[0]), int(p[1]), float(p[2]), p[3], int(p[4]), int(p[5])
    except Exception:
        return -1, -1, -1, "?", -1, -1


def run_sustained(label, N, fn):
    """运行 N 帧并在后台采样时钟/功耗/利用率，返回 (times, stats)。"""
    sms, mems, pows, limits, gpus, memus = [], [], [], [], [], []
    stop = threading.Event()

    def sampler():
        while not stop.is_set():
            sm, mem, pw, lim, gu, mu = sample_clocks_now()
            if sm > 0:
                sms.append(sm); mems.append(mem); pows.append(pw)
                limits.append(lim); gpus.append(gu); memus.append(mu)
            time.sleep(0.1)

    th = threading.Thread(target=sampler, daemon=True)
    th.start()
    times = []
    for _ in range(N):
        times.append(fn())
    stop.set()
    th.join(timeout=1.0)

    f = lambda v: (round(min(v), 1), round(statistics.mean(v), 1), round(max(v), 1))
    sm_stat = f(sms) if sms else (-1, -1, -1)
    mem_stat = f(mems) if mems else (-1, -1, -1)
    pw_stat = f(pows) if pows else (-1, -1, -1)
    gu_stat = f(gpus) if gpus else (-1, -1, -1)
    mu_stat = f(memus) if memus else (-1, -1, -1)
    lim_set = sorted(set(l for l in limits if l not in ("?", "N/A", "[N/A]")))
    print(f"\n[{label} 持续 {N} 帧] 单帧 min/median/mean = "
          f"{min(times):.2f}/{statistics.median(times):.2f}/{sum(times)/N:.2f} ms")
    print(f"  期间 GPU 时钟 SM(min/mean/max)={sm_stat} MHz  "
          f"MEM={mem_stat} MHz")
    print(f"  期间 POW(min/mean/max)={pw_stat} W  "
          f"利用率 GPU={gu_stat}%  显存占用率={mu_stat}%  power.limit={lim_set or 'N/A'}")
    return times


def main():
    cfg = RadarConfig()
    print(f"[GPU] {torch.cuda.get_device_name(0)}")
    ant = build_antennas(cfg)
    Array_Azi, Array_Ele, AziIdx, EleIdx, Array_Azi_gpu, Array_Ele_gpu = ant
    if not doa_env.is_initialized:
        doa_env.prepare_mapping_indices(Array_Azi, Array_Ele)
        doa_env.cache_selection_indices(AziIdx, EleIdx)

    raw = readRawBinCasc(".", 0, cfg.n_samples, cfg.n_chirps, cfg.n_rx).astype(np.int16)
    raw_gpu = torch.from_numpy(raw).to(DEVICE)

    # 预热
    for _ in range(20):
        gpu_only(cfg, raw_gpu)
    torch.cuda.synchronize()

    run_sustained("仅 GPU (preprocess+doppler)", 200, lambda: gpu_only(cfg, raw_gpu))

    for _ in range(10):
        full_pipeline(cfg, raw_gpu, Array_Azi_gpu, AziIdx, Array_Ele_gpu, EleIdx)
    torch.cuda.synchronize()
    run_sustained("全链路 (含 peak+doa)", 50,
                  lambda: full_pipeline(cfg, raw_gpu, Array_Azi_gpu, AziIdx, Array_Ele_gpu, EleIdx))


if __name__ == "__main__":
    main()
