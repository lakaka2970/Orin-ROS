# -*- coding: utf-8 -*-
"""重复运行计时脚本并采集系统资源占用，汇总为 JSON。

用法:
  python bench_reps.py <script> <n_reps> <tag> [--no-gpu]
    script : 要重复运行的计时脚本（run_timing.py / run_timing_gpu.py）
    n_reps : 重复次数
    tag    : 输出标签（local_cpu / remote_cpu / remote_gpu ...）
    --no-gpu : 不采集 GPU（无 NVIDIA GPU 的机器）

输出:
  bench_<tag>.json           —— 汇总（每次运行的资源 min/max/mean + wall + 目标脚本的结构化结果）
  bench_<tag>_run<i>.log     —— 每次运行的完整 stdout/stderr
"""
import os, sys, time, json, subprocess, threading, shutil, re

def parse_timings(stdout):
    """解析 run_timing.py 的 [Timing] ... ms 行。"""
    d = {}
    for m in re.finditer(r"\[Timing\]\s*([^:：]+)[:：]\s*([\d.]+)\s*ms", stdout):
        d[m.group(1).strip()] = float(m.group(2))
    return d

def main():
    script = sys.argv[1]
    n_reps = int(sys.argv[2])
    tag = sys.argv[3]
    no_gpu = "--no-gpu" in sys.argv

    import psutil
    gpu_enabled = (not no_gpu) and (shutil.which("nvidia-smi") is not None)

    def run_once(idx):
        samples = []
        stop = threading.Event()

        def sampler():
            psutil.cpu_percent(interval=None)  # prime 首采样
            while not stop.is_set():
                t = time.time()
                row = {"t": round(t, 1),
                       "cpu%": psutil.cpu_percent(interval=None),
                       "ram_used_mb": round(psutil.virtual_memory().used / 1e6, 1),
                       "ram%": psutil.virtual_memory().percent}
                if gpu_enabled:
                    try:
                        out = subprocess.check_output(
                            ["nvidia-smi",
                             "--query-gpu=utilization.gpu,memory.used,temperature.gpu,power.draw",
                             "--format=csv,noheader,nounits"],
                            text=True, timeout=3).strip()
                        p = [x.strip() for x in out.split(",")]
                        row["gpu_util%"] = int(p[0])
                        row["gpu_mem_mb"] = int(p[1])
                        row["gpu_temp_c"] = int(p[2])
                        row["gpu_power_w"] = float(p[3])
                    except Exception:
                        pass
                samples.append(row)
                time.sleep(0.4)

        th = threading.Thread(target=sampler, daemon=True)
        th.start()
        t0 = time.time()
        proc = subprocess.run([sys.executable, script], cwd=os.getcwd(),
                              capture_output=True, text=True, errors='replace')
        wall = time.time() - t0
        stop.set()
        th.join(timeout=1.5)

        with open(f"bench_{tag}_run{idx}.log", "w", encoding="utf-8") as f:
            f.write(proc.stdout)
            f.write("\n--- STDERR ---\n")
            f.write(proc.stderr)

        # 目标脚本的结构化结果
        timings = parse_timings(proc.stdout)
        target_json = None
        json_name = "gpu_timing_result.json" if "gpu" in script else "cpu_timing_result.json"
        if os.path.exists(json_name):
            try:
                with open(json_name, "r", encoding="utf-8") as f:
                    target_json = json.load(f)
            except Exception:
                pass

        def stat(key):
            vals = [s[key] for s in samples if key in s]
            if not vals:
                return None
            return {"min": round(min(vals), 1), "max": round(max(vals), 1),
                    "mean": round(sum(vals) / len(vals), 1)}

        res = {"wall_s": round(wall, 2), "n_samples": len(samples),
               "returncode": proc.returncode,
               "cpu%": stat("cpu%"), "ram_used_mb": stat("ram_used_mb"),
               "ram%": stat("ram%"), "timings_ms": timings, "target_json": target_json}
        if gpu_enabled:
            res["gpu_util%"] = stat("gpu_util%")
            res["gpu_mem_mb"] = stat("gpu_mem_mb")
            res["gpu_temp_c"] = stat("gpu_temp_c")
            res["gpu_power_w"] = stat("gpu_power_w")
        return res

    runs = []
    for i in range(n_reps):
        print(f"===== rep {i+1}/{n_reps} =====", flush=True)
        r = run_once(i + 1)
        runs.append(r)
        line = f"  wall={r['wall_s']}s  cpu max={r['cpu%']['max'] if r['cpu%'] else '?'}%"
        if gpu_enabled and r.get("gpu_util%"):
            line += (f"  gpu_util max={r['gpu_util%']['max']}%"
                     f" gpu_mem max={r['gpu_mem_mb']['max']}MB"
                     f" temp max={r['gpu_temp_c']['max']}C"
                     f" power max={r['gpu_power_w']['max']}W")
        print(line, flush=True)

    out = {"tag": tag, "script": script, "n_reps": n_reps,
           "gpu_enabled": gpu_enabled, "runs": runs}
    with open(f"bench_{tag}.json", "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"[out] 已写入 bench_{tag}.json", flush=True)

if __name__ == "__main__":
    main()
