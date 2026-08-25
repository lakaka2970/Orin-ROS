# -*- coding: utf-8 -*-
import json, os, statistics

files = ["bench_local_cpu.json", "bench_remote_cpu.json", "bench_remote_gpu.json"]

for fn in files:
    if not os.path.exists(fn):
        print(f"### {fn}: MISSING"); continue
    d = json.load(open(fn, encoding="utf-8"))
    print(f"### {d['tag']}  (script={d['script']}, gpu={d['gpu_enabled']})")
    for i, r in enumerate(d["runs"], 1):
        tj = r.get("target_json") or {}
        tm = tj.get("timings_ms") or {}
        # gpu 脚本的 timings_ms 是 {stage: {min,median,mean}}；cpu 脚本是 {stage: 数值}
        def flat(v):
            return v if isinstance(v, (int, float)) else (v.get("median") if isinstance(v, dict) else None)
        keys = ["preprocess", "doppler", "peak", "doa", "doa_batch", "total"]
        parts = [f"{k}={flat(tm.get(k))}" for k in keys if tm.get(k) is not None]
        res = (f"cpu_max={r['cpu%']['max']}%" if r.get('cpu%') else "")
        if r.get("gpu_util%"):
            res += f" gpu_util={r['gpu_util%']['max']}% mem={r['gpu_mem_mb']['max']}MB temp={r['gpu_temp_c']['max']}C pow={r['gpu_power_w']['max']}W"
        print(f"  rep{i}: wall={r['wall_s']}s rc={r['returncode']} | {' '.join(parts)} | {res}")
    # 汇总 total 的 min/median/mean
    totals = []
    for r in d["runs"]:
        tj = r.get("target_json") or {}
        tm = tj.get("timings_ms") or {}
        v = tm.get("total")
        if isinstance(v, dict):
            v = v.get("median")
        if isinstance(v, (int, float)):
            totals.append(v)
    if totals:
        print(f"  => total across reps: min={min(totals):.2f} median={statistics.median(totals):.2f} mean={sum(totals)/len(totals):.2f}  best_rep={totals.index(min(totals))+1}")
    print()
