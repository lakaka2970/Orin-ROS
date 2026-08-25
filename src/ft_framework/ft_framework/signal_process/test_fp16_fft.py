"""fp16 FFT 可行性测试：cupy 半精度 FFT 是否可用 / 是否更快 / 精度是否够。

背景：torch.fft 不支持 fp16；cuFFT 原生支持半精度（CUFFT_R_16F / C_16F, CC>=7.0）。
      若 cupy 暴露半精度 FFT 且数值可用，则可在 15W 功耗墙（300MHz SM）内再压 ~2×，
      从而逼近 66ms 验收门槛。
"""
import time, statistics
import numpy as np
import torch
import cupy as cp


def bench(name, fn, n=30):
    for _ in range(3):
        fn()
    cp.cuda.Stream.null.synchronize()
    ts = []
    for _ in range(n):
        cp.cuda.Stream.null.synchronize(); t0 = time.perf_counter()
        fn()
        cp.cuda.Stream.null.synchronize(); ts.append((time.perf_counter() - t0) * 1000.0)
    print(f"[bench] {name:44s} min={min(ts):7.3f} med={statistics.median(ts):7.3f} ms")
    return statistics.median(ts)


def main():
    print(f"[cupy] {cp.__version__}  device={cp.cuda.runtime.getDeviceProperties(0)['name'].decode()}")
    # 与 pipeline 一致的尺寸
    N_CHIRPS, N_RX, N_SAMPLES = 512, 16, 2048
    rng = np.random.default_rng(0)
    x = rng.integers(-500, 500, size=(N_CHIRPS, N_RX, N_SAMPLES), dtype=np.int16)

    # ---- 1. fp16 FFT 是否可用 ----
    print("\n== fp16 FFT 可用性 ==")
    xf16 = cp.asarray(x.astype(np.float16))
    try:
        y16 = cp.fft.rfft(xf16, axis=2)
        print(f"  cupy.fft.rfft(float16) OK -> dtype={y16.dtype}, shape={y16.shape}")
        fp16_ok = True
    except Exception as e:
        print(f"  cupy.fft.rfft(float16) FAIL: {type(e).__name__}: {e}")
        fp16_ok = False

    # ---- 2. 精度对比：fp16 vs fp32 FFT ----
    if fp16_ok:
        print("\n== 精度对比 (fp16 vs fp32 rfft, 2048 点) ==")
        xf32 = cp.asarray(x.astype(np.float32))
        y32 = cp.fft.rfft(xf32, axis=2)
        y32_np = cp.asnumpy(y32)
        y16_np = cp.asnumpy(y16)
        # 相对误差
        mag32 = np.abs(y32_np)
        mag16 = np.abs(y16_np.astype(np.complex64))
        rel = np.abs(mag32 - mag16) / (mag32 + 1e-6)
        print(f"  幅度相对误差 mean={rel.mean():.2e}  p50={np.median(rel):.2e}  p99={np.percentile(rel,99):.2e}  max={rel.max():.2e}")

    # ---- 3. 速度对比：fp16 vs fp32 ----
    print("\n== 速度对比 ==")
    # 完整 rfft (512x16x2048 -> 512x16x1025)
    bench("cupy rfft fp32 (16x512x2048)", lambda: cp.fft.rfft(xf32, axis=2))
    if fp16_ok:
        bench("cupy rfft fp16 (16x512x2048)", lambda: cp.fft.rfft(xf16, axis=2))

    # doppler 维 fft (16x512x1025)
    c32 = cp.fft.rfft(xf32, axis=2).transpose(1, 0, 2)
    bench("cupy fft fp32 doppler dim=1", lambda: cp.fft.fft(c32, axis=1))
    if fp16_ok:
        c16 = cp.fft.rfft(xf16, axis=2).transpose(1, 0, 2)
        bench("cupy fft fp16 doppler dim=1", lambda: cp.fft.fft(c16, axis=1))

    # 对照：torch fp32
    xt = torch.as_tensor(x.astype(np.float32), device='cuda')
    torch.cuda.synchronize()
    bench("torch rfft fp32 (对照)", lambda: torch.fft.rfft(xt, dim=2))


if __name__ == "__main__":
    main()
