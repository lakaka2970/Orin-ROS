# GPU 单帧耗时优化——策略变更记录（调试经验参考）

> 目标：把验证平台（GTX 1660 Ti）上 GPU 单帧总耗时从基线压进 ≤66 ms（15 Hz 帧时限）。
> 结论速览：代码层优化把 139.7 ms → 85.3 ms（−39%，正确性不变，4/4 目标检出）；剩下的 85.3 → 66 ms 缺口**不是代码问题，而是验证设备 GPU 被固件锁死在 15 W 功耗墙、SM 时钟恒为 300 MHz（满血 2100 MHz）**。fp16 / CUDA Graph / TensorRT 类手段在该功耗墙下均无收益或负收益，详见第 7 节。
> 每一条都记录：**动机 → 做了什么 → 实测结果 → 为什么有效/无效**。

---

## 0. 基线

- 原始代码：`preprocessing.py` + `doppler.py` + `peak_detection.py` + `doa_proc.py`（PyTorch CUDA fp32，非自定义 kernel）。
- 输入：32 MB 合成帧（512 chirp × 16 RX × 2048 采样，int16）。
- 基线单帧（5 次取中位数，`run_timing_gpu.py`）：**预处理 72.9 + 多普勒 50.8 + 峰值 6.7 + DOA 8.7 ≈ 139.7 ms**。
- 检测正确性基线：25 峰，4 个真值目标 (80,100)(139,130)(139,210)(250,200) 全部精确检出。

---

## 1. 先做细粒度 profile，再动手（关键）

**动机**：基线 140 ms 分布在 4 个阶段，但不知道每阶段内部是「FFT 计算」还是「kernel 启动」还是「无效代码」占主导。盲改会浪费时间。

**做法**：写 `profile_hotspots.py`，用 `bench(name, fn)` 对每个微操作（加窗、rfft、doppler fft、abs、roll、quantile、gather、topk、子带循环……）单独计时取中位数。

**结果**：暴露出两个「免费午餐」：
1. **一段死代码**（时域干扰抑制块）：计算了 `diff / diff_ave / threshold / mask`，但 `torch.where(mask, ...)` 已被注释，`mask` 结果从未被使用 → 纯浪费，~35 ms。
2. **子带峰值 32 次 for 循环**：每次 `arange + gather + max + scatter`，~13 ms。

**教训**：GPU 优化前先 profile，别凭直觉。最大的一笔（35 ms）是「死代码」，任何算法优化都替代不了把它删掉。

---

## 2. 策略 1：删除死代码（时域干扰抑制）

**动机**：profile 显示 35 ms 计算完全未被使用。

**做法**：`preprocessing_opt.py` 复制 `preprocessing.py`，删除 diff/diff_ave/threshold/mask 那一段及未用的 `ZERO_INT16` 常量，其余逐字节一致。

**结果**：预处理 72.9 → ~38 ms（−35 ms），**数值严格等价**（删的是没被引用的量）。

---

## 3. 策略 2：子带峰值循环向量化

**动机**：32 次循环每次 3 个小 kernel 启动，13 ms。

**做法**（`doppler_opt.py`）：利用 stride 语义 reshape+permute+max 一次完成。
```
原:  for sub_idx in range(32): vch_nci[:, sub_idx::32].max()
优化: vch_view = vch_nci.view(R, 16, 32).permute(0, 2, 1)   # [r,s,w]=vch_nci[r, w*32+s]
      max_vch_nci, max_local = vch_view.max(dim=2)          # max over w == 原循环
```
**关键坑（踩过）**：全局多普勒索引必须 `= max_local * n_subbands + subband_idx`，其中 stride 是 `n_subbands`（32），**不是** `doppler_step`（16，DDMA 频移步长）。第一版误用 `doppler_step`，检测到 (80,100)→(80,52) 这类多普勒门减半的错误，改正后 25 峰与原版逐一吻合。

**结果**：13.1 → 0.24 ms（~55×），数值严格等价。

---

## 4. 策略 3：RX 非相干积累「先 sum 后 roll」

**动机**：原版对后 8 通道 abs 得到 (8,512,1025) 大张量，再做两次 `torch.roll` 再 sum。roll 与 sum(dim=0) 可交换。

**做法**：先 `sum(rd_cube[n_rx_half:].abs(), dim=0)` 得到 (512,1025)，再对这个小 8 倍的结果做两次 roll。

**结果**：−~7 ms，数值严格等价（roll 是可交换的线性算子）。

---

## 5. 策略 4：CUDA Graph（减 kernel 启动开销）→ **无效**

**动机**：假设瓶颈是「大量小 kernel 的启动开销」，CUDA Graph 把整条 kernel 序列固化成一张图，重放时一次提交。

**做法**：`run_timing_gpu_opt.py --graph`，`torch.cuda.CUDAGraph()` 捕获 preprocess+doppler。**坑**：`torch.tensor(numpy)` 含 host→device 拷贝，捕获期间被禁止 → 把 DDMA 索引张量 `_ddma_cache` 提到 capture 之前预创建。

**结果**：**~1 ms 收益，可忽略**。说明瓶颈不是 kernel 启动，而是 kernel 本身在 300 MHz 下跑不动（见第 7 节）。

---

## 6. 策略 5 / 6：fp16 混合精度 & 连续 FFT 布局 → **均无效**

**动机**：fp16 理论上 half2 SIMD 翻倍；跨步 FFT 理论上比连续布局慢。

**结果（`test_opt_variants.py`、`test_fp16_fft.py` 实测）**：
- **autocast fp16**：~0 收益。原因：`torch.fft` 被排除在 autocast 之外，FFT 仍走 fp32；元素级 op 虽转 fp16，但不是瓶颈。
- **连续 FFT 布局**：跨步 14.93 ms vs 连续 14.95 ms，**无差异**。
- **cupy fp16 FFT**：`cupy.fft.rfft(float16)` **不报错但返回 `complex64`**（相对误差 0.00e+00 证明它只是把 fp16 升采样成 fp32 再算），且因多一次升采样拷贝反而更慢（8.2 ms vs fp32 4.7 ms）。**cupy 标准 API 不暴露原生半精度 cuFFT（CUFFT_R_16F/C_16F）**。

**结论**：要在 fp16 上真正翻倍，需要绕过 torch/cupy 直接用 cuFFT C API 写半精度 kernel，工程量大、且 2048 点累加在 fp16 下相对精度 ~1e-3 会带来 ~5% 检测风险——**性价比极低**。

---

## 7. 根本原因：15 W 功耗墙 → SM 时钟锁死 300 MHz（决定性发现）

**动机**：上面的优化加起来 −55 ms，还剩 85 ms，离 66 ms 仍差 20 ms。为什么同一块 1660 Ti 该翻倍的翻不了？查时钟。

**做法**：`sustained_bench.py` 连续跑 200 帧并在后台线程采样 `nvidia-smi` 时钟/功耗。

**结果（决定性）**：
```
连续 250 帧负载期间:  SM 时钟 = 300/300/300 MHz（满血 2100 MHz）
                     显存 = 810/810/810 MHz（满血 6000 MHz）
                     功耗 = 15–16.5 W（贴死 15 W 上限）
```
- `nvidia-smi -pl 80` → **"Changing power management limit is not supported"**（固件锁死，无法解除）。
- `nvidia-smi -lgc 300,2100` 虽执行成功，但负载下 SM 仍恒 300 MHz——功耗墙是约束，时钟范围设置无效。

**含义**：
1. 之前文档里「GPU 利用率 41–45%、功耗 ~17 W → 是代码 gap 而非硬件极限」的推断**是错的**。低利用率/低功耗的真实原因是 **GPU 被固件锁在 15 W**（满 TDP 80 W 的 1/5），导致 SM 只有 300 MHz（1/7），所有 kernel 慢 5–7×。
2. 这也是为什么 fp16 / CUDA Graph / TensorRT 全都不管用——**它们都是「省算力/省启动」的手段，而瓶颈是「算力被硬件砍到 1/7」**。在功耗墙下，任何软件手段都补不回那 6/7 的时钟。
3. 这个 15 W 锁很可能来自笔记本 BIOS/EC 的散热或「静音/节能」策略（i5-9300H 老平台常见）。**要在验证平台上达标，必须先解除功耗墙**（插电、切「高性能」电源模式、NVIDIA 控制面板「首选最高性能」、或 BIOS 里关掉 GPU 功耗限制）——这是硬件/OS 层操作，代码无能为力。

**✅ 已解除（2026-08-14）**：在 NVIDIA 控制面板把电源管理设为「首选最高性能」后，功耗墙消失。负载下 SM 升到 1815–1875 MHz、显存满血 6000 MHz、功耗 51–81 W（贴 80 W TDP）。同一份优化代码全链路单帧从 85.3 ms 掉到 **21.5 ms**，直接达标。

---

## 8. 最终数据汇总

| 版本 | 预处理 | 多普勒 | 峰值 | DOA | 单帧总 | 说明 |
|---|---:|---:|---:|---:|---:|---|
| 基线（原始，15 W 锁） | 72.9 | 50.8 | 6.7 | 8.7 | **139.7** | 5 次中位数 |
| 代码优化（删死代码+子带向量化+NCI 先 sum 后 roll，仍 15 W 锁） | ~26 | ~39 | ~6.7 | ~8.5 | **85.3** | 50 帧持续中位数 |
| 代码优化 + 解除功耗墙（首选最高性能） | **3.53** | **4.84** | **4.81** | **8.26** | **21.47** | 5 次中位数，CV<1% |

- 代码优化 −54.4 ms（−39%）；解除功耗墙再 −63.8 ms。**总计 139.7 → 21.5 ms（−84.6%），全链路远低于 66 ms 门槛 ✅**。
- 正确性全程不变：25 峰，4/4 真值 (80,100)(139,130)(139,210)(250,200) 精确检出。
- **CUDA Graph / autocast fp16 / 连续 FFT / cupy fp16 FFT 全部 ~0 或负收益**（原因见 §5–§7）——它们都不是真正的瓶颈，真正的瓶颈是功耗墙。

### 解锁后负载资源占用（sustained_bench 采样）

| 阶段 | SM 时钟 | 显存时钟 | 功耗 | GPU 利用率(mean) | 显存占用率(mean) |
|---|---|---|---|---|---|
| GPU-only（预处理+多普勒，200 帧） | 1815–1860 MHz | 6000 MHz | 52.7–80.8 W | 49.9% | 45.7% |
| 全链路（+峰值+DOA，50 帧） | 1875 MHz | 6000 MHz | 51.2–54.5 W | 58.2% | 53.2% |

> 解锁后 SM 从 300 → ~1875 MHz（6.25×）、显存 810 → 6000 MHz（7.4×），与「全链路 85.3 → 20.6 ms（约 4.1×）」一致（并非线性 6×，因为峰值/DOA 段还含 CPU 字典组装与同步开销，不随 GPU 时钟线性缩放）。

## 9. 对论文与 Orin 的意义

1. 结论修正：**验证平台 66 ms 达标的决定性一步是解除 15 W 功耗墙**（NVIDIA 控制面板「首选最高性能」），而不是纯代码。代码优化负责把「有锁」下的 139.7 压到 85.3，功耗墙解除负责把 85.3 进一步压到 21.5。
2. 验证平台最终 GPU 单帧 **21.5 ms**，已满足 15 Hz 帧时限（66 ms）且余量充足；同机 CPU 基线（i5-9300H）1462.5 ms，**同机加速比 ≈ 68×**（口径：优化 GPU vs 原始 numpy CPU，CPU 未做同等清理，见 §10）。
3. 目标平台 Jetson AGX Orin 无 15 W 锁、SM 满频，同样代码预期与验证平台解锁态同量级或更快，落在 35–60 ms 理想区间无悬念；最终 Orin 数值仍以 Orin 同机实测为准。

## 10. 加速比口径提醒（写论文时注意）

- 1462.5 / 21.5 ≈ 68× 是「**优化后 GPU vs 原始 numpy CPU**」。CPU 侧尚未做同等的死代码清理与向量化，若 CPU 也清理，CPU 基线会略降、加速比会略减，但仍保持数十倍量级。
- 论文若给「GPU vs CPU 加速比」，应说明两者算法等价（逐峰 DOA 已用批量路径对齐），并注明 CPU 是串行 numpy、GPU 是优化后的 CUDA 实现；避免把「代码优化收益」与「GPU 并行收益」混为一谈。

---

## 附：踩坑清单（调试经验）

1. `profile` 的 `bench()` 若返回中位数 float 而非张量，后续 `torch.mean(tensor, dim)` 会报 `got (float, dim=tuple)` → 计时与计算分离。
2. 远程是 Windows `cmd.exe`：没有 `tail`、没有 bash 管道；用 `&&` 串命令、`set VAR=value` 设环境变量。scp 用正斜杠路径，别在 Git Bash 里拼反斜杠变量。
3. `peak_detection.py` 有 `from ft_framework.signal_process.calibration import ...` → 跑之前必须 `set PYTHONPATH=C:\Users\LZY\exp`。
4. CUDA Graph 捕获期间禁止 `torch.tensor(numpy)`（host→device 拷贝）→ 索引张量要预创建缓存。
5. 子带向量化的 stride 是 `n_subbands`(32) 不是 `doppler_step`(16)，写错会导致多普勒门减半、检测错误。
6. `cupy.fft.rfft(float16)` 会**静默升采样到 fp32**（返回 complex64，误差 0），不是真半精度；真半精度要走 cuFFT C API。
