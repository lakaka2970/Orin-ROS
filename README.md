# FT Radar Fusion Framework V2 — ROS2

基于 ROS2 Foxy 的 16T16R 毫米波雷达多传感器融合感知框架，运行于 **NVIDIA Jetson AGX Orin**。

| 平台 | OS | ROS2 | Python | JetPack |
|------|-----|------|--------|---------|
| Jetson AGX Orin 64GB | Ubuntu 20.04 | **Foxy** | 3.8 | 5.1.2 (L4T R35.4.1) |

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-brightgreen)](https://docs.ros.org/en/foxy/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20AGX%20Orin-orange)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
[![DDS](https://img.shields.io/badge/DDS-FastDDS%20SHM-blue)](https://fast-dds.docs.eprosima.com/)

---

## V2 架构核心特性

| 特性 | 说明 |
|------|------|
| **文件路径发布** | ADC/Camera 写入文件后发布路径，DDS 带宽从 480 MB/s 降至 ~20 KB/s（降低 99.99%） |
| **内置 Logging** | 数据采集层 C++ 节点内置 Logging，无独立 logging_node |
| **V4L2 硬件时间戳** | ADC/Camera/Ego 统一使用 CLOCK_MONOTONIC 内核时间戳，微秒精度 |
| **分层 Buffer 管理** | V4L2 buffer 立即释放，DDR 队列持有数据直到 RSP 处理完成 |
| **NVMe SSD 双存储** | 自动检测 NVMe SSD，回退 eMMC + 最大帧数限制 |
| **运行模式管理** | FT_DEBUG_MODE（含 Logging）/ FT_RUNNING_MODE（仅实时处理） |
| **4 种 Logging 模式** | ADC / RD Cell List / Det List / Idle |
| **Raw V4L2 Camera** | MJPEG 直接写盘，零 OpenCV 依赖 |
| **SocketCAN** | 5 报文解析，SIOCGSTAMP 硬件时间戳，Motorola 字节序 |

---

## 快速开始

```bash
cd ~/Orin-ROS

# 0. (仅首次) 安装依赖 + 免 source 配置
bash scripts/install_deps.sh --with-ros2
echo "source ~/Orin-ROS/scripts/env.sh" >> ~/.bashrc
source ~/.bashrc

# 1. 构建 (含 Jetson 硬件初始化)
bash scripts/build.sh

# 2. 启动 (默认: FT_DEBUG_MODE + ADC_MODE + CUDA RSP)
bash scripts/start.sh

# 3. (可选) 启动 RViz 可视化
bash scripts/start.sh --rviz
```

### 启动选项

```bash
# ── 运行模式 ──
bash scripts/start.sh                    # FT_DEBUG_MODE + ADC_MODE (默认)
bash scripts/start.sh --running          # FT_RUNNING_MODE (仅实时处理, 无 Logging)

# ── Logging 模式 ──
bash scripts/start.sh --adc-mode         # ADC Mode (录制 ADC + Ego + Video + Det)
bash scripts/start.sh --rdcell-mode      # RD Cell List Mode (录制 RX NCI + RD Cell)
bash scripts/start.sh --det-mode         # Det List Mode (仅录制 Det + Ego + Video)
bash scripts/start.sh --idle             # Idle Mode (不录制任何数据)

# ── 其他 ──
bash scripts/start.sh --no-adc           # 不启动 adc_rx 节点
bash scripts/start.sh --rviz             # 同时启动 RViz2

# ── 开发管线 (雷达硬件调试) ──
bash scripts/start.sh --capture-only     # 仅采集雷达原始数据
bash scripts/start.sh --capture-only --rsps  # 采集 + RSPS 离线点云可视化
```

---

## 系统架构

### 数据流

```
[V4L2 双 CTRX] ─DQBUF─→ [adc_rx C++]
                            ├─ memcpy → DDR (文件落盘, Logging)
                            ├─ 立即 QBUF (释放 V4L2 buffer)
                            ├─ 发布 /adc/file_path (AdcFilePath)
                            └─ 等待 /system/processing_complete → 释放 DDR 槽位

[USB Camera] ─DQBUF─→ [camera_rx C++]
                         ├─ MJPEG 直接写入 .jpg (内置 Logging)
                         └─ 发布 /camera/file_path (CameraFilePath)

[SocketCAN] ─→ [vehicle_data_rx C++]
                  ├─ 5 报文解析 + SIOCGSTAMP 硬件时间戳
                  ├─ 每 20ms 写入 ego_motion.csv (内置 Logging)
                  └─ 发布 /vehicle/ego_motion (EgoMotion, 50 Hz)

[adc_rx] ─/adc/file_path─→ [rsp_cuda Python+GPU]
                              ├─ 读取 ADC 文件 → GPU RSP 流水线 (66ms 时限)
                              ├─ 发布 /processing/radar/det_list_cuda (DetList)
                              └─ 发布 /system/processing_complete (通知 adc_rx)
```

### 四层节点结构

| 层级 | 节点 | 实现 | 职责 |
|:----:|------|:----:|------|
| 第一层 | adc_rx, camera_rx, vehicle_data_rx | **C++** | 数据采集 + 内置 Logging + 文件路径发布 |
| 第二层 | rsp_cuda | Python+GPU | 雷达信号处理 (66ms 时限) |
| 第三层 | rviz_radar, rviz_image, rviz_ruler | Python | 可视化 |
| 第四层 | object_detection_3d | Python | 3D 目标检测 (可选) |
| 系统 | system_monitor | Python | 磁盘/内存/进程/帧周期监控 |

### Buffer 管理

```
V4L2 buffer (≤8个/设备)              DDR 队列 (文件)
┌───────────────────────┐            ┌──────────────────────────┐
│ DQBUF                 │            │ 数据持有                  │
│   ↓                   │            │   ↓                      │
│ memcpy → DDR (文件)   │──────────→ │ 提供文件路径给 RSP        │
│   ↓                   │            │   ↓                      │
│ 立即 QBUF ✓          │            │ 等待 processing_complete  │
│ (周转不受RSP限制)     │            │   ↓                      │
└───────────────────────┘            │ 释放槽位 ✓               │
                                     └──────────────────────────┘
```

---

## 话题总表

| 话题 | 消息类型 | 发布者 | 订阅者 | 频率 |
|------|---------|--------|--------|:----:|
| `/adc/file_path` | AdcFilePath | adc_rx | rsp_cuda, system_monitor | 15 Hz |
| `/camera/file_path` | CameraFilePath | camera_rx | rviz_image, system_monitor | 15 Hz |
| `/vehicle/ego_motion` | EgoMotion | vehicle_data_rx | rsp_cuda | 50 Hz |
| `/processing/radar/det_list_cuda` | DetList | rsp_cuda | rviz_radar, object_detection_3d | 15 Hz |
| `/perception/objects` | ObjList | object_detection_3d | rviz_radar | 15 Hz |
| `/system/monitor` | SystemMonitor | system_monitor | — | 1 Hz |
| `/system/stop_all` | std_msgs/Bool | adc_rx | — | 事件 |
| `/system/processing_complete` | std_msgs/Bool | rsp_cuda | adc_rx | 事件 |
| `/visualization/ruler` | MarkerArray | rviz_ruler | rviz_radar | 2 Hz |
| `/visualization/radar/display` | PointCloud2 | rviz_radar | RViz | 15 Hz |
| `/visualization/radar/boxes` | MarkerArray | rviz_radar | RViz | 15 Hz |
| `/visualization/camera/display` | Image | rviz_image | RViz | 15 Hz |

---

## 消息定义 (ft_radar_msgs)

| 消息 | 说明 |
|------|------|
| `AdcFilePath` | ADC 文件路径 + 维度信息 (替代 32MB AdcRawData) |
| `CameraFilePath` | Camera 文件路径 + 图像尺寸 |
| `SystemMonitor` | 磁盘/内存/进程/时钟/帧周期监控数据 |
| `DetPoint` | 雷达检测点 (41 字段) |
| `DetList` | 检测点列表 + 帧信息 |
| `EgoMotion` | 自车运动 (7 字段 + is_default) |
| `Object3D` | 3D 目标 (14 字段) |
| `ObjList` | 3D 目标列表 |

---

## 时间戳机制

| 传感器 | 时间戳来源 | 精度 |
|--------|-----------|------|
| ADC | V4L2 `v4l2_buffer.timestamp` | 微秒 (CLOCK_MONOTONIC) |
| Camera | V4L2 `v4l2_buffer.timestamp` | 微秒 (CLOCK_MONOTONIC) |
| Ego | SocketCAN `SIOCGSTAMP` ioctl | 微秒 (CLOCK_MONOTONIC) |
| RSP 输出 | 透传 ADC 时间戳 | — |

三者同源 (CLOCK_MONOTONIC)，无时钟偏移，可直接对齐。下游节点透传 `header.stamp`，禁止覆盖。

---

## RSP GPU 性能优化（2026-08 更新）

针对 15 Hz 帧时限（66 ms/帧）对 GPU RSP 全链路进行了 **profile 驱动的系统性优化**（验证平台 GTX 1660 Ti，合成 32 MB 帧）：

| 版本 | 单帧总耗时 | 说明 |
|------|---------:|------|
| 基线（PyTorch CUDA fp32，15 W 功耗墙锁定） | 139.7 ms | 预处理 72.9 + 多普勒 50.8 + 峰值 6.7 + DOA 8.7 |
| 代码优化后 | 85.3 ms（−39%） | 删除死代码（时域干扰抑制块 −35 ms）、子带峰值循环向量化（13.1→0.24 ms）、RX NCI「先 sum 后 roll」 |
| 代码优化 + 解除功耗墙 | **21.5 ms** ✅ | 「首选最高性能」解锁后 SM 时钟 300→1875 MHz，全链路达标且余量充足 |

- **正确性全程不变**：25 峰，4/4 真值目标精确检出；同机 CPU 基线（numpy 串行）1462.5 ms，**加速比 ≈ 68×**。
- **决定性发现**：验证平台 GPU 被固件锁死在 15 W 功耗墙（满 TDP 的 1/5），SM 恒为 300 MHz（满血的 1/7）。fp16 / CUDA Graph / 连续 FFT 布局 / cupy 半精度 FFT 实测均无收益或负收益——瓶颈是硬件时钟而非代码。
- 目标平台 Jetson AGX Orin 无功耗锁、SM 满频，同代码预期落在 35–60 ms 理想区间。

> 完整策略变更记录与踩坑清单见 [`signal_process/GPU优化策略变更记录.md`](src/ft_framework/ft_framework/signal_process/GPU优化策略变更记录.md)

**Benchmark 工具集**（`ft_framework/signal_process/`）：

| 脚本 | 用途 |
|------|------|
| `profile_hotspots.py` | 微操作级热点计时（加窗/rfft/gather/topk…逐项中位数） |
| `run_timing_gpu.py` / `run_timing_gpu_opt.py` | 原始 vs 优化版单帧全链路耗时（支持 `--graph` CUDA Graph 对照） |
| `sustained_bench.py` | 持续负载压测 + 后台采样 nvidia-smi 时钟/功耗 |
| `bench_reps.py` / `summarize_bench.py` | 多轮重复基准与结果汇总 |
| `test_fp16_fft.py` / `test_opt_variants.py` | 半精度 FFT 与各优化变体的正确性/收益验证 |
| `preprocessing_opt.py` / `doppler_opt.py` | 优化后的信号处理实现（数值严格等价） |

---

## 参数配置

配置文件: [`config/ft_radar_params.yaml`](config/ft_radar_params.yaml)（**最高优先级**）

### 系统参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `system.operation_mode` | FT_DEBUG_MODE | FT_DEBUG_MODE / FT_RUNNING_MODE |
| `system.logging_mode` | ADC_MODE | ADC_MODE / RD_CELL_LIST_MODE / DET_LIST_MODE / IDLE_MODE |
| `system.logging_output_dir` | "" (自动检测) | 空=NVMe SSD，回退 eMMC |
| `system.enable_warmup` | true | V4L2 warm-up 开关 |
| `system.warmup_sec` | 5.0 | warm-up 时长 |

### 节点参数

| 节点 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| adc_rx | fps | 15 | ADC 帧率 |
| adc_rx | device_path_ctrx0 | /dev/radar_ctrx0 | V4L2 设备 |
| adc_rx | device_path_ctrx1 | /dev/radar_ctrx1 | V4L2 设备 |
| adc_rx | logging_max_frames | 100 | eMMC 最大帧数 |
| adc_rx | rsp_timeout_ms | 100 | 等待 RSP 完成超时 |
| camera_rx | fps | 15 | 相机帧率 |
| camera_rx | device_path | /dev/camera_capture | V4L2 设备 |
| vehicle_data_rx | fps | 50 | 发布频率 |
| vehicle_data_rx | can_interface | can0 | SocketCAN 接口 |
| rsp_cuda | processing_fps | 15.0 | 处理帧率 |
| rsp_cuda | snr_threshold | 8.0 | SNR 阈值 (dB) |

---

## Logging 系统 (内置)

V2 架构中 Logging 功能内置在各 C++ Rx 节点中，无独立 logging_node。

| 模式 | ADC | RX NCI | RD Cell | Det List | Ego | Video |
|------|:---:|:------:|:-------:|:--------:|:---:|:-----:|
| **ADC Mode** | ✓ | ✗ | ✗ | ✓ | ✓ | ✓ |
| **RD Cell List Mode** | ✗ | ✓ | ✓ | ✓ | ✓ | ✓ |
| **Det List Mode** | ✗ | ✗ | ✗ | ✓ | ✓ | ✓ |
| **Idle Mode** | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |

### 存储策略

- **NVMe SSD** (`/mnt/nvme`): 主存储，无帧数限制
- **eMMC** (`/mnt/emmc`): 备选，自动计算最大帧数（预留 10% 安全空间）
- 达到上限时发布 `/system/stop_all` 停止信号

---

## 硬件设备

| 设备 | 型号 | 接口 | 帧率 | 数据量/帧 |
|------|------|------|:----:|:---------:|
| 雷达 | Infineon CTRX8188F (16T16R) | GMSL (MAX929x) | 15 Hz | 32 MiB |
| 摄像头 | Rmoncam A2 1080P | USB 3.0 UVC | 15 Hz | ~50 KB (QVGA JPEG) |
| CAN | PCAN-USB | SocketCAN | 50 Hz | < 1 KB |
| 存储 | 1TB NVMe SSD | PCIe 3.0 x4 | — | ≥ 200 MB/s 写入 |

### udev 持久化

```bash
sudo cp scripts/99-ft-sensors.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 验证: ls -la /dev/radar_ctrx0 /dev/radar_ctrx1 /dev/camera_capture
```

---

## 项目结构

```
Orin-ROS/
├── config/
│   ├── ft_radar_params.yaml        # 全局参数 (最高优先级)
│   ├── ft_radar.rviz               # RViz 配置
│   └── fastdds.xml                 # FastDDS SHM 配置 (512MB)
├── scripts/
│   ├── env.sh                      # 环境加载 (FastDDS + SHM)
│   ├── build.sh                    # 构建 (含 Jetson 硬件初始化)
│   ├── start.sh                    # 一键启动 (运行模式 + Logging 模式)
│   ├── install_deps.sh             # 依赖安装
│   └── 99-ft-sensors.rules         # udev 规则
├── src/
│   ├── ft_radar_msgs/              # 自定义消息 (8 个 .msg)
│   ├── ft_rx_cpp/                  # C++ 数据采集层 (内置 Logging)
│   │   ├── include/ft_rx_cpp/
│   │   │   ├── rx_node_base.hpp    # CRTP 基类 + Profiler
│   │   │   └── perf_profiler.hpp
│   │   └── src/
│   │       ├── adc_rx.cpp          # V4L2 双设备 + 文件路径 + DDR 队列
│   │       ├── camera_rx.cpp       # Raw V4L2 MJPEG + 文件路径
│   │       └── vehicle_data_rx.cpp # SocketCAN + Ego CSV
│   ├── ft_framework/               # Python 节点
│   │   ├── launch/ft_radar_launch.py
│   │   └── ft_framework/
│   │       ├── rsp_cuda.py         # GPU RSP (66ms 时限)
│   │       ├── rviz_radar.py       # 点云可视化
│   │       ├── rviz_image.py       # 图像可视化
│   │       ├── rviz_ruler.py       # 坐标标尺
│   │       ├── object_detection_3d.py  # 3D 目标检测 (可选)
│   │       ├── system_monitor.py   # 系统监控
│   │       └── signal_process/     # RSP 算法库 (GPU+CPU) + GPU 性能优化与基准工具
│   │           ├── preprocessing_opt.py / doppler_opt.py  # 优化版信号处理
│   │           ├── profile_hotspots.py / run_timing_gpu*.py / sustained_bench.py  # 基准工具
│   │           └── GPU优化策略变更记录.md  # 优化策略变更记录（139.7→21.5 ms）
│   └── integration-carkit88c0-gmsl/  # GMSL 雷达硬件驱动
└── docs/
    ├── orin_sw_architecture_v2.md  # V2 架构文档 (权威)
    ├── FVR60_XD_Requirement_20260715.md
    └── 项目开发工作记录.md
```

---

## 开发指南

### DDS 策略

仅使用 **FastDDS** (`rmw_fastrtps_cpp`)，内置 SHM 共享内存传输。`env.sh` 强制设置并清理 CycloneDDS 残留。

### 构建

```bash
bash scripts/build.sh                # 增量构建
bash scripts/build.sh --clean        # 清理重建
bash scripts/build.sh --skip-init-hw # 跳过硬件初始化
```

构建顺序: `ft_radar_msgs` → `ft_framework` → `ft_rx_cpp`

### 验证

```bash
source scripts/env.sh
ros2 topic hz /adc/file_path         # ~15 Hz
ros2 topic hz /vehicle/ego_motion    # ~50 Hz
ros2 topic hz /processing/radar/det_list_cuda  # ~15 Hz
ros2 node list
```

---

## 许可证

Apache-2.0

**作者**: zhengyuan.liu
**创建**: 2026-06-08
**V2 更新**: 2026-07-26
**性能优化章节更新**: 2026-08-25
