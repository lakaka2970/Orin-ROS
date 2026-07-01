# FT Radar Fusion Framework — ROS2

基于 ROS2 的雷达-相机-车辆多传感器融合感知框架，运行于 **NVIDIA Jetson AGX Orin**。

| 操作系统 | ROS2 发行版 | Python | 说明 |
|---------|------------|--------|------|
| Ubuntu 20.04 | **Foxy** | 3.8 | 官方原生支持 |
| Ubuntu 22.04 | **Humble** | 3.10 | 官方原生支持 |

> 两个发行版 rclpy API 兼容，项目代码无需修改。脚本自动检测并加载对应环境。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20AGX%20Orin-orange)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)

---

## 目录

- [快速开始](#快速开始)
- [启动模式](#启动模式)
- [系统架构](#系统架构)
- [节点说明](#节点说明)
- [话题总表](#话题总表)
- [消息定义](#消息定义)
- [时间戳机制](#时间戳机制)
- [参数配置](#参数配置)
- [Logging 系统](#logging-系统)
- [RViz 可视化](#rviz-可视化)
- [开发指南](#开发指南)
- [项目结构](#项目结构)

---

## 快速开始

```bash
cd ~/Orin-ROS

# 0a. (仅首次) 安装系统依赖 + ROS2 + CycloneDDS
bash scripts/install_deps.sh --with-ros2 --with-cyclonedds

# 0b. (推荐) 免 source: 以后每次开终端自动加载环境
echo "source ~/Orin-ROS/scripts/env.sh" >> ~/.bashrc
source ~/.bashrc

# 1. 构建 (消息包 + Python 节点 + C++ rx 节点)
bash scripts/build.sh

# 2. 一键启动
bash scripts/start.sh python
```

> 完成步骤 0b 后，每次开新终端自动完成环境加载，不再需要手动 `source`。

### 安装说明

| 场景 | 命令 |
|------|------|
| **全新系统** (含 ROS2) | `bash scripts/install_deps.sh --with-ros2 --with-cyclonedds` |
| **已有 ROS2** | `bash scripts/install_deps.sh --with-cyclonedds` |
| **仅检查不安装** | `bash scripts/install_deps.sh --dry-run` |

> **CycloneDDS 必须安装**: 32 MiB/帧 ADC 消息在默认 FastDDS 下会被 UDP 分片导致大量丢包。CycloneDDS 内置共享内存传输，本地 IPC 不走网络栈。

### 启动选项

```bash
bash scripts/start.sh                   # 默认 cuda + C++ rx
bash scripts/start.sh python            # Python RSP + C++ rx
bash scripts/start.sh cuda --rviz       # CUDA + RViz
bash scripts/start.sh both_compare      # 双路对比
bash scripts/start.sh python --py-rx    # Python RSP + Python rx (回退)
```

### 验证运行状态

```bash
source scripts/env.sh

# 查看话题频率 (应接近设定值)
ros2 topic hz /adc/raw_data          # 期望 ~15 Hz
ros2 topic hz /camera/image_raw      # 期望 ~30 Hz
ros2 topic hz /vehicle/ego_motion    # 期望 ~50 Hz

# 查看所有节点
ros2 node list

# 查看话题连接
ros2 topic info /adc/raw_data
ros2 topic info /processing/radar/det_list
```

---

## 启动模式

### RSP 模式

| 模式 | 命令 | Python RSP | CUDA RSP | 适用场景 |
|------|------|:----------:|:--------:|----------|
| `cuda` (默认) | `rsp_mode:=cuda` | ✗ | ✓ | 生产部署 |
| `python` | `rsp_mode:=python` | ✓ | ✗ | 算法调试 |
| `both` | `rsp_mode:=both` | ✓ | ✓ | A/B 对比验证 |
| `both_compare` | `rsp_mode:=both_compare` | ✓ | ✓ | 差异分析 |

### Rx 实现

| 实现 | rx_impl 参数 | 发布频率 | 说明 |
|------|:---:|:---:|------|
| **C++** (默认) | `rx_impl:=cpp` | 稳定 15 Hz | 零拷贝, 噪声池预生成, CycloneDDS SHM 传输 |
| Python | `rx_impl:=python` | ~10 Hz | 原版实现, 用于对比验证 |

### 常见组合

```bash
# C++ rx + Python RSP (推荐调试)
bash scripts/start.sh python

# C++ rx + CUDA RSP (推荐生产)
bash scripts/start.sh cuda

# Python rx + Python RSP (回退验证)
bash scripts/start.sh python --py-rx

# 仅录制 ADC + 自车数据
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda \
  log_image:=false log_det_list:=false log_obj_list:=false
```

---

## 系统架构

### 数据流

```
[ADC Rx C++] ──→ /adc/raw_data (32MiB/帧, 15Hz, SHM传输)
                    ├──→ [RSP Python / CUDA] ──→ /processing/radar/det_list
                    └──→ [Logging] ──→ output/ft_dataset/

[Camera Rx C++] ──→ /camera/image_raw ──→ [Rviz_Image] [Logging]

[Vehicle Rx C++] ──→ /vehicle/ego_motion ──→ [RSP] [Logging]
```

### 四层结构

| 层级 | 节点 | 实现 | 职责 |
|:----:|------|:----:|------|
| 第一层 | ADC Rx, Camera Rx, Vehicle Data Rx | **C++** | 数据采集, 零拷贝发布 |
| 第二层 | RSP MIL Python, RSP Cuda | Python | 雷达信号处理 |
| 第三层 | Rviz_radar, Rviz_Image, Logging | Python | 可视化, 日志记录 |
| 第四层 | Object Detection 3D, Rviz_Ruler | Python | 目标检测, 坐标标尺 |

> 第一层 rx 节点使用 C++ 实现（`ft_rx_cpp` 包），利用零拷贝、噪声池预生成、CycloneDDS SHM 传输保证 15 Hz 稳定发布。第二至四层保留 Python 便于算法迭代。

---

## 节点说明

### 第一层：数据采集 (C++)

| 节点 | 频率 | 输出话题 | 数据量 | 说明 |
|------|:----:|------|------|------|
| **ADC Rx** | 15 Hz | `/adc/raw_data` | 32 MiB/帧 | ADC 原始数据 (uint8 字节流) |
| **Camera Rx** | 30 Hz | `/camera/image_raw` | ~2.7 MB/帧 | 测试图案 (1280×720 BGR) |
| **Vehicle Data Rx** | 50 Hz | `/vehicle/ego_motion` | < 1 KB/帧 | 车速/偏航角/转向/加速度/挡位 |

### C++ Rx 优化要点

| 优化 | 效果 |
|------|------|
| 噪声池预生成 (4x 帧大小) | 消除每帧 MT19937 开销 |
| 单次 memcpy 入消息 | 替代 Python tolist() 的 16.7M 对象分配 |
| `uint8[]` 消息类型 | ROS2 C 序列化走 memcpy 快速路径 |
| CycloneDDS SHM 传输 | 32MiB 不走 UDP 分片, 本地 IPC 直通 |
| Best Effort QoS | 模拟场景无需可靠传输, 消除 ACK 等待 |

### 第二层：信号处理 (Python)

| 节点 | 输入 | 输出 | 说明 |
|------|------|------|------|
| **RSP MIL Python** | ADC + EgoMotion | `/processing/radar/det_list` | SNR≥10dB，~30 检测点 |
| **RSP Cuda** | ADC + EgoMotion | `/processing/radar/det_list` 或 `*_cuda` | SNR≥8dB，~45 检测点 |

### 第三层：可视化与日志 (Python)

| 节点 | 功能 |
|------|------|
| **Rviz_radar** | 合并 DetList/ObjList/Ruler → PointCloud2 + MarkerArray |
| **Rviz_Image** | 图像叠加帧号和时间戳信息 |
| **Logging** | 5 通道独立开关, 6 种输出格式, 异步写盘 |

### 第四层：高级感知 (Python)

| 节点 | 功能 |
|------|------|
| **Object Detection 3D** | 欧氏聚类 (模拟 AI) → 14 字段 3D 目标 |
| **Rviz_Ruler** | 坐标尺参考标记 |

---

## 话题总表

| 话题 | 消息类型 | Pub | Sub |
|------|----------|:---:|:---:|
| `/adc/raw_data` | `AdcRawData` | adc_rx (C++) | rsp_*, logging_node |
| `/camera/image_raw` | `sensor_msgs/Image` | camera_rx (C++) | rviz_image, logging_node |
| `/vehicle/ego_motion` | `EgoMotion` | vehicle_data_rx (C++) | rsp_*, logging_node |
| `/processing/radar/det_list` | `DetList` | rsp_mil_python / rsp_cuda | rviz_radar, obj_detection_3d, logging_node |
| `/processing/radar/det_list_cuda` | `DetList` | rsp_cuda (双路模式) | rviz_radar, logging_node |
| `/perception/objects` | `ObjList` | object_detection_3d | rviz_radar, logging_node |
| `/visualization/ruler` | `MarkerArray` | rviz_ruler | rviz_radar |
| `/visualization/radar/display` | `PointCloud2` | rviz_radar | RViz |
| `/visualization/radar/boxes` | `MarkerArray` | rviz_radar, obj_detection_3d | RViz |
| `/visualization/radar/frame_info` | `MarkerArray` | rviz_radar | RViz |
| `/visualization/camera/display` | `Image` | rviz_image | RViz |

---

## 消息定义

### 自定义消息 `ft_radar_msgs`

| 消息 | 字段数 | 说明 |
|------|:------:|------|
| `AdcRawData` | 4 + header | ADC 原始字节流 (uint8[]) |
| `DetPoint` | 14 | 雷达检测目标点 |
| `DetList` | header + DetPoint[] | 检测列表 |
| `Object3D` | 14 | 3D 目标 |
| `ObjList` | header + Object3D[] | 目标列表 |
| `EgoMotion` | 7 + header + is_default | 自车运动数据 |

---

## 时间戳机制

- **时钟源**: `time.monotonic_ns()`, 微秒精度
- **注入位置**: rx 节点采集数据第一时间注入 `header.stamp`
- **透传规则**: 下游节点禁止覆盖, 必须原样传递
- **可视化例外**: rviz_radar / rviz_ruler 使用 `get_clock().now()`

---

## 参数配置

配置文件: [`config/ft_radar_params.yaml`](config/ft_radar_params.yaml)

### 常用参数

| 节点 | 参数 | 默认值 | 说明 |
|------|------|:------:|------|
| adc_rx | `fps` | 15.0 | ADC 帧率 |
| adc_rx | `sim_noise_pool_factor` | 4 | 噪声池倍数 |
| camera_rx | `fps` | 30.0 | 相机帧率 |
| vehicle_data_rx | `fps` | 50.0 | 车辆数据帧率 |
| vehicle_data_rx | `timeout_cycles` | 3 | 超时回退默认值 |
| rsp_mil_python | `snr_threshold` | 10.0 | SNR 阈值 (dB) |
| rsp_cuda | `snr_threshold` | 8.0 | SNR 阈值 (dB) |
| logging_node | `output_dir` | output/ft_dataset | 日志输出目录 |

---

## Logging 系统

### 通道

| 通道 | 开关参数 | 最大帧数 | 输出 |
|:----:|----------|:--------:|------|
| ADC | `enable_adc` | 100 | `adc_data/{ts}.bin` |
| Image | `enable_image` | 1000 | `camera_front_center/{ts}.jpg` |
| Det_List | `enable_det_list` | 1000 | `pc_csv_radar_front_center/{ts}.csv` |
| Ego_Motion | `enable_ego_motion` | 1000 | `ego_motion.csv` |
| Obj_List | `enable_obj_list` | 1000 | `obj_csv_radar/{ts}.csv` |

### 数据格式

ADC `.bin` 文件 = 20 字节 header + int16[] payload:

```
[0-7]   timestamp_us (uint64 LE)
[8-11]  num_rows (uint32 LE)
[12-15] num_chirps_per_row (uint32 LE)
[16-19] num_samples_per_chirp (uint32 LE)
[20-]   uint8[] payload  → np.frombuffer(payload, dtype=np.int16)
```

---

## RViz 可视化

```bash
rviz2 -d config/ft_radar.rviz
```

### TF 坐标系

| 坐标系 | 父系 | 偏移 |
|--------|------|------|
| `map` | — | 世界原点 |
| `radar` | map | z = 0.5m |
| `camera` | radar | z = 1.2m |
| `base_link` | — | 车辆本体 |

---

## 开发指南

### 代码约定

- Python 节点: `配置区 → 工具函数 → 节点类 → main()`
- C++ 节点: CRTP 基类 (`rx_node_base.hpp`) → 子类实现 `generate_and_publish()`
- 所有节点包含清理逻辑

### 添加新节点

- **Python**: 在 `ft_framework/` 创建 `.py` → `setup.py` 注册 → launch 添加
- **C++**: 在 `ft_rx_cpp/src/` 创建 `.cpp` → `CMakeLists.txt` 添加 `add_executable`

### 接入真实硬件

| 当前模拟 | 替换方案 |
|---------|---------|
| 噪声池随机切片 | v4l2 DMA 读取 |
| OpenCV 测试图案 | GMSL/FPD-Link 相机驱动 |
| 正态分布模拟车速 | CAN/ETH 总线解析 |
| 随机点 + SNR 过滤 | 真实 Range-FFT + CFAR |
| 欧氏聚类 | TensorRT AI 推理 |

---

## 项目结构

```
Orin-ROS/
├── README.md
├── src/
│   ├── ft_radar_msgs/              # 自定义消息 (ament_cmake)
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── msg/                    # 6 个 .msg 文件
│   ├── ft_framework/               # Python 节点 (ament_python)
│   │   ├── package.xml / setup.py
│   │   ├── launch/
│   │   │   └── ft_radar_launch.py
│   │   └── ft_framework/           # 10 个节点 + common.py + perf_profiler.py
│   └── ft_rx_cpp/                  # C++ rx 节点 (ament_cmake)
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/ft_rx_cpp/
│       │   └── rx_node_base.hpp    # CRTP 基类 + PerfTimer
│       └── src/
│           ├── adc_rx.cpp          # ADC 采集 (零拷贝, 噪声池)
│           ├── camera_rx.cpp       # 相机采集 (cv::Mat + cv_bridge)
│           └── vehicle_data_rx.cpp # 车辆数据 (<random>)
├── config/
│   ├── ft_radar_params.yaml
│   ├── ft_radar.rviz
│   └── fastdds_shm.xml            # FastDDS SHM 配置 (回退用)
├── scripts/
│   ├── env.sh                      # 环境加载 (自动 CycloneDDS)
│   ├── install_deps.sh             # 依赖安装 (含 CycloneDDS)
│   ├── build.sh                    # 一键构建 (3 个包)
│   ├── start.sh                    # 一键启动 (推荐入口)
│   └── launch_all.sh               # 兼容入口 → start.sh
├── docs/                           # 架构与优化文档
└── output/ft_dataset/              # 日志输出
```

---

## 许可证

Apache-2.0

**作者**: zhengyuan.liu
**创建**: 2026.6.8
**更新**: 2026.6.30
