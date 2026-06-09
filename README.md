# FT Radar Fusion Framework — ROS2 Humble

基于 ROS2 Humble 的雷达-相机-车辆多传感器融合感知框架，运行于 **NVIDIA Jetson AGX Orin 64GB**。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20AGX%20Orin-orange)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
[![License](https://img.shields.io/badge/License-Apache--2.0-yellow)](LICENSE)

---

## 目录

- [系统概述](#系统概述)
- [快速开始](#快速开始)
- [启动模式](#启动模式)
- [系统架构](#系统架构)
- [节点说明](#节点说明)
- [话题总表](#话题总表)
- [消息定义](#消息定义)
- [数据格式](#数据格式)
- [时间戳机制](#时间戳机制)
- [参数配置](#参数配置)
- [Logging 系统](#logging-系统)
- [开发指南](#开发指南)
- [项目结构](#项目结构)

---

## 系统概述

本框架实现了一个完整的 **雷达-相机-车辆多传感器融合感知系统**，共 10 个 ROS2 节点，分布在四个逻辑层级中：

### 数据流架构

```
[ADC Rx] ─pub→ /adc/raw_data ─────────────────────────────────→ [Logging]
                      │
              ┌───────┴───────┐
              ▼               ▼
    [RSP MIL Python]     [RSP Cuda]         [Camera Rx] ─pub→ /camera/image_raw ─→ [Rviz_Image]
              │               │                                        │
     ┌────────┘               └────────┐                               └─→ [Logging]
     ▼                                 ▼
/processing/radar/det_list   /processing/radar/det_list_cuda
     │                                                    │
     ├────────────→ [3D Object Detection] ──pub→ /perception/objects ──→ [Rviz_radar, Logging]
     │                                                         │
     └─────────────────────→ [Rviz_radar] ←──────pub← [Rviz_Ruler] ← /visualization/ruler

[Vehicle Rx] ─pub→ /vehicle/ego_motion ──→ [RSP Python, RSP Cuda, Logging]
```

### 核心特性

| 特性 | 说明 |
|------|------|
| **自定义消息** | 6 种 .msg 类型，与 FT_radar_dataset_requirement 完全对齐 |
| **双路径 RSP** | Python 和 CUDA 可切换/并行，4 种启动模式 |
| **全局时间戳** | 微秒 (μs) 精度，`time.monotonic_ns()` 统一时钟源，全程透传 |
| **参数集中管理** | 所有节点参数统一配置于 `config/ft_radar_params.yaml` |
| **Logging 系统** | 5 通道独立开关、帧数上限、异步写入、多格式输出（CSV/PCD/JPEG/BIN）|
| **运行时动态配置** | Logging 开关支持 `ros2 param set` 运行时切换 |
| **条件启动** | Launch 文件根据 `rsp_mode` 选择性启动 RSP 节点，0 冗余进程 |

---

## 快速开始

```bash
cd ~/Orin-ROS

# 1. 加载环境
source /opt/ros/humble/setup.bash

# 2. 构建消息包（首次需先构建 msg）
colcon build --packages-select ft_radar_msgs --symlink-install
source install/setup.bash

# 3. 构建功能包
colcon build --packages-select ft_framework --symlink-install
source install/setup.bash

# 4. 启动（默认 CUDA 模式）
ros2 launch ft_framework ft_radar_launch.py

# 5. 或使用一键脚本
bash scripts/launch_all.sh
```

### 构建脚本

```bash
bash scripts/build.sh            # 增量构建
bash scripts/build.sh --clean    # 全量重构建
```

---

## 启动模式

框架支持 **4 种 RSP 启动模式**，通过 `rsp_mode` Launch 参数控制：

### 模式一览

```bash
# CUDA 模式（默认）— 仅启动 CUDA RSP
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda

# Python 模式 — 仅启动 Python RSP
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=python

# 双路并行 — Python + CUDA 独立运行
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=both

# 双路对比 — 并行 + 输出差异对比
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=both_compare
```

### 模式对比

| 模式 | Python RSP | CUDA RSP | 主话题 | CUDA 话题 | 适用场景 |
|------|:----------:|:--------:|--------|-----------|----------|
| `cuda` | ✗ | ✓ | `/processing/radar/det_list` | — | 生产部署，GPU 加速 |
| `python` | ✓ | ✗ | `/processing/radar/det_list` | — | 开发调试，算法验证 |
| `both` | ✓ | ✓ | `/processing/radar/det_list` | `/processing/radar/det_list_cuda` | 对比验证 |
| `both_compare` | ✓ | ✓ | `/processing/radar/det_list` | `/processing/radar/det_list_cuda` | 差异分析 |

### 自定义 Logging 开关

```bash
# 仅录制 ADC 和 EgoMotion，关闭其他通道
ros2 launch ft_framework ft_radar_launch.py \
  rsp_mode:=cuda \
  log_image:=false \
  log_det_list:=false \
  log_obj_list:=false
```

---

## 系统架构

### 四层逻辑架构

```
┌──────────────────────────────────────────────────────────────────────┐
│ 第四层：高级感知与辅助层                                               │
│  ┌─────────────────────────┐    ┌──────────────────┐                 │
│  │   3D Object Detection   │    │   Rviz_Ruler     │                 │
│  │   (ObjList 14 字段)     │    │   (标尺参考)      │                 │
│  └──────────┬──────────────┘    └────────┬─────────┘                 │
│             │ /perception/objects        │ /visualization/ruler       │
├─────────────┼────────────────────────────┼──────────────────────────┤
│ 第三层：可视化与日志层                      │                           │
│  ┌──────────┴──────────┐  ┌───────────────┴──┐  ┌────────────────┐ │
│  │    Rviz_radar       │  │   Rviz_Image     │  │   Logging      │ │
│  │ 4 in / 4 out topics │  │ image→overlay    │  │ 5 channel I/O  │ │
│  └──────────┬──────────┘  └──────────────────┘  └───────┬────────┘ │
│     ↑ det_list ↑ obj_list  ↑ ruler                      ↑ all 5     │
├──────────────┴────────────┴───────────────────────────────┴────────┤
│ 第二层：雷达信号处理层（按 mode 条件启动）                              │
│  ┌──────────────────┐       ┌──────────────────┐                     │
│  │  RSP MIL Python  │       │   RSP Cuda       │                     │
│  │  SNR≥10dB, 30pts │       │  SNR≥8dB, 45pts  │                     │
│  │  DetPoint 14字段 │       │  DetPoint 14字段  │                     │
│  └────────┬─────────┘       └────────┬─────────┘                     │
│     ↑ /adc/raw_data + /vehicle/ego_motion                            │
├──────────┴──────────────────────────┴──────────────────────────────┤
│ 第一层：数据采集层                                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐            │
│  │  ADC Rx      │  │  Camera Rx   │  │  Vehicle Data Rx │            │
│  │  15Hz, 32MB  │  │  30Hz, TBD   │  │  50Hz, 7 fields  │            │
│  │  μs timestamp│  │  μs timestamp│  │  +default fallback│            │
│  └──────┬───────┘  └──────┬───────┘  └────────┬─────────┘            │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 节点说明

### 第一层：数据采集

| 节点 | 频率 | 消息类型 | 话题 | 功能 |
|------|:----:|----------|------|------|
| **ADC Rx** | 15 Hz | [`AdcRawData`](src/ft_radar_msgs/msg/AdcRawData.msg) | `/adc/raw_data` | 采集雷达 ADC 数据，注入 μs 时间戳 |
| **Camera Rx** | 30 Hz | `sensor_msgs/Image` | `/camera/image_raw` | 采集相机图像（格式 TBD，占位实现） |
| **Vehicle Data Rx** | 50 Hz | [`EgoMotion`](src/ft_radar_msgs/msg/EgoMotion.msg) | `/vehicle/ego_motion` | 采集车辆动态数据，含默认值机制 |

### 第二层：信号处理

| 节点 | 输入 | 输出 | 消息类型 | 功能 |
|------|------|------|----------|------|
| **RSP MIL Python** | ADC + EgoMotion | [`DetList`](src/ft_radar_msgs/msg/DetList.msg) | SNR 滤波 + 速度补偿 |
| **RSP Cuda** | ADC + EgoMotion | [`DetList`](src/ft_radar_msgs/msg/DetList.msg) | 同上，更低 SNR 阈值（更高灵敏度） |

### 第三层：可视化与日志

| 节点 | 输入 | 输出 | 功能 |
|------|------|------|------|
| **Rviz_radar** | DetList×2, ObjList, Ruler | PointCloud2 + MarkerArray + Image | 汇聚显示，按高度着色 |
| **Rviz_Image** | Image | Image (overlay) | 叠加帧号和时间戳 |
| **Logging** | 5 通道全部数据 | 6 种文件格式 | 5 独立开关，帧上限，异步写入 |

### 第四层：高级感知

| 节点 | 输入 | 输出 | 消息类型 | 功能 |
|------|------|------|----------|------|
| **3D Object Detection** | DetList | [`ObjList`](src/ft_radar_msgs/msg/ObjList.msg) | 欧氏聚类模拟 AI 检测 |
| **Rviz_Ruler** | — | MarkerArray | 发布坐标尺/参考标记 |

---

## 话题总表

| 话题 | 消息类型 | 发布者 | 订阅者 | 频率 |
|------|----------|--------|--------|:----:|
| `/adc/raw_data` | `ft_radar_msgs/AdcRawData` | `adc_rx` | `rsp_mil_python`, `rsp_cuda`, `logging_node` | 15 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | `camera_rx` | `rviz_image`, `logging_node` | 30 Hz |
| `/vehicle/ego_motion` | `ft_radar_msgs/EgoMotion` | `vehicle_data_rx` | `rsp_mil_python`, `rsp_cuda`, `logging_node` | 50 Hz |
| `/processing/radar/det_list` | `ft_radar_msgs/DetList` | `rsp_mil_python` / `rsp_cuda` | `rviz_radar`, `object_detection_3d`, `logging_node` | 10 Hz |
| `/processing/radar/det_list_cuda` | `ft_radar_msgs/DetList` | `rsp_cuda` (双路模式) | `rviz_radar`, `logging_node` | 10 Hz |
| `/perception/objects` | `ft_radar_msgs/ObjList` | `object_detection_3d` | `rviz_radar`, `logging_node` | — |
| `/visualization/ruler` | `visualization_msgs/MarkerArray` | `rviz_ruler` | `rviz_radar` | 2 Hz |
| `/visualization/radar/display` | `sensor_msgs/PointCloud2` | `rviz_radar` | (RViz) | 10 Hz |
| `/visualization/radar/boxes` | `visualization_msgs/MarkerArray` | `rviz_radar` / `object_detection_3d` | (RViz) | 10 Hz |
| `/visualization/radar/colorbar` | `sensor_msgs/Image` | `rviz_radar` | (RViz) | 10 Hz |
| `/visualization/radar/frame_info` | `visualization_msgs/MarkerArray` | `rviz_radar` | (RViz) | 10 Hz |
| `/visualization/camera/display` | `sensor_msgs/Image` | `rviz_image` | (RViz) | — |

---

## 消息定义

### 自定义消息包 `ft_radar_msgs`

| 消息 | 字段数 | 说明 |
|------|:------:|------|
| [`AdcRawData.msg`](src/ft_radar_msgs/msg/AdcRawData.msg) | 4 + header | ADC 原始数据（int16[] 大数组） |
| [`DetPoint.msg`](src/ft_radar_msgs/msg/DetPoint.msg) | **14** | 雷达检测目标点 |
| [`DetList.msg`](src/ft_radar_msgs/msg/DetList.msg) | header + DetPoint[] | 检测目标列表 |
| [`Object3D.msg`](src/ft_radar_msgs/msg/Object3D.msg) | **14** | 3D 目标（含跟踪信息、包围盒、运动状态） |
| [`ObjList.msg`](src/ft_radar_msgs/msg/ObjList.msg) | header + Object3D[] | 3D 目标列表 |
| [`EgoMotion.msg`](src/ft_radar_msgs/msg/EgoMotion.msg) | **7** + header + is_default | 自车运动数据 |

### DetPoint 14 字段

```
  x, y, z           — 车辆系空间坐标 (m)
  range, azimuth, elevation — 雷达系原视测量 (m, rad, rad)
  RCS, SNR          — 信号特征 (dBsm, dB)
  ambgt             — 速度模糊窗宽 (m/s)
  exist_prob        — 存在概率 [0,255]
  multi_tgt_prob    — 多目标概率 [0,255]
  ambgt_prob        — 模糊概率 [0,255]
  raw_doppler       — 原始多普勒速度 (m/s)
  idx               — 多普勒解模糊索引
```

### Object3D 14 字段

```
  object_id         — 跟踪 ID
  tracked_times     — 跟踪帧数
  score             — 置信度 [0,1]
  x, y, z           — bbox 中心 (m)
  l, w, h           — bbox 长宽高 (m)
  yaw               — 航向角 (rad)
  vx/vy/vz_absolute — 对地速度 (m/s)
  moving_state      — 运动状态枚举
```

---

## 时间戳机制

### 设计原则

- **时钟源**: 统一使用 `time.monotonic_ns()`（开发阶段），预留 PTP 硬件时钟接口
- **精度**: 微秒 (μs)
- **注入位置**: 所有 Rx 节点在数据采集第一时间注入
- **透传规则**: 后续节点**不得覆盖** `header.stamp`，仅在消息头中传递

### 实现

```python
def monotonic_us_stamp() -> tuple:
    now_ns = time.monotonic_ns()
    sec = int(now_ns // 1_000_000_000)
    nsec = int(now_ns % 1_000_000_000)
    return (sec, nsec)
```

所有节点共享此函数，Logging 系统使用 `get_timestamp_us()` 从中提取微秒整数用于文件命名。

---

## 参数配置

所有节点参数统一管理于 [`config/ft_radar_params.yaml`](config/ft_radar_params.yaml)。

### 配置层级

```
config/ft_radar_params.yaml        ★ 默认参数（全局）
  ↓
ros2 launch ... parameters=[...]    ★ Launch 时覆盖
  ↓
ros2 param set /node param value    ★ 运行时动态修改（Logging 开关支持）
```

### 主要参数速查

| 节点 | 参数 | 默认值 | 说明 |
|------|------|:------:|------|
| `adc_rx` | `fps` | 15 | 采集帧率 |
| `camera_rx` | `fps` | 30 | 采集帧率 |
| `vehicle_data_rx` | `fps` | 50 | 采集帧率 |
| `vehicle_data_rx` | `timeout_cycles` | 3 | 超时周期数 |
| `rsp_mil_python` | `snr_threshold` | 10.0 | SNR 阈值 (dB) |
| `rsp_cuda` | `snr_threshold` | 8.0 | SNR 阈值 (dB) |
| `object_detection_3d` | `cluster_distance` | 5.0 | 聚类距离 (m) |
| `rviz_ruler` | `ruler_axis` | x | 标尺方向 |
| `logging` | `max_frames.adc` | 100 | ADC 最大帧数 |

---

## Logging 系统

### 5 个独立通道

| 通道 | 开关参数 | 最大帧数 | 输出文件 |
|:----:|----------|:--------:|----------|
| ADC | `enable_adc` | 100 | `adc.bin`（二进制连续） |
| Image | `enable_image` | 1000 | `{timestamp_us}.jpg` |
| Det_List | `enable_det_list` | 1000 | `{timestamp_us}.csv` + `{timestamp_us}.pcd` |
| Ego_Motion | `enable_ego_motion` | 1000 | `ego_motion.csv`（单文件追加） |
| Obj_List | `enable_obj_list` | 1000 | `{timestamp_us}.csv` |

### 运行时切换

```bash
# 关闭 ADC 录制
ros2 param set /logging_node enable_adc false
ros2 param get /logging_node enable_adc   # 验证
```

### 帧上限策略

达到上限后**停止记录并输出告警日志**，不循环覆盖。

### 异步写入

所有文件写入通过独立线程 + 队列完成（`AsyncWriter` 类），不阻塞 ROS2 主回调循环。

---

## RViz 可视化

```bash
# 使用预配置的 RViz 文件
rviz2 -d config/ft_radar.rviz

# 或手动配置
rviz2  →  Fixed Frame: radar
  →  Add → /visualization/radar/display     (PointCloud2, Color Transformer: RGB8)
  →  Add → /visualization/radar/boxes       (MarkerArray)
  →  Add → /visualization/camera/display    (Image)
  →  Add → /visualization/radar/colorbar    (Image)
```

### 坐标系

| 坐标系 | 父坐标系 | 偏移 | 说明 |
|--------|---------|------|------|
| `map` | — | — | 世界坐标系 |
| `radar` | `map` | z=0.5m | 雷达 |
| `camera` | `radar` | z=1.2m | 相机 |
| `base_link` | — | — | 车辆本体 |

---

## 开发指南

### 代码规范

- 所有节点遵循统一结构：`配置区 → 工具函数 → 节点类 → main()`
- 使用 `★ 用户配置区` 标记集中管理可调参数，配中文说明
- Docstring 包含规格、话题列表、连接关系
- 使用 ROS2 标准 `declare_parameter()` + `get_parameter()` 模式
- 所有节点包含 `destroy_node()` + `finally` 清理逻辑

### 添加新节点

1. 在 `ft_framework/ft_framework/` 下创建 `your_node.py`
2. 参照现有节点模板编写
3. 在 `setup.py` 的 `entry_points` 中添加入口
4. 在 `launch/ft_radar_launch.py` 中添加启动配置

### 接入真实硬件

| 当前模拟实现 | 替换为 |
|-------------|--------|
| `adc_rx._on_timer()` 随机 int16 | v4l2 驱动读取 |
| `camera_rx._on_timer()` 测试图案 | 相机驱动 |
| `vehicle_data_rx._on_timer()` 模拟数据 | CAN/ETH 总线读取 |
| `rsp_mil_python._on_process()` 模拟检测 | 真实 FFT + CFAR 处理 |
| `object_detection_3d._on_det_list()` 欧氏聚类 | TensorRT AI 模型 |

---

## 项目结构

```
Orin-ROS/
├── README.md                               # 本文件
├── .gitignore
│
├── src/
│   ├── ft_radar_msgs/                      # 自定义消息包 (ament_cmake)
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── msg/
│   │       ├── AdcRawData.msg
│   │       ├── DetPoint.msg / DetList.msg
│   │       ├── Object3D.msg / ObjList.msg
│   │       └── EgoMotion.msg
│   │
│   └── ft_framework/                       # 节点功能包 (ament_python)
│       ├── package.xml / setup.py
│       ├── launch/
│       │   └── ft_radar_launch.py
│       └── ft_framework/
│           ├── adc_rx.py                   # Layer 1
│           ├── camera_rx.py
│           ├── vehicle_data_rx.py
│           ├── rsp_mil_python.py           # Layer 2
│           ├── rsp_cuda.py
│           ├── rviz_radar.py              # Layer 3
│           ├── rviz_image.py
│           ├── logging_node.py
│           ├── object_detection_3d.py      # Layer 4
│           └── rviz_ruler.py
│
├── config/
│   ├── ft_radar_params.yaml               # 全局参数配置
│   └── ft_radar.rviz                      # RViz2 预配置
│
├── scripts/
│   ├── build.sh                           # 一键构建
│   └── launch_all.sh                      # 一键启动
│
├── docs/
│   ├── architecture.md                    # 架构设计
│   └── user_guide.md                     # 使用指南
│
└── 参考/                                   # 原始需求文档
    ├── 框架描述.md
    ├── 详细化开发方案.md
    ├── FT_radar_dataset_requirement.md
    └── FT_visualizer/
```

---

## 运行验证

```bash
# 验证节点
ros2 node list
# 应显示全部 10 个节点

# 验证话题
ros2 topic list | grep /ft/   # 旧版不支持此模式
ros2 topic list | grep '^/'   # 新版话题

# 验证连接
ros2 topic info /adc/raw_data --verbose
ros2 topic info /processing/radar/det_list --verbose
ros2 topic info /perception/objects --verbose

# 验证消息内容
ros2 interface show ft_radar_msgs/msg/DetPoint
ros2 interface show ft_radar_msgs/msg/Object3D
ros2 interface show ft_radar_msgs/msg/EgoMotion

# 验证 Logging 运行时开关
ros2 param set /logging_node enable_adc false
ros2 param get /logging_node enable_adc
```

---

## 许可证

本项目采用 **Apache-2.0** 许可证。

**作者**: zhengyuan.liu  
**创建日期**: 2026.6.8  
**最后更新**: 2026.6.9
