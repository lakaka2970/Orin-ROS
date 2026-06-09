# FT Radar Fusion Framework — ROS2 Humble

基于 ROS2 Humble 的雷达-相机-车辆多传感器融合感知框架，运行于 **NVIDIA Jetson AGX Orin**。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)
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

# 1. 一键构建（自动加载 ROS2 环境，按序编译消息包和节点包）
bash scripts/build.sh

# 2. 加载工作空间
source install/setup.bash

# 3. 启动全部节点（默认 CUDA 模式）
ros2 launch ft_framework ft_radar_launch.py
```

### 构建选项

```bash
bash scripts/build.sh                   # 增量构建
bash scripts/build.sh --clean           # 全量重构建
bash scripts/build.sh --launch          # 构建后直接启动
bash scripts/build.sh --clean --launch  # 清理 → 构建 → 启动
```

### 验证运行状态

```bash
# 查看所有节点（应显示 8-10 个）
ros2 node list

# 查看所有话题
ros2 topic list

# 查看话题连接关系
ros2 topic info /adc/raw_data
ros2 topic info /processing/radar/det_list
ros2 topic info /perception/objects

# 查看消息定义
ros2 interface show ft_radar_msgs/msg/DetPoint
ros2 interface show ft_radar_msgs/msg/Object3D
```

---

## 启动模式

框架支持 **4 种 RSP 启动模式**：

### 模式

| 模式 | 命令 | Python RSP | CUDA RSP | 适用场景 |
|------|------|:----------:|:--------:|----------|
| `cuda` (默认) | `rsp_mode:=cuda` | ✗ | ✓ | 生产部署 |
| `python` | `rsp_mode:=python` | ✓ | ✗ | 算法调试 |
| `both` | `rsp_mode:=both` | ✓ | ✓ | A/B 对比验证 |
| `both_compare` | `rsp_mode:=both_compare` | ✓ | ✓ | 差异分析 |

### 常见组合

```bash
# 默认 CUDA 模式
ros2 launch ft_framework ft_radar_launch.py

# Python 调试模式
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=python

# 双路并行（Python + CUDA 各自输出到不同话题）
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=both

# 仅录制 ADC 和自车数据，关闭图像和检测
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda \
  log_image:=false log_det_list:=false log_obj_list:=false
```

---

## 系统架构

```
[ADC Rx] ──→ /adc/raw_data ──→ [RSP Python / CUDA] ──→ /processing/radar/det_list
                                                              │
                                              ┌───────────────┤
                                              ▼               ▼
                                   [Obj Detection 3D]    [Rviz_radar]
                                        │                    │
                                   /perception/objects        │
                                        │                    │
                                        ▼               ┌────┘
[Camera Rx] ──→ /camera/image_raw ──→ [Rviz_Image]     │
                                                         │
[Vehicle Rx] ──→ /vehicle/ego_motion ──→ [RSP, Logging] │
                                                         │
[Rviz_Ruler] ──→ /visualization/ruler ──────────────────→┘

[Logging] ←── 全部 5 路数据 → 异步写入磁盘
```

### 四层逻辑结构

| 层级 | 节点 | 职责 |
|:----:|------|------|
| 第一层 | ADC Rx, Camera Rx, Vehicle Data Rx | 数据采集，注入微秒时间戳 |
| 第二层 | RSP MIL Python, RSP Cuda | 雷达信号处理，SNR 滤波 + 速度补偿 |
| 第三层 | Rviz_radar, Rviz_Image, Logging | 可视化显示，数据日志记录 |
| 第四层 | Object Detection 3D, Rviz_Ruler | 3D 目标检测，坐标标尺 |

---

## 节点说明

### 第一层：数据采集

| 节点 | 频率 | 输出话题 | 说明 |
|------|:----:|------|------|
| **ADC Rx** | 15 Hz | `/adc/raw_data` | ADC 原始数据，32 MB/帧 |
| **Camera Rx** | 30 Hz | `/camera/image_raw` | 相机图像（格式待确认，占位实现） |
| **Vehicle Data Rx** | 50 Hz | `/vehicle/ego_motion` | 车速/偏航角/转向/加速度/挡位，超时自动回退默认值 |

### 第二层：信号处理

| 节点 | 输入 | 输出 | 说明 |
|------|------|------|------|
| **RSP MIL Python** | ADC + EgoMotion | `/processing/radar/det_list` | SNR≥10dB，~30 检测点 |
| **RSP Cuda** | ADC + EgoMotion | `/processing/radar/det_list` 或 `*_cuda` | SNR≥8dB，~45 检测点 |

### 第三层：可视化与日志

| 节点 | 功能 |
|------|------|
| **Rviz_radar** | 合并 DetList/ObjList/Ruler → PointCloud2 (按高度着色) + MarkerArray |
| **Rviz_Image** | 图像叠加帧号和时间戳信息 |
| **Logging** | 5 通道独立开关，6 种输出格式，异步写盘，帧数上限 |

### 第四层：高级感知

| 节点 | 功能 |
|------|------|
| **Object Detection 3D** | 欧氏聚类 (模拟 AI) → 14 字段 3D 目标 |
| **Rviz_Ruler** | 发布坐标尺参考标记 |

---

## 话题总表

| 话题 | 消息类型 | Pub | Sub |
|------|----------|:---:|:---:|
| `/adc/raw_data` | `AdcRawData` | adc_rx | rsp_*, logging_node |
| `/camera/image_raw` | `sensor_msgs/Image` | camera_rx | rviz_image, logging_node |
| `/vehicle/ego_motion` | `EgoMotion` | vehicle_data_rx | rsp_*, logging_node |
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
| `AdcRawData` | 4 + header | ADC 原始 int16 数组 |
| `DetPoint` | 14 | 雷达检测目标点 |
| `DetList` | header + DetPoint[] | 检测列表 |
| `Object3D` | 14 | 3D 目标 (跟踪 ID / 包围盒 / 速度 / 运动状态) |
| `ObjList` | header + Object3D[] | 目标列表 |
| `EgoMotion` | 7 + header + is_default | 自车运动数据 |

### 关键字段说明

**DetPoint**: 含雷达系原视测量 (range, azimuth, elevation) 和车辆系坐标 (x, y, z)，共 14 字段与数据规格文档完全对齐。

**Object3D**: 含跟踪信息 (object_id, tracked_times)、包围盒 (x/y/z/l/w/h/yaw)、对地速度 (vx/vy/vz_absolute)、运动分类 (moving_state)。

---

## 时间戳机制

- **时钟源**: `time.monotonic_ns()`，精度纳秒 → 微秒
- **注入位置**: Rx 节点采集数据第一时间注入 `header.stamp`
- **透传规则**: 下游节点**禁止覆盖**时间戳，必须原样传递
- **文件命名**: Logging 节点从 `header.stamp` 提取微秒整数作为文件名

---

## 参数配置

所有节点参数统一管理：[`config/ft_radar_params.yaml`](config/ft_radar_params.yaml)

### 配置优先级

```
config/ft_radar_params.yaml        ← 默认值
  ↓
ros2 launch ... params:=xxx        ← Launch 覆盖
  ↓
ros2 param set /node param value   ← 运行时动态修改
```

### 常用参数

| 节点 | 参数 | 默认值 | 说明 |
|------|------|:------:|------|
| adc_rx | `fps` | 15 | ADC 帧率 |
| camera_rx | `fps` | 30 | 相机帧率 |
| vehicle_data_rx | `fps` | 50 | 车辆数据帧率 |
| vehicle_data_rx | `timeout_cycles` | 3 | 超时回退默认值 |
| rsp_mil_python | `snr_threshold` | 10.0 | SNR 阈值 (dB) |
| rsp_cuda | `snr_threshold` | 8.0 | SNR 阈值 (dB) |
| object_detection_3d | `cluster_distance` | 5.0 | 聚类距离 (m) |
| logging_node | `output_dir` | ~/ft_radar_dataset | 日志输出目录 |

---

## Logging 系统

### 5 个独立通道

| 通道 | 开关参数 | 最大帧数 | 输出文件 |
|:----:|----------|:--------:|----------|
| ADC | `enable_adc` | 100 | `adc.bin` |
| Image | `enable_image` | 1000 | `{timestamp_us}.jpg` |
| Det_List | `enable_det_list` | 1000 | `{timestamp_us}.csv` + `{timestamp_us}.pcd` |
| Ego_Motion | `enable_ego_motion` | 1000 | `ego_motion.csv` (追加) |
| Obj_List | `enable_obj_list` | 1000 | `{timestamp_us}.csv` |

### 运行时控制

```bash
# 查看当前开关状态
ros2 param get /logging_node enable_adc

# 动态关闭 ADC 录制
ros2 param set /logging_node enable_adc false

# 修改输出目录
ros2 param set /logging_node output_dir /home/lzy/my_dataset
```

### 工作方式

- **异步写入**: 独立线程 + 队列，不阻塞 ROS2 节点主循环
- **帧数上限**: 达到上限自动停止并告警，**不循环覆盖**
- **自动创建目录**: 首次运行自动创建 `output_dir` 及其子目录
- **ego_motion.csv**: 单文件追加写入，带 CSV 表头
- **逐帧文件**: 图片/DetList/ObjList 按 `{timestamp_us}` 命名

### 启动 Logging

Logging 节点随 `ros2 launch ft_framework ft_radar_launch.py` 自动启动（1 秒延迟等待上游订阅就绪）。

如果节点列表中缺少 `/logging_node`：
```bash
# 1. 检查启动日志中的错误信息
# 2. 确认输出目录可写
ls -la ~/ft_radar_dataset

# 3. 手动启动（调试用）
ros2 run ft_framework logging_node --ros-args -p output_dir:=~/ft_radar_dataset
```

---

## RViz 可视化

```bash
# 使用预配置布局
rviz2 -d config/ft_radar.rviz

# 或手动添加显示面板:
#   Fixed Frame: radar
#   → /visualization/radar/display    (PointCloud2)
#   → /visualization/radar/boxes      (MarkerArray)
#   → /visualization/camera/display   (Image)
```

### TF 坐标系

| 坐标系 | 父系 | 偏移 | 说明 |
|--------|------|------|------|
| `map` | — | — | 世界原点 |
| `radar` | map | z=0.5m | 雷达安装位 |
| `camera` | radar | z=1.2m | 相机安装位 |
| `base_link` | — | — | 车辆本体 |

---

## 开发指南

### 代码约定

- 每个节点文件按 `配置区 → 工具函数 → 节点类 → main()` 组织
- `★ 用户配置区` 集中可调参数，附带中文说明
- 所有节点包含 `destroy_node()` 和 `finally` 清理逻辑
- 日志使用中文，方便国内团队阅读

### 添加新节点

1. 在 `ft_framework/ft_framework/` 创建 `your_node.py`
2. 参照现有节点的 `main()` 模板
3. 在 `setup.py` 的 `entry_points` 注册入口
4. 在 `launch/ft_radar_launch.py` 添加启动配置

### 接入真实硬件

| 当前模拟 | 替换方案 |
|---------|---------|
| `np.random.randint` 模拟 ADC | v4l2 驱动 + DMA 读取 |
| OpenCV 测试图案 | GMSL/FPD-Link 相机驱动 |
| 正态分布模拟车速 | CAN/ETH 总线解析 |
| 随机点 + SNR 过滤 | 真实 Range-FFT + CFAR |
| 欧氏聚类 | TensorRT AI 推理模型 |

---

## 项目结构

```
Orin-ROS/
├── README.md
├── .gitignore
├── src/
│   ├── ft_radar_msgs/              # 自定义消息 (ament_cmake)
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── msg/                    # 6 个 .msg 文件
│   └── ft_framework/               # 功能节点 (ament_python)
│       ├── package.xml / setup.py
│       ├── launch/
│       │   └── ft_radar_launch.py
│       └── ft_framework/           # 10 个节点 + common.py
├── config/
│   ├── ft_radar_params.yaml
│   └── ft_radar.rviz
├── scripts/
│   └── build.sh
├── data/                           # 模拟测试数据
├── docs/                           # 架构与使用文档
└── 参考/                           # 原始需求文档
```

---

## 许可证

Apache-2.0

**作者**: zhengyuan.liu  
**创建**: 2026.6.8  
**更新**: 2026.6.9
