# FT Radar-Camera-Vehicle Fusion Framework

基于 ROS2 Humble 的雷达-相机-车辆数据融合感知框架，运行于 **NVIDIA Jetson AGX Orin 64GB** 嵌入式计算平台。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20AGX%20Orin-orange)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
[![License](https://img.shields.io/badge/License-Apache--2.0-yellow)](LICENSE)

---

## 目录

- [系统概述](#系统概述)
- [项目结构](#项目结构)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [框架架构](#框架架构)
- [节点说明](#节点说明)
- [话题列表](#话题列表)
- [RViz 可视化](#rviz-可视化)
- [参数配置](#参数配置)
- [开发指南](#开发指南)
- [参考文档](#参考文档)

---

## 系统概述

本框架实现了一个完整的 **雷达-相机-车辆多传感器融合感知系统**，包含 10 个 ROS2 节点，分布在四个逻辑层级中，通过发布/订阅机制进行通信。

```
[ADC Rx] ──pub──→ [R SP MIL Python] ──pub──→ [3D Object Detection] ──pub──→ [Rviz_radar]
    │                    │                        ↑
    ├──pub──→ [R SP Cuda] ─pub───────────────────┘
    │
    └──pub──→ [Logging] ←──pub── [Camera Rx] ─pub──→ [Rviz_Image]
                        ↑
[Vehicle Data Rx] ──pub──┘
                        ↑
[Rviz_Ruler] ──pub──────→ [Rviz_radar]
```

**核心数据流向：**
1. **三条数据输入流**：雷达 ADC 数据、相机视频数据、车辆 CAN/ETH 数据
2. **双路径雷达信号处理**：Python 版与 CUDA 版并行处理，均执行速度模糊消除
3. **3D 目标检测**：基于 AI 模型（模拟）的 3D 目标检测与分类
4. **统一可视化**：Rviz_radar 汇聚多源数据进行综合显示
5. **全量日志**：Logging 节点收集所有原始数据和中间结果

---

## 项目结构

```
Orin-ROS/
├── README.md                         # 项目主文档（本文件）
├── .gitignore                        # Git 忽略规则
│
├── src/
│   └── ft_framework/                 # ROS2 功能包
│       ├── package.xml               # 包清单（依赖声明）
│       ├── setup.py                  # Python 包安装脚本（10 个节点入口）
│       ├── setup.cfg                 # ROS2 Python 包配置
│       ├── resource/
│       │   └── ft_framework          # ament 资源标记
│       ├── launch/
│       │   └── ft_framework.launch.py  # 全系统启动文件
│       └── ft_framework/
│           ├── __init__.py
│           ├── adc_rx.py             # 节点 1：雷达 ADC 数据接收
│           ├── camera_rx.py          # 节点 2：相机数据接收
│           ├── vehicle_data_rx.py    # 节点 3：车辆数据接收
│           ├── rsp_mil_python.py     # 节点 4：Python 雷达信号处理
│           ├── rsp_cuda.py           # 节点 5：CUDA 雷达信号处理
│           ├── rviz_radar.py         # 节点 6：雷达可视化
│           ├── rviz_image.py         # 节点 7：图像可视化
│           ├── logging_node.py       # 节点 8：数据日志记录
│           ├── object_detection_3d.py  # 节点 9：3D 目标检测
│           └── rviz_ruler.py         # 节点 10：标尺参考
│
├── config/
│   └── ft_framework.rviz             # RViz2 配置文件
│
├── docs/
│   ├── architecture.md               # 架构设计文档
│   └── user_guide.md                 # 用户使用指南
│
├── scripts/
│   ├── build.sh                      # 一键构建脚本
│   └── launch_all.sh                 # 一键启动脚本
│
└── 参考/                             # 参考资料
    ├── 框架描述.md                    # 原始框架描述文档
    └── FT_visualizer/                # 参考可视化工具
        ├── README.md
        ├── rviz_FT_visualizer_xy.py
        ├── rviz_FT_visualizer_xy_simple.py
        └── data/
```

---

## 环境要求

| 项目 | 版本/说明 |
|------|-----------|
| 操作系统 | Ubuntu 20.04 (Jetpack 5.1.2) |
| ROS2 发行版 | Humble |
| Python | 3.10+ |
| 关键依赖 | `rclpy`, `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `tf2_ros`, `cv_bridge` |
| Python 依赖 | `numpy>=1.21.0`, `opencv-python>=4.5.0` |

> **注意**：`cv_bridge`、`numpy` 随 ROS2 完整安装，无需额外 pip 安装。  
> 上电顺序：**Radar（雷达）和 Camera（相机）先上电，然后再启动 Orin 计算平台。**

---

## 快速开始

### 1. 克隆并构建

```bash
# 进入工作空间
cd ~/Orin-ROS

# 构建（首次）
colcon build --packages-select ft_framework --symlink-install

# 或使用脚本
bash scripts/build.sh
```

### 2. 加载环境

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 3. 启动全部节点

```bash
# 方式一：Launch 文件（推荐）
ros2 launch ft_framework ft_framework.launch.py

# 方式二：脚本启动
bash scripts/launch_all.sh
```

### 4. 单独启动某个节点

```bash
ros2 run ft_framework adc_rx                    # 雷达 ADC 接收
ros2 run ft_framework camera_rx                 # 相机接收
ros2 run ft_framework vehicle_data_rx           # 车辆数据接收
ros2 run ft_framework rsp_mil_python            # Python 信号处理
ros2 run ft_framework rsp_cuda                  # CUDA 信号处理
ros2 run ft_framework rviz_radar                # 雷达可视化
ros2 run ft_framework rviz_image                # 图像可视化
ros2 run ft_framework logging_node              # 数据日志
ros2 run ft_framework object_detection_3d       # 3D 目标检测
ros2 run ft_framework rviz_ruler                # 标尺参考
```

### 5. 启动 RViz2 可视化

```bash
# 使用预配置文件
rviz2 -d config/ft_framework.rviz

# 或手动配置
rviz2
# 在 RViz 中设置 Fixed Frame 为 "radar"
# 添加 /ft/radar_display (PointCloud2)、/ft/radar_boxes (MarkerArray)、/ft/video_display (Image)
```

### 6. 查看运行状态

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看话题连接详情
ros2 topic info /ft/adc_data --verbose
ros2 topic info /ft/det_list_py --verbose

# 实时查看话题数据
ros2 topic echo /ft/vehicle_data
```

---

## 框架架构

### 四层逻辑架构

```
┌─────────────────────────────────────────────────────────────┐
│ 第四层：高级感知与辅助层                                       │
│  ┌─────────────────────┐    ┌──────────────┐                │
│  │ 3D Object Detection │    │  Rviz_Ruler  │                │
│  │   (AI 目标检测)      │    │  (标尺参考)   │                │
│  └──────────┬──────────┘    └──────┬───────┘                │
│             │ obj_list              │ ruler                   │
├─────────────┼──────────────────────┼────────────────────────┤
│ 第三层：可视化与日志层                  │                        │
│  ┌──────────┴──────────┐  ┌─────────┴──────┐  ┌──────────┐ │
│  │    Rviz_radar       │  │  Rviz_Image    │  │ Logging  │ │
│  │   (雷达可视化)       │  │  (图像可视化)   │  │ (日志记录) │ │
│  └──────────┬──────────┘  └────────┬───────┘  └────┬─────┘ │
│             ↑ det_list              ↑ video          ↑ all   │
├─────────────┼──────────────────────┼────────────────┼───────┤
│ 第二层：雷达信号处理层                                           │
│  ┌──────────┴──────────┐  ┌─────────┴──────┐                  │
│  │ R SP MIL Python     │  │  R SP Cuda     │                  │
│  │  (Python 实现)       │  │  (CUDA 模拟)    │                  │
│  └──────────┬──────────┘  └────────┬───────┘                  │
│             ↑ adc + vehicle         ↑ adc + vehicle            │
├─────────────┼──────────────────────┼──────────────────────────┤
│ 第一层：数据采集层                                              │
│  ┌──────────┴──────────┐  ┌────────┴───────┐  ┌──────────┐  │
│  │     ADC Rx          │  │   Camera Rx    │  │Vehicle Rx│  │
│  │  (v4l2 雷达采集)     │  │  (v4l2 相机采集) │  │(CAN/ETH) │  │
│  └─────────────────────┘  └────────────────┘  └──────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## 节点说明

| 层级 | 节点名 | 可执行文件 | 功能 | 实现方式 |
|------|--------|-----------|------|----------|
| 1 | ADC Rx | `adc_rx` | 雷达 ADC 数据接收 | 模拟 v4l2，生成随机点云 |
| 1 | Camera Rx | `camera_rx` | 相机视频数据接收 | 模拟 v4l2，生成测试图案 |
| 1 | Vehicle Data Rx | `vehicle_data_rx` | 车辆数据接收 | 模拟 CAN/ETH，生成车速航向 |
| 2 | R SP MIL Python | `rsp_mil_python` | 雷达信号处理 (Python) | SNR 滤波 + 速度补偿 |
| 2 | R SP Cuda | `rsp_cuda` | 雷达信号处理 (CUDA) | 更低 SNR 阈值，更高灵敏度 |
| 3 | Rviz_radar | `rviz_radar` | 雷达数据 RViz 可视化 | 汇聚多源，发布着色点云和标记 |
| 3 | Rviz_Image | `rviz_image` | 图像 RViz 可视化 | 接收视频，叠加帧信息发布 |
| 3 | Logging | `logging_node` | 数据日志记录 | 订阅计数（文件写入待实现） |
| 4 | 3D Object Detection | `object_detection_3d` | 3D 目标检测 | 欧氏聚类模拟 AI 检测 |
| 4 | Rviz_Ruler | `rviz_ruler` | 标尺参考 | 发布坐标尺标记 |

---

## 话题列表

| 话题 | 消息类型 | 发布者 | 订阅者 | 频率 |
|------|----------|--------|--------|------|
| `/ft/adc_data` | `PointCloud2` | `adc_rx` | `rsp_mil_python`, `rsp_cuda`, `logging_node` | 10 Hz |
| `/ft/video_raw` | `Image` | `camera_rx` | `rviz_image`, `logging_node` | 15 Hz |
| `/ft/vehicle_data` | `TwistStamped` | `vehicle_data_rx` | `rsp_mil_python`, `rsp_cuda`, `logging_node` | 20 Hz |
| `/ft/det_list_py` | `PointCloud2` | `rsp_mil_python` | `rviz_radar`, `object_detection_3d`, `logging_node` | 10 Hz |
| `/ft/det_list_cu` | `PointCloud2` | `rsp_cuda` | `rviz_radar`, `logging_node` | 10 Hz |
| `/ft/obj_list` | `MarkerArray` | `object_detection_3d` | `rviz_radar` | — |
| `/ft/ruler` | `MarkerArray` | `rviz_ruler` | `rviz_radar` | 2 Hz |
| `/ft/radar_display` | `PointCloud2` | `rviz_radar` | (RViz) | 10 Hz |
| `/ft/radar_boxes` | `MarkerArray` | `rviz_radar` | (RViz) | 10 Hz |
| `/ft/radar_colorbar` | `Image` | `rviz_radar` | (RViz) | 10 Hz |
| `/ft/video_display` | `Image` | `rviz_image` | (RViz) | — |

### PointCloud2 字段说明

**`/ft/adc_data`** (ADC 原始数据):
| 字段 | 偏移 | 类型 | 说明 |
|------|------|------|------|
| `x` | 0 | float32 | 笛卡尔 X 坐标 (m) |
| `y` | 4 | float32 | 笛卡尔 Y 坐标 (m) |
| `z` | 8 | float32 | 笛卡尔 Z 坐标 (m) |
| `intensity` | 12 | float32 | 信号强度 (dB) |

**`/ft/det_list_py` `/ft/det_list_cu`** (检测目标列表):
| 字段 | 偏移 | 类型 | 说明 |
|------|------|------|------|
| `x` | 0 | float32 | 目标 X 坐标 (m) |
| `y` | 4 | float32 | 目标 Y 坐标 (m) |
| `z` | 8 | float32 | 目标 Z 坐标 (m) |
| `velocity` | 12 | float32 | 径向速度 (m/s) |
| `snr` | 16 | float32 | 信噪比 (dB) |

---

## RViz 可视化

### 快速启动

```bash
# 使用预配置的 RViz 文件
rviz2 -d config/ft_framework.rviz

# 手动配置：
# 1. 打开 rviz2
# 2. Global Options → Fixed Frame: radar
# 3. Add → By topic → /ft/radar_display (PointCloud2)
#    - Color Transformer: RGB8
# 4. Add → By topic → /ft/radar_boxes (MarkerArray)
# 5. Add → By topic → /ft/video_display (Image)
```

### 坐标系

| 坐标系 | 父坐标系 | 说明 |
|--------|---------|------|
| `map` | — | 世界坐标系（根） |
| `radar` | `map` | 雷达坐标系（偏移 z=0.5m） |
| `camera` | `radar` | 相机坐标系（偏移 z=1.2m） |
| `base_link` | — | 车辆本体坐标系 |

---

## 参数配置

所有节点支持通过 ROS2 参数系统进行配置。

### 通过 Launch 文件配置（推荐）

编辑 `src/ft_framework/launch/ft_framework.launch.py` 中的 `parameters` 字典。

### 通过命令行覆盖

```bash
# 调整雷达帧率
ros2 run ft_framework adc_rx --ros-args -p radar_fps:=20.0

# 调整 SNR 阈值
ros2 run ft_framework rsp_mil_python --ros-args -p snr_threshold:=15.0

# 调整标尺参数
ros2 run ft_framework rviz_ruler --ros-args \
  -p ruler_axis:=y \
  -p ruler_offset:=30.0 \
  -p ruler_length:=200.0
```

### 主要参数速查

| 节点 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| `adc_rx` | `radar_fps` | 10.0 | 雷达帧率 (Hz) |
| `adc_rx` | `num_targets` | 50 | 每帧目标数 |
| `adc_rx` | `range_max` | 300.0 | 最大探测距离 (m) |
| `camera_rx` | `camera_fps` | 15.0 | 相机帧率 (Hz) |
| `vehicle_data_rx` | `sim_speed_mean` | 15.0 | 模拟车速 (m/s) |
| `rsp_mil_python` | `snr_threshold` | 10.0 | SNR 阈值 (dB) |
| `rsp_cuda` | `snr_threshold` | 8.0 | SNR 阈值 (CUDA版) |
| `object_detection_3d` | `cluster_distance` | 5.0 | 聚类距离 (m) |
| `object_detection_3d` | `min_cluster_size` | 3 | 最小簇大小 |
| `rviz_ruler` | `ruler_axis` | x | 标尺方向 (x/y) |
| `rviz_ruler` | `ruler_length` | 300.0 | 标尺长度 (m) |

---

## 开发指南

### 代码规范

- 所有节点遵循统一的代码结构：`配置区 → 工具函数 → 节点类 → main()`
- 使用 `★ 用户配置区` 标记集中管理可调参数
- 每个节点包含完整的中文 docstring 说明话题和连接关系
- 遵循 ROS2 src guideline 开发规范
- 作者：zhengyuan.liu

### 添加新节点

1. 在 `ft_framework/` 目录下创建 `your_node.py`
2. 按照现有节点模板编写代码（参考 `adc_rx.py` 作为最简示例）
3. 在 `setup.py` 的 `entry_points` 中添加入口：
   ```python
   'your_node = ft_framework.your_node:main',
   ```
4. 在 `launch/ft_framework.launch.py` 中添加节点启动配置

### 实现具体算法

当前算法节点使用模拟/占位实现。要实现真实算法，修改对应节点中的处理方法：

- **ADC 数据采集** → 修改 `adc_rx.py` 的 `_on_timer()` 接入真实 v4l2 驱动
- **雷达信号处理** → 修改 `rsp_mil_python.py` / `rsp_cuda.py` 的 `_on_process()` 
- **3D 目标检测** → 修改 `object_detection_3d.py` 的 `_on_det_list()` 接入 AI 模型
- **数据日志** → 修改 `logging_node.py` 添加文件写入逻辑

---

## 参考文档

- [架构设计文档](docs/architecture.md) — 详细的系统架构设计
- [用户使用指南](docs/user_guide.md) — 节点参数详解和调试技巧
- [框架描述](参考/框架描述.md) — 原始框架需求文档
- [FT Visualizer 参考](参考/FT_visualizer/README.md) — 参考可视化工具文档
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [NVIDIA Jetson Orin 文档](https://developer.nvidia.com/embedded/jetson-orin)

---

## 许可证

本项目采用 [Apache-2.0](LICENSE) 许可证。

---

**作者：** zhengyuan.liu  
**创建日期：** 2026.6.8  
**最后更新：** 2026.6.8
