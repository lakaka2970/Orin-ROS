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

## 硬件设备

### 设备规格

| 设备 | 型号 | 接口 | 驱动 | 分辨率 | 帧率 | 格式 | 数据量/帧 |
|------|------|------|------|--------|:----:|------|:--------:|
| **雷达** | Infineon CTRX8188F (16T16R) | GMSL/FPD-Link (MAX929x) | `tegra-video` | ctrx0: 8192×1024<br>ctrx1: 8192×1024 | 15.38 Hz | RG12 (12-bit) | 32 MiB (ctrx0+ctrx1 合并) |
| **摄像头** | Rmoncam A2 1080P | USB 3.0 (UVC) | `uvcvideo` | 1920×1080 | 30 Hz | MJPEG → BGR8 | ~6 MB (解压后) |

### udev 持久化设备命名

内核分配的 `/dev/videoN` 编号随驱动加载顺序变化。udev 规则创建固定符号链接：

| 符号链接 | → 设备 | 匹配条件 |
|----------|--------|----------|
| `/dev/radar_ctrx0` | video0 | `ID_PATH=platform-tegra-capture-vi` + `ID_V4L_PRODUCT=vi-output, Radar 2-0030` |
| `/dev/radar_ctrx1` | video1 | `ID_PATH=platform-tegra-capture-vi` + `ID_V4L_PRODUCT=vi-output, Radar 2-0031` |
| `/dev/camera_capture` | video2 | USB VID/PID `0801:0101` + v4l2 `index=0` |

```bash
# 安装 udev 规则 (首次部署)
sudo cp scripts/99-ft-sensors.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=video4linux
ls -la /dev/camera_capture /dev/radar_ctrx0 /dev/radar_ctrx1
```

### 启动雷达硬件

已集成到 `scripts/build.sh` 和 `scripts/start.sh`，无需手动执行:

```bash
# 硬件初始化已集成至构建脚本 (默认自动执行)
bash scripts/build.sh                    # 含 sudo python3 init_jetson.py
bash scripts/build.sh --skip-init-hw     # 跳过硬件初始化 (已初始化时使用)

# 雷达数据采集已集成至启动脚本 (开发管线)
bash scripts/start.sh --capture-only --rsps    # 采集 + RSPS 离线点云可视化
```

**原始手动流程** (仍可用于独立调试):

```bash
# 1. 启动雷达解串板 (加载 GMSL 驱动 + 配置 MAX929x)
cd ~/Desktop/Orin-ROS/src/integration-carkit88c0-gmsl
sudo python3 init_jetson.py
cd src/demo_app/boards/carkit_88c0/example/bin
sudo ./carkit88c0_gmsl_example

# 2. 采集雷达数据 (生成 output/ctrx0_raw.bin 和 output/ctrx1_raw.bin)
cd ~/Desktop/Orin-ROS/src/integration-carkit88c0-gmsl/scripts
bash capture_video0_2048x1024.sh
```

> **上电顺序**: 雷达和摄像头先上电 → 再启动 Orin 计算平台。上述命令在系统重启后需重新执行。

### YAML 设备路径配置

```yaml
# config/ft_radar_params.yaml
adc_rx:
  device_path: "/dev/radar_ctrx0"    # GMSL 雷达 ctrx0 半集

camera_rx:
  device_path: "/dev/camera_capture"  # USB 摄像头 udev 符号链接
  image_width: 1920
  image_height: 1080
  pixel_format: "MJPG"               # MJPEG@30fps, 或 YUYV@5fps
  fps: 30
```

---

## 目录

- [硬件设备](#硬件设备)

- [硬件设备](#硬件设备)
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

# 0a. (仅首次) 安装系统依赖 + ROS2
bash scripts/install_deps.sh --with-ros2

# 0b. (推荐) 免 source: 以后每次开终端自动加载环境
echo "source ~/Orin-ROS/scripts/env.sh" >> ~/.bashrc
source ~/.bashrc

# 1. 构建 (含 Jetson 硬件初始化: Pinmux + GMSL 驱动 + 雷达驱动)
bash scripts/build.sh
#    可选: bash scripts/build.sh --skip-init-hw   # 跳过硬件初始化

# 2. 一键启动 (生产管线: adc_rx → ROS2 topics → RSP → det_list)
bash scripts/start.sh

# 2b. (可选) 开发管线: 采集雷达原始数据 + RSPS 离线点云可视化
bash scripts/start.sh --capture-only --rsps
```

> 完成步骤 0b 后，每次开新终端自动完成环境加载，不再需要手动 `source`。

### 安装说明

| 场景 | 命令 |
|------|------|
| **全新系统** (含 ROS2) | `bash scripts/install_deps.sh --with-ros2` |
| **已有 ROS2** | 直接构建即可，系统依赖已满足 |
| **仅检查不安装** | `bash scripts/install_deps.sh --dry-run` |

> **RMW 传输**: 默认使用 **FastDDS**（ROS2 Foxy 内置），其内置 **SHM (共享内存) 传输**，同机节点间 32 MiB ADC 消息不经网络栈、零 UDP 分片。无需额外安装 CycloneDDS。

### 启动选项

| 参数 | 说明 |
|------|------|
| `--analog` | 使用模拟 ADC 数据源 (噪声池/.bin), 默认 real 硬件 |
| `--no-adc` | **不启动 adc_rx 节点**, logging_node 自动关闭 adc.bin 录制 |
| `--py-rx` | 使用 Python 版 rx 节点 (默认 C++) |
| `--rviz` | 同时启动 RViz2 |

```bash
# ── 生产管线 (ROS2 框架) ──
bash scripts/start.sh                   # 默认 cuda + C++ rx
bash scripts/start.sh python            # Python RSP + C++ rx
bash scripts/start.sh cuda --rviz       # CUDA + RViz
bash scripts/start.sh both_compare      # 双路对比
bash scripts/start.sh python --py-rx    # Python RSP + Python rx (回退)
bash scripts/start.sh --no-adc          # 不启动 adc_rx (仅 RSP + 其他节点)
bash scripts/start.sh --no-adc --rviz   # 不启动 adc_rx + RViz

# ── 开发管线 (离线调试, 与 ROS 框架互斥 — 共享 /dev/video0) ──
bash scripts/start.sh --capture-only           # 仅采集雷达原始数据 (v4l2-ctl)
bash scripts/start.sh --capture-only --rsps    # 采集 + RSPS 离线点云可视化
bash scripts/start.sh --capture --rsps         # 采集 + RSPS → 自动启动 ROS 框架
```

### 验证运行状态

```bash
source scripts/env.sh

# 查看话题频率
ros2 topic hz /adc/raw_data          # 硬件实际频率 (轮询模式)
ros2 topic hz /camera/image_raw      # 硬件实际频率 (轮询模式)
ros2 topic hz /vehicle/ego_motion    # ~50 Hz (发布频率, 可 ros2 param set 调整)

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

| 实现 | rx_impl 参数 | 采集/发布方式 | 说明 |
|------|:---:|:---:|------|
| **C++** (默认) | `rx_impl:=cpp` | 轮询模式 (adc/camera) + 混合模式 (vehicle) | 阻塞硬件读取, 即时 ROS 时间戳, FastDDS SHM 传输 |
| Python | `rx_impl:=python` | 同 C++ 架构 | 轮询/混合模式, 用于对比验证 |

### 常见组合

```bash
# C++ rx + Python RSP (推荐调试)
bash scripts/start.sh python

# C++ rx + CUDA RSP (推荐生产)
bash scripts/start.sh cuda

# Python rx + Python RSP (回退验证)
bash scripts/start.sh python --py-rx

# 不启动 adc_rx (仅 RSP + camera + vehicle + logging, 无 ADC 数据)
bash scripts/start.sh --no-adc

# 不启动 adc_rx + CUDA RSP
bash scripts/start.sh cuda --no-adc

# 仅录制 ADC + 自车数据
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda \
  log_image:=false log_det_list:=false log_obj_list:=false
```

---

## 系统架构

### 数据流

```
[ADC Rx C++] ──→ /adc/raw_data (32MiB/帧, 轮询模式, FastDDS SHM 共享内存传输)
                    ├──→ [RSP Python / CUDA] ──→ /processing/radar/det_list
                    │                            └──→ /processing/radar/rn_nci_data
                    └──→ [Logging] ──→ output/ft_dataset/

[Camera Rx C++] ──→ /camera/image_raw (轮询模式-硬件实际频率) ──→ [Rviz_Image] [Logging]

[Vehicle Rx C++] ──→ /vehicle/ego_motion (混合模式-CAN读取线程+buffer+定时发布50Hz) ──→ [RSP] [Logging]
```

### 四层结构

| 层级 | 节点 | 实现 | 职责 |
|:----:|------|:----:|------|
| 第一层 | ADC Rx, Camera Rx, Vehicle Data Rx | **C++** | 数据采集, 零拷贝发布 |
| 第二层 | RSP MIL Python, RSP Cuda | Python | 雷达信号处理 |
| 第三层 | Rviz_radar, Rviz_Image, Logging | Python | 可视化, 日志记录 |
| 第四层 | Object Detection 3D, Rviz_Ruler | Python | 目标检测, 坐标标尺 |

> 第一层 rx 节点使用 C++ 实现（`ft_rx_cpp` 包），利用零拷贝、噪声池预生成、FastDDS SHM 传输保证 15 Hz 稳定发布。第二至四层保留 Python 便于算法迭代。

---

## 节点说明

### 第一层：数据采集 (C++)

| 节点 | 采集/发布方式 | 输出话题 | 数据量 | 说明 |
|------|:----:|------|------|------|
| **ADC Rx** | 轮询模式 (硬件实际频率) | `/adc/raw_data` | 32 MiB/帧 | 阻塞 V4L2 DQBUF, 即时 ROS 时间戳 |
| **Camera Rx** | 轮询模式 (硬件实际频率) | `/camera/image_raw` | ~6 MB/帧 | 阻塞 camera.read(), Rmoncam A2 1080P MJPEG→BGR8 |
| **Vehicle Data Rx** | 混合模式 (发布 50 Hz 可配置) | `/vehicle/ego_motion` | < 1 KB/帧 | CAN读取线程 → buffer(mutex) → 定时发布, 超时20ms检测 |

### C++ Rx 优化要点

| 优化 | 效果 |
|------|------|
| 噪声池预生成 (4x 帧大小) | 消除每帧 MT19937 开销 |
| 单次 memcpy 入消息 | 替代 Python tolist() 的 16.7M 对象分配 |
| `uint8[]` 消息类型 | ROS2 C 序列化走 memcpy 快速路径 |
| FastDDS SHM 传输 | 32 MiB 消息走共享内存, 零网络栈开销, 零拷贝 |
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
| **Object Detection 3D** | CUDA 优先订阅: det_list_cuda (主) + det_list (回退) → 欧氏聚类 → 14 字段 3D 目标 |
| **Rviz_Ruler** | 坐标尺参考标记 |

---

## 话题总表

| 话题 | 消息类型 | Pub | Sub |
|------|----------|:---:|:---:|
| `/adc/raw_data` | `AdcRawData` | adc_rx (C++) | rsp_*, logging_node |
| `/camera/image_raw` | `sensor_msgs/Image` | camera_rx (C++) | rviz_image, logging_node |
| `/vehicle/ego_motion` | `EgoMotion` | vehicle_data_rx (C++) | rsp_*, logging_node |
| `/processing/radar/det_list` | `DetList` | rsp_mil_python / rsp_cuda | rviz_radar, obj_detection_3d, logging_node, monitor_rsp |
| `/processing/radar/det_list_cuda` | `DetList` | rsp_cuda (双路模式) | rviz_radar, logging_node, monitor_rsp |
| `/processing/radar/rn_nci_data` | `RnNciData` | rsp_mil_python / rsp_cuda | logging_node |
| `/processing/radar/rn_nci_data_cuda` | `RnNciData` | rsp_cuda (双路模式) | logging_node |
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
| `DetPoint` | 41 | 雷达检测目标点 (对齐 spec §5.3 / §6.3) |
| `DetList` | header + frame_id + DetPoint[] | 检测列表 |
| `Object3D` | 14 | 3D 目标 |
| `ObjList` | header + Object3D[] | 目标列表 |
| `EgoMotion` | 7 + header + is_default | 自车运动数据 |
| `RnNciData` | 20 + header | RD Cell List + Rx NCI (对齐 spec §7 + §8) |

---

## 时间戳机制

- **时钟源**: ROS 系统时间 `get_clock().now()`, 微秒精度
- **注入位置**: rx 节点采集数据第一时间注入 `header.stamp` (轮询线程/定时器回调中)
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
| camera_rx | `image_width` | 1920 | 图像宽度 (px) |
| camera_rx | `image_height` | 1080 | 图像高度 (px) |
| camera_rx | `device_path` | /dev/camera_capture | V4L2 设备路径 (udev 符号链接) |
| camera_rx | `pixel_format` | MJPG | MJPEG@30fps 或 YUYV@5fps |
| vehicle_data_rx | `fps` | 50.0 | 发布频率 (定时器控制) |
| vehicle_data_rx | `timeout_cycles` | 1 | CAN 超时周期数 (1周期=20ms @50Hz) |
| vehicle_data_rx | `can_interface` | can0 | CAN 接口名 |
| rsp_mil_python | `snr_threshold` | 10.0 | SNR 阈值 (dB) |
| rsp_cuda | `snr_threshold` | 8.0 | SNR 阈值 (dB) |
| logging_node | `output_dir` | output/ft_dataset | 日志输出目录 |

---

## Logging 系统

### 通道

| 通道 | 开关参数 | 循环覆盖上限 | 输出 |
|:----:|----------|:--------:|------|
| ADC | `enable_adc` | 100 | `adc_data/{ts}.bin` |
| Image | `enable_image` | 1000 | `camera_front_center/{ts}.jpg` |
| Det_List | `enable_det_list` | 1000 | `pc_pcd_radar_front_center/{ts}.pcd` + `pc_csv_radar_front_center/{ts}.csv` |
| RnNci | `enable_rn_nci` | 1000 | `rdCell_csv_radar_front_center/{ts}.csv` + `rxNci_bin_radar_front_center/{ts}.bin` |
| Ego_Motion | `enable_ego_motion` | 1000 | `ego_motion.csv` (保留最新 N 行) |
| Obj_List | `enable_obj_list` | 1000 | `obj_csv_radar/{ts}.csv` |

> 达到上限后**循环覆盖**: 删除最旧文件, 保留最新 N 帧。ego_motion.csv 保留最新 N 行后重写。
> 所有数据格式对齐 `FT_FVR60_XD_radar_dataset_requirement.md` (spec §3~§9)。

### 数据格式

| 输出 | Spec | 格式 |
|------|:----:|------|
| `pc_pcd_radar_front_center/{ts}.pcd` | §5.3 | PCD v0.7 ASCII, 19 字段 (x/y/z/range/speed/azimuth_ang/ele_ang/snr_db/rcs_db/power_db/obj_same_rv/rd_cell_idx/range_idx/doppler_idx/azimuth_idx/elevation_idx/peak_val/sin_azim_snr_lin/sin_elev_snr_lin) |
| `pc_csv_radar_front_center/{ts}.csv` | §6.3 | CSV, 22 列 (u32TimeStamp + u16FrameID + u16DetObjNum + 19 字段) |
| `rdCell_csv_radar_front_center/{ts}.csv` | §7.3 | CSV, RD Cell List (13 字段, f32PowRbNci_Q7dB[3]→-0/-1/-2, sVch[256]→-0_r/-0_im...-255_r/-255_im) |
| `rxNci_bin_radar_front_center/{ts}.bin` | §8.3 | 原始 float32 二维数组 (Rx NCI 全谱) |
| `adc_data/{ts}.bin` | — | 20 字节 header + int16[] payload |

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

| 硬件 | 状态 | 接入路径 | 说明 |
|------|:----:|------|------|
| **雷达 ADC** | ✅ 已接入 | `/dev/radar_ctrx0` (V4L2 mmap, 阻塞 DQBUF) | 轮询模式, ctrx0+ctrx1 双文件合并, 硬件实际频率, 32 MiB/帧 |
| **USB 摄像头** | ✅ 已接入 | `/dev/camera_capture` (V4L2/OpenCV, 阻塞 read) | 轮询模式, Rmoncam A2 1080P, MJPEG 1920×1080, 硬件实际频率 → BGR8 |
| **车辆数据** | ✅ 架构就绪 | CAN/ETH 总线 (待接入) | 混合模式: CAN读取线程+buffer+定时发布50Hz, 超时20ms检测, 当前默认值 |
| **RSP 信号处理** | ✅ 完整 | Python (NumPy) + CUDA (PyTorch) | 完整流水线: DC去除+干扰抑制+Range FFT+Doppler FFT+DDMA+CFAR+DOA角度估计 |
| **3D 目标检测** | ⚠️ 部分 | — (待部署 AI 模型) | 欧氏聚类可用, 待 TensorRT 推理 |

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
│   └── fastdds.xml                   # FastDDS SHM 配置 (128MB 段, 默认生效)
├── scripts/
│   ├── env.sh                         # 环境加载 (默认 FastDDS + SHM 配置)
│   ├── install_deps.sh                # 依赖安装
│   ├── build.sh                       # 一键构建 (3 个包 + Jetson 硬件初始化)
│   ├── start.sh                       # 一键启动 (生产管线 + 开发管线)
│   ├── launch_all.sh                  # 兼容入口 → start.sh
│   └── 99-ft-sensors.rules           # udev 设备持久化命名规则
├── src/
│   └── integration-carkit88c0-gmsl/ # GMSL 雷达硬件集成 (驱动 + 采集脚本)
├── docs/                           # 架构与优化文档
└── output/ft_dataset/              # 日志输出
```

---

## 许可证

Apache-2.0

**作者**: zhengyuan.liu
**创建**: 2026.6.8
**更新**: 2026.7.2
