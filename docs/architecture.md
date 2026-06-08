# FT Framework — 架构设计文档

> 基于 ROS2 Humble 的雷达-相机-车辆数据融合感知框架详细架构设计

---

## 一、设计目标

1. **模块化**：每个传感器和处理模块作为独立的 ROS2 节点，便于开发、测试和替换
2. **可扩展**：支持并行处理路径（Python/CUDA），可方便添加新的处理节点
3. **标准化**：使用 ROS2 标准消息类型，遵循 ROS2 src guideline 开发规范
4. **可视化**：与 RViz2 深度集成，提供完整的实时可视化能力
5. **日志完备**：集中式日志节点收集所有数据流，支持离线分析

---

## 二、节点架构

### 2.1 节点分层

系统采用四层架构设计，层间通过话题通信解耦：

```
┌──────────────────────────────────────────────────────────┐
│                    第四层：高级感知与辅助                    │
│  object_detection_3d (AI 检测)    rviz_ruler (标尺参考)     │
│         ↓ obj_list                       ↓ ruler           │
├──────────────────────────────────────────────────────────┤
│                    第三层：可视化与日志                      │
│  rviz_radar (雷达可视化)  rviz_image (图像)  logging_node    │
│    ↑ det_list       ↑ video_raw           ↑ 全部数据        │
├──────────────────────────────────────────────────────────┤
│                    第二层：雷达信号处理                      │
│  rsp_mil_python (Python实现)    rsp_cuda (CUDA加速模拟)      │
│    ↑ adc_data + vehicle           ↑ adc_data + vehicle     │
├──────────────────────────────────────────────────────────┤
│                     第一层：数据采集                        │
│  adc_rx (雷达)    camera_rx (相机)    vehicle_data_rx (车辆) │
└──────────────────────────────────────────────────────────┘
```

### 2.2 节点启动顺序

使用 ROS2 Launch 的 `TimerAction` 实现分层启动：

1. **t=0s**：第一层（数据采集层）立即启动
2. **t=0.5s**：第二层（信号处理层）启动
3. **t=1.0s**：第三层（可视化与日志层）启动
4. **t=1.5s**：第四层（高级感知层）启动

---

## 三、数据流设计

### 3.1 主数据流

```
ADC Rx → (adc_data) → R SP MIL Python → (det_list_py) → 3D Object Det → (obj_list) → Rviz_radar
                    ↘                              ↘
                      R SP Cuda ─────────────────→ Rviz_radar
                    ↘                              ↘
                      Logging ←──────────────────────┘

Camera Rx → (video_raw) → Rviz_Image
                        → Logging

Vehicle Rx → (vehicle_data) → R SP MIL Python
                            → R SP Cuda
                            → Logging

Rviz_Ruler → (ruler) → Rviz_radar
```

### 3.2 消息定义

所有节点间通信使用 ROS2 标准消息类型，无需自定义 .msg 编译：

| 数据类型 | ROS2 消息类型 | 字段说明 |
|----------|-------------|----------|
| ADC 原始数据 | `sensor_msgs/PointCloud2` | x, y, z, intensity (float32) |
| 视频帧 | `sensor_msgs/Image` | bgr8 编码 |
| 车辆动态 | `geometry_msgs/TwistStamped` | linear.x/y, angular.z |
| 检测列表 | `sensor_msgs/PointCloud2` | x, y, z, velocity, snr (float32) |
| 3D 目标 | `visualization_msgs/MarkerArray` | CUBE + TEXT_VIEW_FACING |
| 标尺 | `visualization_msgs/MarkerArray` | TEXT_VIEW_FACING + LINE_STRIP |

---

## 四、坐标系设计

```
map (世界坐标系)
│
├── radar (雷达坐标系)
│   └── 平移: z = +0.5m (雷达安装高度)
│   └── camera (相机坐标系)
│       └── 平移: z = +1.2m (相机安装高度)
│
└── base_link (车辆本体坐标系)
    └── 用于车辆动态数据的参考系
```

- `radar → map`：由 `adc_rx` 节点发布静态 TF（identity + 高度偏移）
- `camera → radar`：由 `camera_rx` 节点发布静态 TF（高度偏移）
- 坐标变换由 `tf2_ros::StaticTransformBroadcaster` 实现

---

## 五、核心算法（模拟实现说明）

### 5.1 雷达信号处理 (rsp_mil_python / rsp_cuda)

模拟 pipeline：
1. **输入**：ADC 原始点云 (x, y, z, intensity)
2. **SNR 滤波**：`intensity >= snr_threshold` 的点保留
3. **速度补偿**：`velocity = scale * (distance * 0.1 - ego_vx)`
4. **输出**：检测目标列表 (x, y, z, velocity, snr)

Python 和 CUDA 版本的区别：
- **SNR 阈值**：Python 版 10dB，CUDA 版 8dB（模拟更高的灵敏度）
- **输出话题**：Python 版 `/ft/det_list_py`，CUDA 版 `/ft/det_list_cu`

### 5.2 3D 目标检测 (object_detection_3d)

模拟 AI 模型 pipeline：
1. **输入**：Python 检测列表 (x, y, z, velocity, snr)
2. **欧氏聚类**：BFS 实现的简单空间聚类
3. **簇过滤**：`len(cluster) >= min_cluster_size`
4. **框生成**：基于簇的 min/max 生成 3D 目标框
5. **输出**：MarkerArray（CUBE 框 + TEXT_VIEW_FACING 标签）

---

## 六、扩展点

框架预留了以下接口，用于接入真实硬件和算法：

| 节点 | 扩展接口 | 接入方式 |
|------|---------|----------|
| `adc_rx` | `_on_timer()` | 替换为真实 v4l2 驱动读取 |
| `camera_rx` | `_on_timer()` | 替换为真实 v4l2/GStreamer 读取 |
| `vehicle_data_rx` | `_on_timer()` | 替换为 CAN/ETH 总线读取 |
| `rsp_mil_python` | `_on_process()` | 替换为真实雷达信号处理算法 |
| `rsp_cuda` | `_on_process()` | 替换为 CUDA 加速信号处理 |
| `object_detection_3d` | `_on_det_list()` | 接入 AI 模型（TensorRT/ONNX） |
| `logging_node` | `_on_data()` | 添加文件写入（bin/mp4/csv） |

---

## 七、性能设计

### 7.1 发布频率

| 节点 | 频率 | 说明 |
|------|------|------|
| `adc_rx` | 10 Hz | 匹配典型雷达帧率 |
| `camera_rx` | 15 Hz | 匹配典型相机帧率 |
| `vehicle_data_rx` | 20 Hz | 高于传感器频率，确保及时性 |
| `rsp_mil_python` | 10 Hz | 随 ADC 数据处理 |
| `rsp_cuda` | 10 Hz | 随 ADC 数据处理 |
| `rviz_radar` | 10 Hz | 实时可视化 |
| `rviz_ruler` | 2 Hz | 标尺变化慢，低频即可 |

### 7.2 QoS 设置

- 所有话题使用默认 QoS：`depth=10`, `RELIABLE`, `VOLATILE`
- 可视化话题可通过参数调整 depth

---

## 八、目录结构设计原则

```
Orin-ROS/
├── src/          # ROS2 包源码（colcon workspace 标准结构）
├── config/       # 配置文件（RViz、参数等）
├── docs/         # 项目文档
├── scripts/      # 构建和启动脚本
└── 参考/         # 设计参考和原始需求文档
```

- **src/**：遵循 colcon workspace 规范，每个 ROS2 包一个子目录
- **config/**：存放 RViz 配置、参数 YAML 文件等
- **docs/**：架构设计、使用指南等独立于代码的文档
- **scripts/**：不依赖 ROS2 包系统的便利脚本
- **参考/**：保留原始需求文档和参考实现，不使用 git submodule

---

**作者：** zhengyuan.liu  
**创建日期：** 2026.6.8
