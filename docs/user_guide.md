# FT Framework — 用户使用指南

> 节点参数详解、调试技巧和常见问题

---

## 目录

- [环境准备](#环境准备)
- [启动方式](#启动方式)
- [节点详解](#节点详解)
- [参数速查表](#参数速查表)
- [调试技巧](#调试技巧)
- [常见问题](#常见问题)

---

## 环境准备

### 1. ROS2 Humble 安装

```bash
# 参考官方文档安装 ROS2 Humble
# https://docs.ros.org/en/humble/Installation.html

# 每次打开终端需 source 环境
source /opt/ros/humble/setup.bash
```

### 2. 构建工作空间

```bash
cd ~/Orin-ROS

# 首次构建
colcon build --packages-select ft_framework --symlink-install

# 后续增量构建
colcon build --packages-select ft_framework --symlink-install

# 加载工作空间
source install/setup.bash
```

推荐将 source 命令添加到 `~/.bashrc`：
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Orin-ROS/install/setup.bash" >> ~/.bashrc
```

---

## 启动方式

### 方式一：Launch 文件（推荐，启动全部 10 个节点）

```bash
ros2 launch ft_framework ft_framework.launch.py
```

### 方式二：脚本启动

```bash
bash scripts/launch_all.sh
```

### 方式三：手动逐个启动（调试用）

```bash
# 终端 1：数据采集层
ros2 run ft_framework adc_rx
# 终端 2：
ros2 run ft_framework camera_rx
# 终端 3：
ros2 run ft_framework vehicle_data_rx
# 终端 4：信号处理层
ros2 run ft_framework rsp_mil_python
# 终端 5：
ros2 run ft_framework rsp_cuda
# 终端 6：可视化与日志
ros2 run ft_framework rviz_radar
# 终端 7：
ros2 run ft_framework rviz_image
# 终端 8：
ros2 run ft_framework logging_node
# 终端 9：高级感知
ros2 run ft_framework object_detection_3d
# 终端 10：
ros2 run ft_framework rviz_ruler
```

### 方式四：按层级逐层启动

```bash
# 先启动第一层（数据采集），确认数据流正常
ros2 run ft_framework adc_rx &
ros2 run ft_framework camera_rx &
ros2 run ft_framework vehicle_data_rx &

# 验证数据流
ros2 topic echo /ft/adc_data --once
ros2 topic echo /ft/video_raw --once
ros2 topic echo /ft/vehicle_data --once

# 启动后续层级...
```

---

## 节点详解

### 第一层：数据采集

#### adc_rx — 雷达 ADC 数据接收节点

模拟 v4l2 接口采集雷达 ADC 数据。生成随机球坐标目标点，转换为笛卡尔坐标后以 PointCloud2 格式发布。

```bash
# 启动
ros2 run ft_framework adc_rx

# 自定义参数
ros2 run ft_framework adc_rx --ros-args \
  -p radar_fps:=20.0 \
  -p num_targets:=100 \
  -p range_max:=500.0
```

#### camera_rx — 相机数据接收节点

模拟 v4l2 接口采集相机视频。生成包含渐变色背景、移动圆和帧号文字的测试图案。

```bash
ros2 run ft_framework camera_rx --ros-args \
  -p camera_fps:=30.0 \
  -p image_width:=1920 \
  -p image_height:=1080
```

#### vehicle_data_rx — 车辆数据接收节点

模拟 CAN/ETH 接口采集车辆动态数据（车速、航向角速度）。

```bash
ros2 run ft_framework vehicle_data_rx --ros-args \
  -p sim_speed_mean:=20.0 \
  -p sim_yaw_rate:=0.1
```

### 第二层：雷达信号处理

#### rsp_mil_python — Python 雷达信号处理节点

订阅 ADC 数据和车辆数据，执行 SNR 滤波和速度补偿（解决速度模糊）。

```bash
ros2 run ft_framework rsp_mil_python --ros-args \
  -p snr_threshold:=12.0 \
  -p velocity_scale:=0.8
```

#### rsp_cuda — CUDA 雷达信号处理节点（模拟）

与 Python 版并行，使用更低的 SNR 阈值模拟 GPU 加速的高灵敏度处理。

```bash
ros2 run ft_framework rsp_cuda --ros-args \
  -p snr_threshold:=5.0
```

### 第三层：可视化与日志

#### rviz_radar — 雷达可视化节点

汇聚 Python 和 CUDA 检测列表、3D 目标列表和标尺数据，发布 RViz 可视化消息。

```bash
ros2 run ft_framework rviz_radar --ros-args \
  -p min_z:=-10.0 \
  -p max_z:=20.0
```

#### rviz_image — 图像可视化节点

接收原始视频帧，叠加帧号和时间戳信息后发布。

```bash
ros2 run ft_framework rviz_image --ros-args \
  -p show_overlay:=false
```

#### logging_node — 数据日志记录节点

订阅全部 5 个上游数据源，定期输出接收统计。文件写入功能待后续实现。

```bash
ros2 run ft_framework logging_node --ros-args \
  -p status_log_interval:=10.0
```

### 第四层：高级感知

#### object_detection_3d — 3D 目标检测节点

基于欧氏聚类的模拟 AI 目标检测，生成 3D 目标框和 ID 标签。

```bash
ros2 run ft_framework object_detection_3d --ros-args \
  -p cluster_distance:=3.0 \
  -p min_cluster_size:=5
```

#### rviz_ruler — 标尺参考节点

发布坐标尺标记（数字、刻度线、坐标轴线），为雷达可视化提供空间参考。

```bash
ros2 run ft_framework rviz_ruler --ros-args \
  -p ruler_axis:=y \
  -p ruler_offset:=30.0
```

---

## 参数速查表

### adc_rx

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `radar_fps` | double | 10.0 | 雷达帧率 (Hz) |
| `num_targets` | int | 50 | 每帧模拟目标数 |
| `range_max` | double | 300.0 | 最大探测距离 (m) |
| `range_min` | double | 1.0 | 最小探测距离 (m) |
| `azimuth_range` | double | 90.0 | 方位角范围 (±°) |
| `elevation_range` | double | 15.0 | 俯仰角范围 (±°) |
| `fixed_frame` | str | radar | RViz 坐标系 |

### camera_rx

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `camera_fps` | double | 15.0 | 相机帧率 (Hz) |
| `image_width` | int | 1280 | 图像宽度 (px) |
| `image_height` | int | 720 | 图像高度 (px) |
| `fixed_frame` | str | camera | RViz 坐标系 |

### vehicle_data_rx

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `vehicle_fps` | double | 20.0 | 车辆数据更新率 (Hz) |
| `sim_speed_mean` | double | 15.0 | 模拟平均车速 (m/s) |
| `sim_speed_std` | double | 2.0 | 车速噪声标准差 (m/s) |
| `sim_yaw_rate` | double | 0.05 | 航向角变化率 (rad/s) |
| `fixed_frame` | str | base_link | RViz 坐标系 |

### rsp_mil_python / rsp_cuda

| 参数 | 类型 | 默认值(PY/CU) | 说明 |
|------|------|---------------|------|
| `processing_fps` | double | 10.0 | 处理帧率 (Hz) |
| `snr_threshold` | double | 10.0 / 8.0 | SNR 阈值 (dB) |
| `velocity_scale` | double | 0.5 | 速度缩放因子 |
| `fixed_frame` | str | radar | RViz 坐标系 |

### rviz_radar

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `min_z` | double | -5.0 | 色带下限 (m) |
| `max_z` | double | 15.0 | 色带上限 (m) |
| `marker_lifetime` | double | 1.0 | Marker 生命周期 (s) |
| `publish_hz` | double | 10.0 | 发布频率 (Hz) |
| `fixed_frame` | str | radar | RViz 坐标系 |

### object_detection_3d

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `cluster_distance` | double | 5.0 | 聚类距离阈值 (m) |
| `min_cluster_size` | int | 3 | 最小簇大小 |
| `box_height` | double | 2.0 | 默认目标框高度 (m) |
| `marker_lifetime` | double | 1.0 | Marker 生命周期 (s) |
| `fixed_frame` | str | radar | RViz 坐标系 |

### rviz_ruler

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ruler_axis` | str | x | 标尺方向 (x / y) |
| `ruler_offset` | double | -50.0 | 正交方向偏移 (m) |
| `ruler_interval` | double | 20.0 | 标记间隔 (m) |
| `ruler_length` | double | 300.0 | 标尺总长度 (m) |
| `ruler_font` | double | 0.8 | 字体大小 |
| `ruler_color` | double[] | [0.8,0.8,0.8] | RGB 颜色 (0~1) |
| `publish_hz` | double | 2.0 | 发布频率 (Hz) |
| `fixed_frame` | str | radar | RViz 坐标系 |

### logging_node

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `status_log_interval` | double | 5.0 | 状态输出间隔 (s) |

---

## 调试技巧

### 1. 检查节点状态

```bash
# 列出所有运行中的节点
ros2 node list

# 查看特定节点信息
ros2 node info /adc_rx
ros2 node info /rsp_mil_python
```

### 2. 检查话题连接

```bash
# 列出所有活跃话题
ros2 topic list

# 查看话题的发布者和订阅者
ros2 topic info /ft/adc_data --verbose
ros2 topic info /ft/det_list_py --verbose

# 查看话题发布频率
ros2 topic hz /ft/adc_data
ros2 topic hz /ft/det_list_py
```

### 3. 查看话题数据

```bash
# 查看单条消息
ros2 topic echo /ft/adc_data --once

# 持续查看
ros2 topic echo /ft/vehicle_data

# 查看特定字段
ros2 topic echo /ft/det_list_py --field width
```

### 4. 调整日志级别

```bash
# 启动时设置 DEBUG 级别查看更多信息
ros2 run ft_framework adc_rx --ros-args --log-level DEBUG

# 运行时动态调整
ros2 param set /adc_rx log_level DEBUG
```

### 5. 检查 TF 树

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查特定坐标变换
ros2 run tf2_ros tf2_echo map radar
```

### 6. 使用 rqt 图形化工具

```bash
# 节点图
rqt_graph

# 话题监控
rqt_plot /ft/vehicle_data/twist/linear/x

# 参数配置
rqt_reconfigure
```

---

## 常见问题

### Q: 构建失败，提示 "package not found"

确保在正确的目录下运行构建命令，且 package.xml 中声明的依赖都已安装：
```bash
cd ~/Orin-ROS
colcon build --packages-select ft_framework --symlink-install
```

### Q: 启动节点后看不到任何话题

检查节点是否正确启动：
```bash
ros2 node list          # 确认节点在列表中
ros2 topic list         # 确认话题被发布
```

### Q: RViz 中看不到点云

1. 确认 Fixed Frame 设置为 `radar`
2. 确认 PointCloud2 的 Color Transformer 设置为 `RGB8`
3. 确认话题 `/ft/radar_display` 有数据发布

### Q: RViz 报 "Could not transform from [radar] to [map]"

检查 TF 是否正确发布：
```bash
ros2 run tf2_ros tf2_echo map radar
```
节点启动时会自动发布静态 TF，如果仍报错，检查节点是否正常启动。

### Q: cv_bridge 导入错误

cv_bridge 随 ROS2 完整安装包含。如遇到导入错误：
```bash
sudo apt install ros-humble-cv-bridge
```

### Q: 如何在实际硬件上运行

1. 将 `adc_rx.py` 的 `_on_timer()` 替换为真实 v4l2 读取代码
2. 将 `camera_rx.py` 的 `_on_timer()` 替换为真实相机驱动
3. 将 `vehicle_data_rx.py` 的 `_on_timer()` 替换为 CAN 总线读取
4. 在 `rsp_mil_python.py` / `rsp_cuda.py` 中实现真实的雷达信号处理算法
5. 在 `object_detection_3d.py` 中接入 AI 推理引擎（如 TensorRT）

---

**作者：** zhengyuan.liu  
**创建日期：** 2026.6.8
