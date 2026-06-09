# FT Radar Framework — 功能测试指南

> 本指南提供对 10 节点 ROS2 框架的完整逐层测试方案。

---

## 目录

- [前置检查](#前置检查)
- [测试流程总览](#测试流程总览)
- [阶段 0：环境和构建验证](#阶段-0环境和构建验证)
- [阶段 1：自定义消息验证](#阶段-1自定义消息验证)
- [阶段 2：数据采集层节点测试](#阶段-2数据采集层节点测试)
  - [2.1 ADC Rx 测试](#21-adc-rx-测试)
  - [2.2 Camera Rx 测试](#22-camera-rx-测试)
  - [2.3 Vehicle Data Rx 测试](#23-vehicle-data-rx-测试)
- [阶段 3：信号处理层节点测试](#阶段-3信号处理层节点测试)
  - [3.1 RSP MIL Python 测试](#31-rsp-mil-python-测试)
  - [3.2 RSP Cuda 测试](#32-rsp-cuda-测试)
  - [3.3 RSP 模式切换测试](#33-rsp-模式切换测试)
- [阶段 4：高级感知层节点测试](#阶段-4高级感知层节点测试)
  - [4.1 3D Object Detection 测试](#41-3d-object-detection-测试)
  - [4.2 Rviz_Ruler 测试](#42-rviz_ruler-测试)
- [阶段 5：可视化层节点测试](#阶段-5可视化层节点测试)
  - [5.1 Rviz_radar 测试](#51-rviz_radar-测试)
  - [5.2 Rviz_Image 测试](#52-rviz_image-测试)
- [阶段 6：Logging 节点完整测试](#阶段-6logging-节点完整测试)
  - [6.1 5 通道独立开关测试](#61-5-通道独立开关测试)
  - [6.2 输出文件格式验证](#62-输出文件格式验证)
  - [6.3 帧数上限测试](#63-帧数上限测试)
  - [6.4 标定文件加载测试](#64-标定文件加载测试)
- [阶段 7：全系统集成测试](#阶段-7全系统集成测试)
- [阶段 8：RViz 可视化测试](#阶段-8rviz-可视化测试)
- [测试通过标准](#测试通过标准)

---

## 前置检查

```bash
# 确认 ROS2 环境
source /opt/ros/humble/setup.bash
ros2 --version
# 预期: 输出 ROS2 版本信息

# 确认项目路径
cd ~/Orin-ROS
ls src/           # 应显示 ft_framework 和 ft_radar_msgs
ls data/          # 应显示测试数据文件
ls config/        # 应显示 ft_radar_params.yaml 和 ft_radar.rviz
```

---

## 测试流程总览

```
阶段 0: 环境 & 构建
    ↓
阶段 1: 消息定义
    ↓
阶段 2: 数据采集层 (3 个 Rx 节点)
    ↓
阶段 3: 信号处理层 (RSP ×2 + 模式切换)
    ↓
阶段 4: 高级感知层 (3D Detection + Ruler)
    ↓
阶段 5: 可视化层 (Rviz_radar + Rviz_Image)
    ↓
阶段 6: Logging 系统 (5 通道 + 输出文件)
    ↓
阶段 7: 全系统集成
    ↓
阶段 8: RViz 可视化
```

---

## 阶段 0：环境和构建验证

### 0.1 构建（一步式）

```bash
cd ~/Orin-ROS

# 一键加载环境 + 构建（自动按依赖顺序编译两个包）
bash scripts/build.sh
# 预期: ✅ 构建成功！（Summary: 2 packages finished）
```

> `bash scripts/build.sh` 自动完成：加载 ROS2 Humble → 编译 `ft_radar_msgs` → 加载 → 编译 `ft_framework` → 加载工作空间。一步到位。

如需分步手动构建，也可使用 colcon 命令操作。

### 0.2 验证可执行文件

```bash
ros2 run ft_framework adc_rx --help
ros2 run ft_framework camera_rx --help
ros2 run ft_framework vehicle_data_rx --help
ros2 run ft_framework rsp_mil_python --help
ros2 run ft_framework rsp_cuda --help
ros2 run ft_framework rviz_radar --help
ros2 run ft_framework rviz_image --help
ros2 run ft_framework logging_node --help
ros2 run ft_framework object_detection_3d --help
ros2 run ft_framework rviz_ruler --help
# 预期: 10 个命令均输出 usage 信息（无报错）
```

---

## 阶段 1：自定义消息验证

### 1.1 消息列表

```bash
ros2 interface list | grep ft_radar_msgs
# 预期输出 6 个消息:
#   ft_radar_msgs/msg/AdcRawData
#   ft_radar_msgs/msg/DetPoint
#   ft_radar_msgs/msg/DetList
#   ft_radar_msgs/msg/Object3D
#   ft_radar_msgs/msg/ObjList
#   ft_radar_msgs/msg/EgoMotion
```

### 1.2 逐消息验证字段

```bash
# AdcRawData: 3 个 uint32 + 1 个 int16[] + header
ros2 interface show ft_radar_msgs/msg/AdcRawData

# DetPoint: 14 字段 (9 float32 + 4 uint8 + 1 float32)
ros2 interface show ft_radar_msgs/msg/DetPoint

# DetList: header + DetPoint[]
ros2 interface show ft_radar_msgs/msg/DetList

# Object3D: 14 字段 (2 uint64 + 11 float32 + 1 uint8)
ros2 interface show ft_radar_msgs/msg/Object3D

# ObjList: header + Object3D[]
ros2 interface show ft_radar_msgs/msg/ObjList

# EgoMotion: header + 6 float64 + 1 uint8 + 1 bool
ros2 interface show ft_radar_msgs/msg/EgoMotion
```

**通过标准**: 每行 `ros2 interface show` 输出与 `参考/FT_radar_dataset_requirement.md` 第 3/5/7 节字段定义完全一致。

---

## 阶段 2：数据采集层节点测试

### 2.1 ADC Rx 测试

```bash
# 终端 1: 启动 ADC Rx
cd ~/Orin-ROS && source install/setup.bash
ros2 run ft_framework adc_rx --ros-args \
  -p fps:=15 \
  -p num_rows:=512 \
  -p num_chirps_per_row:=16 \
  -p num_samples_per_chirp:=2048
# 预期日志:
#   [INFO] ADC Rx 启动: 15 Hz, 512 rows × 16 chirps/row × 2048 samples/chirp, 每帧 32.0 MB

# 终端 2: 验证话题
cd ~/Orin-ROS && source install/setup.bash

# 检查话题存在
ros2 topic list | grep /adc/raw_data
# 预期: /adc/raw_data

# 检查消息类型
ros2 topic info /adc/raw_data
# 预期: Type: ft_radar_msgs/msg/AdcRawData

# 检查发布频率
ros2 topic hz /adc/raw_data
# 预期: average rate: ~15.000 Hz

# 检查单条消息内容
ros2 topic echo /adc/raw_data --once
# 预期输出:
#   num_rows: 512
#   num_chirps_per_row: 16
#   num_samples_per_chirp: 2048
#   data: [-100, 85, -32, ...] (int16 数组)

# 终端 1: Ctrl+C 停止
# 预期: [INFO] ADC Rx 已停止（共处理 N 帧）
```

### 2.2 Camera Rx 测试

```bash
# 终端 1
ros2 run ft_framework camera_rx --ros-args \
  -p fps:=30 \
  -p image_width:=640 \
  -p image_height:=480
# 预期: [INFO] Camera Rx 启动: 30 Hz, 640x480

# 终端 2
ros2 topic list | grep /camera/image_raw
# 预期: /camera/image_raw

ros2 topic info /camera/image_raw
# 预期: Type: sensor_msgs/msg/Image

ros2 topic hz /camera/image_raw
# 预期: average rate: ~30.000 Hz

ros2 topic echo /camera/image_raw --once
# 预期: height: 480, width: 640, encoding: bgr8 (非空图像数据)

# Ctrl+C 停止
```

### 2.3 Vehicle Data Rx 测试

```bash
# 终端 1
ros2 run ft_framework vehicle_data_rx --ros-args \
  -p fps:=50 \
  -p timeout_cycles:=3
# 预期: [INFO] Vehicle Data Rx 启动: 50 Hz, 超时检测: 3 周期 (0.1s)

# 终端 2
ros2 topic list | grep /vehicle/ego_motion
ros2 topic info /vehicle/ego_motion
# 预期: Type: ft_radar_msgs/msg/EgoMotion

ros2 topic hz /vehicle/ego_motion
# 预期: average rate: ~50.000 Hz

ros2 topic echo /vehicle/ego_motion --once
# 预期:
#   vx: ~15.0 (模拟车速)
#   yaw_rate: ~0.05
#   steering_angle: 非零
#   ax: 小波动
#   ay: 非零 (向心加速度)
#   gear: 1
#   is_default: false

# ---- 默认值超时测试 ----
# 确认 is_default 为 false（模拟正常数据）
ros2 topic echo /vehicle/ego_motion --once | grep is_default
# 预期: is_default: false

# Ctrl+C 停止
```

---

## 阶段 3：信号处理层节点测试

> **前置**: 先在一个 `ros2 launch` 中启动全部数据采集节点以提供上游话题

### 3.1 RSP MIL Python 测试

```bash
# 终端 1: 启动数据源
ros2 run ft_framework adc_rx --ros-args -p fps:=15 &
ros2 run ft_framework vehicle_data_rx --ros-args -p fps:=50 &
sleep 1

# 终端 2: 启动 Python RSP
ros2 run ft_framework rsp_mil_python --ros-args \
  -p processing_fps:=10.0 \
  -p snr_threshold:=10.0 \
  -p rsp_mode:=python
# 预期: [INFO] RSP MIL Python 发布: /processing/radar/det_list

# 终端 3: 验证输出
ros2 topic list | grep /processing/radar/det_list
# 预期: /processing/radar/det_list

ros2 topic info /processing/radar/det_list
# 预期: Type: ft_radar_msgs/msg/DetList

ros2 topic hz /processing/radar/det_list
# 预期: average rate: ~10.000 Hz

# 验证 DetPoint 14 字段
ros2 topic echo /processing/radar/det_list --once
# 预期输出包含: x, y, z, range, azimuth, elevation, rcs, snr,
#              ambgt, exist_prob, multi_tgt_prob, ambgt_prob, raw_doppler, idx
# 每个字段都有合法数值（非零、非空）

# 停止
kill %1 %2 %3  # 清理终端 1 的进程
pkill -f rsp_mil_python
```

### 3.2 RSP Cuda 测试

```bash
# 终端 1: 启动数据源
ros2 run ft_framework adc_rx --ros-args -p fps:=15 &
ros2 run ft_framework vehicle_data_rx --ros-args -p fps:=50 &
sleep 1

# 终端 2: 启动 CUDA RSP
ros2 run ft_framework rsp_cuda --ros-args \
  -p processing_fps:=10.0 \
  -p snr_threshold:=8.0 \
  -p rsp_mode:=cuda
# 预期: [INFO] RSP Cuda 发布: /processing/radar/det_list

# 终端 3
ros2 topic info /processing/radar/det_list
# 预期: Type: ft_radar_msgs/msg/DetList

# 比较: CUDA 版 (snr=8.0) 应比 Python 版 (snr=10.0) 有更多检测点
ros2 topic echo /processing/radar/det_list --once
# 预期: points 数组长度 > 0，且 ≥ Python 版

pkill -f adc_rx; pkill -f vehicle_data_rx; pkill -f rsp_cuda
```

### 3.3 RSP 模式切换测试

```bash
# 模式 1: both（双路并行）
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=both
# 检查
ros2 topic list | grep processing/radar
# 预期输出两个话题:
#   /processing/radar/det_list       (来自 rsp_mil_python)
#   /processing/radar/det_list_cuda   (来自 rsp_cuda)

ros2 node list | grep rsp
# 预期输出两个节点:
#   /rsp_mil_python
#   /rsp_cuda
# Ctrl+C

# 模式 2: CUDA（默认）
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda
ros2 node list | grep rsp
# 预期只有: /rsp_cuda
# Ctrl+C

# 模式 3: Python
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=python
ros2 node list | grep rsp
# 预期只有: /rsp_mil_python
# Ctrl+C
```

---

## 阶段 4：高级感知层节点测试

### 4.1 3D Object Detection 测试

```bash
# 启动完整上游链路
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda &
sleep 3

# 验证输入话题存在
ros2 topic info /processing/radar/det_list
# 预期: Type: ft_radar_msgs/msg/DetList

# 验证输出
ros2 topic list | grep /perception/objects
# 预期: /perception/objects

ros2 topic info /perception/objects
# 预期: Type: ft_radar_msgs/msg/ObjList

# 验证 Object3D 14 字段
ros2 topic echo /perception/objects --once
# 预期输出:
#   object_id: 非零整数
#   tracked_times: 正整数
#   score: [0.0, 1.0]
#   x, y, z: m
#   l, w, h: m
#   yaw: rad
#   vx_absolute, vy_absolute, vz_absolute: m/s
#   moving_state: 0-5

# 验证同时发布的 MarkerArray（供 RViz）
ros2 topic list | grep /visualization/radar/boxes
# 预期: /visualization/radar/boxes

ros2 topic echo /visualization/radar/boxes --once | head -20
# 预期: 包含 CUBE 和 TEXT_VIEW_FACING 类型的 Marker

kill %1
```

### 4.2 Rviz_Ruler 测试

```bash
ros2 run ft_framework rviz_ruler --ros-args \
  -p ruler_axis:=x \
  -p ruler_length:=200.0 &
sleep 1

# 验证话题
ros2 topic list | grep /visualization/ruler
# 预期: /visualization/ruler

ros2 topic hz /visualization/ruler
# 预期: ~2.000 Hz

# 验证内容
ros2 topic echo /visualization/ruler --once
# 预期: MarkerArray 包含 TEXT_VIEW_FACING (数字标签) + LINE_STRIP (刻度线 + 坐标轴)

pkill -f rviz_ruler
```

---

## 阶段 5：可视化层节点测试

### 5.1 Rviz_radar 测试

```bash
# 完整链路启动
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=both &
sleep 3

# 验证 4 个输出话题
ros2 topic list | grep /visualization/radar/
# 预期 4 个话题全部存在:
#   /visualization/radar/display      (PointCloud2)
#   /visualization/radar/boxes        (MarkerArray)
#   /visualization/radar/colorbar     (Image)
#   /visualization/radar/frame_info   (MarkerArray)

# 验证点云
ros2 topic echo /visualization/radar/display --once
# 预期: PointCloud2, fields 包含 x/y/z/rgb

# 验证目标框（应合并 obj_3d_boxes + ruler）
ros2 topic echo /visualization/radar/boxes --once
# 预期: MarkerArray, markers 包含 obj_3d_boxes 命名空间和 ruler_labels 命名空间

# 验证帧信息
ros2 topic echo /visualization/radar/frame_info --once
# 预期: text 字段如 "Det: 30 | Obj: 3 | Frame: 42"

kill %1
```

### 5.2 Rviz_Image 测试

```bash
ros2 run ft_framework camera_rx --ros-args -p image_width:=640 -p image_height:=480 &
sleep 1
ros2 run ft_framework rviz_image &
sleep 1

ros2 topic list | grep /visualization/camera/display
# 预期: /visualization/camera/display

ros2 topic echo /visualization/camera/display --once
# 预期: Image, encoding: bgr8, 非空 data

pkill -f camera_rx; pkill -f rviz_image
```

---

## 阶段 6：Logging 节点完整测试

### 6.1 5 通道独立开关测试

```bash
# 启动框架 + Logging（全部开关开启）
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda \
  log_adc:=true log_image:=true log_det_list:=true \
  log_ego_motion:=true log_obj_list:=true &
sleep 5

# 检查 Logging 状态输出
# 预期每 5s 输出: [Logging 状态] 运行 X.Xs | adc=N/100 | image=N/1000 | ...

# ---- 运行时关闭 ADC 录制 ----
ros2 param set /logging_node enable_adc false
# 预期日志: （不报错）

ros2 param get /logging_node enable_adc
# 预期: Boolean value is: False

# ---- 运行时重新开启 ----
ros2 param set /logging_node enable_adc true
ros2 param get /logging_node enable_adc
# 预期: Boolean value is: True

# ---- 验证 5 个开关全部可操作 ----
ros2 param set /logging_node enable_image false
ros2 param get /logging_node enable_image
# 预期: Boolean value is: False

ros2 param set /logging_node enable_det_list false
ros2 param get /logging_node enable_det_list
# 预期: Boolean value is: False

ros2 param set /logging_node enable_ego_motion false
ros2 param get /logging_node enable_ego_motion
# 预期: Boolean value is: False

ros2 param set /logging_node enable_obj_list false
ros2 param get /logging_node enable_obj_list
# 预期: Boolean value is: False

# 全部恢复
ros2 param set /logging_node enable_adc true
ros2 param set /logging_node enable_image true
ros2 param set /logging_node enable_det_list true
ros2 param set /logging_node enable_ego_motion true
ros2 param set /logging_node enable_obj_list true

kill %1
```

### 6.2 输出文件格式验证

```bash
# 启动框架，指定输出目录
OUT_DIR=/tmp/ft_test_$(date +%Y%m%d_%H%M%S)
mkdir -p $OUT_DIR

ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda &
sleep 5

# 修改输出目录
ros2 param set /logging_node output_dir $OUT_DIR

# 等待数据收集
sleep 10

# 检查文件
ls -la $OUT_DIR/
# 预期:
#   adc.bin           (非空，二进制)
#   ego_motion.csv    (非空，包含 header 行 + 数据行)
#   {timestamp}.jpg   (至少 1 个)
#   {timestamp}.csv   (至少 1 个，对应 det_list)
#   {timestamp}.pcd   (至少 1 个，对应 det_list)
#   {timestamp}.csv   (至少 1 个，对应 obj_list)

# === 验证 adc.bin ===
# 前 20 bytes = 8B 时间戳 + 4B rows + 4B chirps_per_row + 4B samples
hexdump -C $OUT_DIR/adc.bin | head -2
# 预期: 有数据输出

# === 验证 ego_motion.csv ===
head -3 $OUT_DIR/ego_motion.csv
# 预期:
#   timestamp_us,vx,yaw_rate,steering_angle,ax,ay,gear
#   1083......,15...... (数据行)
# 与 data/sample_ego_motion.csv 格式一致

# === 验证 det_list CSV ===
PCD_CSV=$(ls $OUT_DIR/*.csv | head -1)
head -1 $PCD_CSV
# 预期: x,y,z,range,azimuth,elevation,RCS,SNR,ambgt,exist_prob,multi_tgt_prob,ambgt_prob,raw_doppler,idx
# 与 data/sample_det_list.csv header 完全一致

# === 验证 PCD ===
PCD_FILE=$(ls $OUT_DIR/*.pcd | head -1)
head -12 $PCD_FILE
# 预期:
#   # .PCD v0.7
#   VERSION 0.7
#   FIELDS x y z range azimuth elevation RCS SNR ambgt exist_prob multi_tgt_prob ambgt_prob raw_doppler idx
#   SIZE 4 4 4 4 4 4 4 4 4 1 1 1 4 1
#   TYPE F F F F F F F F F U U U F U
#   DATA ascii

# === 验证 obj_list CSV ===
OBJ_CSV=$(ls $OUT_DIR/*.csv | tail -1)
head -1 $OBJ_CSV
# 预期: object_id,tracked_times,score,x,y,z,l,w,h,yaw,vx_absolute,vy_absolute,vz_absolute,moving_state
# 与 data/sample_obj_list.csv header 完全一致

# 停止
kill %1
echo "输出目录: $OUT_DIR"
```

### 6.3 帧数上限测试

```bash
# 启动框架 + Logging（降低上限以便快速触发）
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda &
sleep 3

# 设置极低的上限
ros2 param set /logging_node max_frames.adc 2
ros2 param set /logging_node max_frames.image 3

# 等待几秒触发上限
sleep 10

# 检查日志输出（应在终端中看到）:
# 预期: [WARN] adc 已达帧数上限 (2)，停止录制
# 预期: [WARN] image 已达帧数上限 (3)，停止录制

# 验证状态输出中的计数器停止增长
# 预期: adc=2/2, image=3/3（不再增长）

kill %1
```

### 6.4 标定文件加载测试

```bash
ros2 run ft_framework logging_node --ros-args \
  -p calibration_file:="$(pwd)/data/calibration/radar_front_center_ft.yaml" \
  -p output_dir:=/tmp/ft_calib_test &
sleep 2

# 检查标定文件是否被复制到输出目录
ls -la /tmp/ft_calib_test/calibration.yaml
# 预期: 文件存在，内容与 data/calibration/radar_front_center_ft.yaml 相同

diff /tmp/ft_calib_test/calibration.yaml \
     data/calibration/radar_front_center_ft.yaml
# 预期: 无差异（或仅空行差异）

# 检查日志
# 预期: [INFO] 标定文件已复制: ... → /tmp/ft_calib_test/calibration.yaml

pkill -f logging_node
```

---

## 阶段 7：全系统集成测试

### 7.1 完整启动

```bash
FULL_OUT_DIR=/tmp/ft_integration_test_$(date +%Y%m%d_%H%M%S)
mkdir -p $FULL_OUT_DIR

ros2 launch ft_framework ft_radar_launch.py rsp_mode:=cuda &
sleep 3

# 设置输出目录
ros2 param set /logging_node output_dir $FULL_OUT_DIR

# 运行 30 秒收集足量数据
echo "Running integration test for 30 seconds..."
sleep 30
```

### 7.2 节点完整性检查

```bash
ros2 node list
# 预期 10 个节点:
#   /adc_rx
#   /camera_rx
#   /vehicle_data_rx
#   /rsp_cuda                     (CUDA 模式下只有这个)
#   /rviz_radar
#   /rviz_image
#   /logging_node
#   /object_detection_3d
#   /rviz_ruler

# 不应出现 /rsp_mil_python（CUDA 模式）
ros2 node list | grep rsp_mil_python
# 预期: 无输出
```

### 7.3 全部话题连通性

```bash
# 验证每个关键 topic 的 pub/sub 匹配
ros2 topic info /adc/raw_data --verbose | grep "Node name: rsp_cuda\|Node name: logging_node"
# 预期: 必须匹配

ros2 topic info /processing/radar/det_list --verbose | grep "Node name: rviz_radar\|Node name: object_detection_3d\|Node name: logging_node"
# 预期: 必须匹配

ros2 topic info /perception/objects --verbose | grep "Node name: rviz_radar\|Node name: logging_node"
# 预期: 必须匹配

ros2 topic info /vehicle/ego_motion --verbose | grep "Node name: rsp_cuda\|Node name: logging_node"
# 预期: 必须匹配

ros2 topic info /visualization/ruler --verbose | grep "Node name: rviz_radar"
# 预期: 必须匹配
```

### 7.4 频率验证

```bash
ros2 topic hz /adc/raw_data                 # 预期: ~15 Hz
ros2 topic hz /camera/image_raw             # 预期: ~30 Hz
ros2 topic hz /vehicle/ego_motion           # 预期: ~50 Hz
ros2 topic hz /processing/radar/det_list    # 预期: ~10 Hz
ros2 topic hz /visualization/radar/display  # 预期: ~10 Hz
ros2 topic hz /visualization/ruler          # 预期: ~2 Hz
```

### 7.5 时间戳透传验证

```bash
# 检查 ADC 时间戳
ros2 topic echo /adc/raw_data --once 2>&1 | grep -E "sec|nanosec"
# 预期: sec 和 nanosec 为单调递增的有效值

# 检查 DetList 时间戳（应与 ADC 的时间接近）
ros2 topic echo /processing/radar/det_list --once 2>&1 | grep -E "sec|nanosec"
# 预期: 时间戳值与上游接近（允许处理延迟偏差）

# 检查 ObjList 时间戳（透传自 DetList）
ros2 topic echo /perception/objects --once 2>&1 | grep -E "sec|nanosec"
# 预期: 时间戳与 DetList 一致

# 检查 ego_motion 时间戳
ros2 topic echo /vehicle/ego_motion --once 2>&1 | grep -E "sec|nanosec"
```

### 7.6 数据内容抽样

```bash
# ADC 数据
ros2 topic echo /adc/raw_data --once 2>&1 | head -10
# 预期: num_rows: 512, data: [...] (int16 数组非空)

# EgoMotion 字段完整性
ros2 topic echo /vehicle/ego_motion --once 2>&1
# 预期: 包含 vx, yaw_rate, steering_angle, ax, ay, gear, is_default 全部字段

# DetPoint 14 字段完整性
ros2 topic echo /processing/radar/det_list --once 2>&1 | head -20
# 预期: 每个 point 包含全部 14 字段

# Object3D 14 字段完整性
ros2 topic echo /perception/objects --once 2>&1 | head -20
# 预期: 每个 object 包含 object_id, score, x/y/z, l/w/h, yaw, vx_abs 等全部字段
```

### 7.7 停止验证

```bash
# Ctrl+C 停止 launch
# 预期: 所有 10 个节点逐一输出 [INFO] ... 已停止 或收到中断信号

# 检查 Logging 最终统计
# 预期: [Logging 状态] 运行 XX.Xs | adc=NN/100 | image=NN/1000 | ...
```

---

## 阶段 8：RViz 可视化测试

> 需要 WSLg（WSL）或桌面环境支持

```bash
# 终端 1: 启动框架
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=both
sleep 2

# 终端 2: 启动 RViz
cd ~/Orin-ROS
source install/setup.bash
rviz2 -d config/ft_radar.rviz
```

### RViz 手动检查清单

| 显示项 | 话题 | 检查内容 |
|--------|------|---------|
| Radar Points | `/visualization/radar/display` | 彩色点云可见，Color Transformer 设为 RGB8 |
| Radar Boxes | `/visualization/radar/boxes` | 绿色目标框 + 黄色 ID 标签 + 灰色标尺同时显示 |
| Camera Display | `/visualization/camera/display` | 测试图案视频，含帧号和时间戳 overlay |
| Colorbar | `/visualization/radar/colorbar` | 高度色带图像 |
| Frame Info | `/visualization/radar/frame_info` | 白色帧信息文字 "Det: N \| Obj: M \| Frame: K" |
| TF | — | radar→map, camera→radar 两个 TF 正常显示 |

> 若 WSL 环境下 `rviz2` 启动失败，可仅通过 `ros2 topic echo` 验证数据内容的正确性，RViz 验证推迟到 Jetson Orin 真机上进行。

---

## 测试通过标准

| # | 检查项 | 通过条件 |
|:--:|--------|---------|
| 1 | 构建成功 | `colcon build` 两个包均无错误 |
| 2 | 消息定义 | `ros2 interface show` 输出与 dataset_requirement 完全一致 |
| 3 | 10 节点启动 | `ros2 node list` 显示 10 个节点 |
| 4 | 12 个话题存在 | `ros2 topic list \| grep '^/'` 显示所有预期话题 |
| 5 | pub/sub 匹配 | `ros2 topic info --verbose` 出版商/订阅商与框架描述一致 |
| 6 | 频率正确 | ADC 15Hz, Camera 30Hz, Vehicle 50Hz, RSP 10Hz, Ruler 2Hz |
| 7 | 时间戳透传 | 下游节点消息 timestamp 与上游一致 |
| 8 | DetPoint 14 字段 | `echo --once` 输出全部字段且值合法 |
| 9 | Object3D 14 字段 | `echo --once` 输出全部字段且值合法 |
| 10 | RSP 模式切换 | `both` 模式两节点均可见, `cuda`/`python` 模式仅一个节点 |
| 11 | Logging 5 开关 | `ros2 param set` 可单独关闭/开启每个通道 |
| 12 | CSV 格式 | 输出 CSV header 与 `sample_det_list.csv` / `sample_obj_list.csv` 一致 |
| 13 | PCD 格式 | `head -1 *.pcd` 显示 `# .PCD v0.7`, FIELDS 完全正确 |
| 14 | ego_motion CSV | 输出 CSV header 与 `sample_ego_motion.csv` 一致 |
| 15 | 帧数上限 | 达到上限后输出 `[WARN]` 并停止计数 |
| 16 | 标定加载 | `calibration.yaml` 正确复制到输出目录 |
| 17 | 优雅退出 | Ctrl+C 后所有节点正常停止，无报错 |

---

**作者**: zhengyuan.liu  
**创建日期**: 2026.6.9
