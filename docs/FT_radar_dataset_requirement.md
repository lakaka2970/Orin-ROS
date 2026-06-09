# FT Radar Dataset Requirement

> 本文档定义了 `FT Radar Dataset Requirement` 包含的数据格式，供准备FT雷达数据使用。

---

## 目录

1. [总览](#1-总览)
2. [目录结构](#2-目录结构)
3. [Egomotion CSV 文件](#3-egomotion-csv-文件)
4. [标定 YAML 文件](#4-标定-yaml-文件)
5. [雷达点云 PCD 文件](#5-雷达点云-pcd-文件)
6. [雷达点云 CSV 文件](#6-雷达点云-csv-文件)
7. [雷达目标 CSV 文件](#7-雷达目标-csv-文件)
8. [摄像头图片文件](#8-摄像头图片文件)
9. [坐标系定义](#9-坐标系定义)
10. [运行命令示例](#10-运行命令示例)

---

## 1. 总览

FT Radar Dataset的6类数据文件：

| 数据文件 | 格式 | 说明 |
|------|------|------|
| 自车运动 | CSV 文件 | 单文件，包含全部时间帧的位姿和速度 |
| 雷达标定 | YAML 文件 | 每个雷达一个文件，包含外参 |
| 雷达点云 | PCD 文件 (PCL格式) | 每帧一个文件，按传感器位置分目录存放 |
| 雷达点云 | CSV 文件 | 每帧一个文件，按传感器位置分目录存放 |
| 雷达目标 | CSV 文件 | 每帧一个文件，包含当前帧所有目标信息 |
| 摄像头图片 | jpg 文件 | 每帧一个文件，按传感器位置分目录存放 |

**时间同步要求:** 所有数据共用统一时间戳，单位为**微秒 (μs)**。程序会将 ego 帧和 radar 帧按时间戳排序后交替处理。

---

## 2. 目录结构

```
<dataset_root>/
├── ego_motion.csv                        # 自车运动数据
│
├── calibration/                          # 标定文件目录
│   ├── radar_front_center_ft.yaml
│   ├── radar_front_left_ft.yaml
│   ├── radar_front_right_ft.yaml
│   ├── radar_rear_left_ft.yaml
│   └── radar_rear_right_ft.yaml
│
├── pc_pcd_radar_front_center/            # 前雷达pcd点云 (pos=0)
│   ├── 1083029893.pcd
│   ├── 1083080582.pcd
│   └── ...
├── pc_pcd_radar_corner_x/                # 角雷达pcd点云 (pos=x)
│   └── ...
│
├── pc_csv_radar_front_center/            # 前雷达csv点云 (pos=0)
│   ├── 1083029893.csv
│   ├── 1083080582.csv
│   └── ...
├── pc_csv_radar_corner_x/                # 角雷达csv点云 (pos=x)
│   └── ...
│
├── obj_csv_radar/                        # 雷达csv目标
│   ├── 1083029893.csv
│   ├── 1083080582.csv
│   └── ...
│
├── camera_front_center/                  # 前视像头jpg图片
│   ├── 1083029893.jpg
│   ├── 1083080582.jpg
│   └── ...
```

**目录命名规则** — 程序通过目录名中的关键词推断雷达位置：

| 目录名包含 | 雷达位置 | pos 值 |
|-----------|---------|--------|
| `front_center` | 前方中央 | 0 |
| `front_left` | 左前角 | 1 |
| `side_left` | 左侧 | 2 |
| `rear_left` | 左后角 | 3 |
| `rear_center` | 后方中央 | 4 |
| `rear_right` | 右后角 | 5 |
| `side_right` | 右侧 | 6 |
| `front_right` | 右前角 | 7 |


---

## 3. Egomotion CSV 文件

### 3.1 文件格式

标准 CSV，首行为列名 header，逗号分隔，无引号。

### 3.2 列定义

| # | 列名 | 类型 | 单位 | 坐标系 | 必需性 | 描述 |
|---|------|------|------|--------|:------:|------|
| 1 | `timestamp_us` | uint64 | **μs** | - | **必需** | 时间戳（微秒） |
| 2 | `vx` | double | **m/s** | Local | **必需** | 车速 |
| 3 | `yaw_rate` | double | **rad/s** | Local | **必需** | 偏航角速度 |
| 4 | `steering_angle` | double | **rad** | Local | **必需** | 转向角度 |
| 5 | `ax` | double | m/s^2 | Local | **必需** | 纵向加速度 |
| 6 | `ay` | double | m/s^2 | Local | **必需** | 横向加速度 |
| 7 | `gear` | uint8 | - | Local | **必需** | 挡位 |

### 3.3 CSV 示例

```csv
timestamp_us,vx,yaw_rate,steering_angle,ax,ay,gear
1083050000,2.08708,-0.0532361,-0.930277,0.000537685,-0.000316581,1
```

### 3.4 时间频率要求

- **推荐频率:** ≥50 Hz（即间隔 ≤20ms）
- **最大允许间隔:** 500ms（两帧间隔超过此值无法插值）
- **前置要求:** 至少有 2 帧 ego pose 后才开始处理雷达帧

---

## 4. 标定 YAML 文件

### 4.1 文件命名规则

| 文件名 | 雷达位置 |
|--------|---------|
| `radar_front_center_ars620.yaml` | 前方中央 (pos=0) |
| `radar_front_left_hella.yaml` | 左前角 (pos=1) |
| `radar_front_right_hella.yaml` | 右前角 (pos=7) |
| `radar_rear_left_hella.yaml` | 左后角 (pos=3) |
| `radar_rear_right_hella.yaml` | 右后角 (pos=5) |

### 4.2 YAML 格式定义

```yaml
%YAML:1.0
---
sensor_name: "radar_front"          # 传感器命名（信息字段，不影响逻辑）
sensor_type: ft_fvr60_eo            # 传感器类型（信息字段）
vehicle_xyz: front_left_up          # 车体系定义: 前左上, 后轴中心接地点
r_s2b:                              # 旋转向量 (Rodrigues), 传感器系 → 车体系
  [3.141563, 0.01370774, -1.034906e-09]
t_s2b:                              # 平移向量 (m), 传感器系 → 车体系
  [4.052, -0.17, 0.677]
```

### 4.3 字段说明

| 字段 | 类型 | 单位 | 描述 |
|------|------|------|------|
| `r_s2b` | float[3] | **rad** | Rodrigues 旋转向量，定义传感器到车体系的旋转。变换: `p_body = R(r_s2b) × p_sensor` |
| `t_s2b` | float[3] | **m** | 平移向量 `[tx, ty, tz]`，传感器原点在车体系中的位置 |

**Rodrigues 旋转向量:** 向量方向为旋转轴，模长为旋转角度（弧度）。使用 `cv::Rodrigues()` 或 Eigen `AngleAxis` 可转为 3×3 旋转矩阵。

### 4.4 标定示例

**前方雷达 (ARS620):**
```yaml
r_s2b: [3.141563, 0.01370774, -1.034906e-09]   # 约绕X轴旋转180° (π rad)
t_s2b: [4.052, -0.17, 0.677]                    # 前方4.05m, 右偏0.17m, 高0.68m
```

**左前角雷达 (Hella SRR):**
```yaml
r_s2b: [0, 0, 1.068141]                         # 约绕Z轴旋转61.2°
t_s2b: [3.748548, 0.878803, 0.4602]             # 前方3.75m, 左偏0.88m, 高0.46m
```

### 4.5 可选字段（出现时会被读取但非必需）

```yaml
CLOCK_calib_version:      # 标定版本号（仅信息）
CLOCK_calib_details:      # 标定详情（仅信息）
CLOCK_calib_date:         # 标定日期（仅信息）
timestamp_shift: 0        # 时间戳偏移 ms（仅信息，程序未使用此字段）
```

---

## 5. 雷达点云 PCD 文件

### 5.1 文件格式

标准 PCL PCD 格式 (v0.7)，支持 ASCII 和 Binary 两种 `DATA` 模式。

pycd4 是一个用于处理 PCD（Point Cloud Data）文件的现代 Python 库，使用 Python 3 编写，其核心是 PointCloud 类，所有 PCD 文件都会读取为这个对象。

PCD是点云库（PCL，Point Cloud Library）的原生文件格式，而v0.7是官方指定的标准版本。

### 5.2 文件命名

文件名（不含 `.pcd` 后缀）为该帧的时间戳，单位**微秒 (μs)**。

```
<timestamp_us>.pcd
```

例如: `1083029893.pcd` 表示时间戳 1083029893 μs。

> 如果点云内部 `timestamp` 字段有效（>0），优先使用内部时间戳；否则回退到文件名。

### 5.3 字段定义

| # | 字段名 | SIZE | TYPE | 单位 | 必需性 | 描述 |
|---|--------|------|------|------|:------:|------|
| 1 | `x` | 4 | F (float32) | **m** | **必需** | 目标纵向坐标（车辆系，前方为正） |
| 2 | `y` | 4 | F (float32) | **m** | **必需** | 目标横向坐标（车辆系，左侧为正） |
| 3 | `z` | 4 | F (float32) | **m** | **必需** | 目标垂直坐标（车辆系，上方为正） |
| 4 | `range` | 4 | F (float32) | **m** | **必需** | 径向距离（雷达传感器坐标系） |
| 5 | `azimuth` | 4 | F (float32) | **rad** | **必需** | 方位角（雷达传感器坐标系） |
| 6 | `elevation` | 4 | F (float32) | **rad** | **必需** | 俯仰角（雷达传感器坐标系） |
| 7 | `RCS` | 4 | F (float32) | **dBsm** | **必需** | 雷达散射截面积。物理值域 [-100, 129.37] |
| 8 | `SNR` | 4 | F (float32) | **dB** | 可选 | 信噪比。物理值域 [0, 63.75] |
| 9 | `ambgt` | 4 | F (float32) | **m/s** | **必需** | 速度模糊窗宽（非模糊速度区间宽度），典型值 3-15 ，就是FFT下多普勒的最大不模糊度范围，上限减下限|
| 10 | `exist_prob` | 1 | U (uint8) | [0,255] | **必需** | 存在概率，0-100，阈值: ≥30 保留，255=无效；如果没有，可以写成100 |
| 11 | `multi_tgt_prob` | 1 | U (uint8) | [0,255] | 可选 | 多目标概率，0-100，255=无效；如果没有，可以写成100 |
| 12 | `ambgt_prob` | 1 | U (uint8) | [0,255] | **必需** | 模糊概率，0-100，阈值: ≥40 保留，255=无效；如果没有，可以写成100 |
| 13 | `raw_doppler` | 4 | F (float32) | **m/s** | **必需** | 原始多普勒径向速度（雷达传感器坐标系，经硬件一次解模糊），即解模糊后的速度 |
| 14 | `idx` | 1 | U (uint8) | - | **必需** | 多普勒解模糊索引。如果是正的或0，idx=倍数+128；如果是负的，idx=192-倍数；255=无效(SNA=Spurious/Not Available，杂波/无效目标点)会被过滤；0=多普勒解模糊失败 |

**必需性说明:**
- **必需**: 缺少会导致过滤或解模糊失败
- **推荐**: 强烈建议提供，缺失有回退方案
- **可选**: 缺失字段自动填零，不影响核心功能

### 5.4 PCD Header 示例

```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z range azimuth elevation RCS SNR ambgt exist_prob multi_tgt_prob ambgt_prob raw_doppler idx
SIZE 4 4 4 4 4 4 4 4 4 1 1 1 4 1
TYPE F F F F F F F F F U U U F U
COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1
WIDTH 627
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 627
DATA ascii
```
WIDTH表示这一帧当中点云的总数
### 5.5 数据行示例

```
4.052 -0.17 0.677 0 0.423884 0.094428 -36.0 14.75 21.82 10 100 77 -0.04 128
```

对应字段解读：

| 字段 | 值 | 含义 |
|------|-----|------|
| x | 4.052 | 前方 4.052m |
| y | -0.17 | 右侧 0.17m |
| z | 0.677 | 上方 0.677m |
| range | 0 | 径向距离 0m（此例为近场标定点） |
| azimuth | 0.4239 | 方位角 ≈24.3° |
| elevation | 0.0944 | 俯仰角 ≈5.4° |
| RCS | -36.0 | 雷达截面积 -36 dBsm |
| SNR | 14.75 | 信噪比 14.75 dB |
| ambgt | 21.82 | 速度模糊窗宽 21.82 m/s |
| exist_prob | 10 | 存在概率 10（< 30 会被过滤） |
| multi_tgt_prob | 100 | 多目标概率 100 |
| ambgt_prob | 77 | 模糊概率 77（≥ 40 保留） |
| raw_doppler | -0.04 | 多普勒速度 -0.04 m/s |
| idx | 128 | 多普勒索引 128（有效值） |

### 5.6 关键过滤规则

以下条件的点会被剔除（数据准备时可预先清洗，也可交由程序过滤）：

| 过滤器 | 条件 | 默认阈值 |
|--------|------|----------|
| ROI | x/y/z 超出包围盒 | x∈[-80,200], y∈[-40,40], z∈[-15,15] m |
| 高度 | `\|z\| >= \|x\| × 0.1 + 2` | 自适应 |
| RCS | 10m 内: RCS ≤ -40; 10m 外: RCS ≤ -20 | dBsm |
| 存在概率 | `exist_prob < 30` | - |
| idx | `idx == 255` (SNA) | - |
| 模糊概率 | `ambgt_prob < 40` | - |

---

## 6. 雷达点云 CSV 文件

### 6.1 文件格式

标准 CSV，首行为列名 header，逗号分隔，无引号。

### 6.2 文件命名

文件名（不含 `.csv` 后缀）为该帧的时间戳，单位**微秒 (μs)**。

```
<timestamp_us>.csv
```

例如: `1083029893.csv` 表示时间戳 1083029893 μs。

> 如果点云内部 `timestamp` 字段有效（>0），优先使用内部时间戳；否则回退到文件名。

### 6.3 字段定义

| # | 字段名 | SIZE | TYPE | 单位 | 必需性 | 描述 |
|---|--------|------|------|------|:------:|------|
| 1 | `x` | 4 | F (float32) | **m** | **必需** | 目标纵向坐标（车辆系，前方为正） |
| 2 | `y` | 4 | F (float32) | **m** | **必需** | 目标横向坐标（车辆系，左侧为正） |
| 3 | `z` | 4 | F (float32) | **m** | **必需** | 目标垂直坐标（车辆系，上方为正） |
| 4 | `range` | 4 | F (float32) | **m** | **必需** | 径向距离（雷达传感器坐标系） |
| 5 | `azimuth` | 4 | F (float32) | **rad** | **必需** | 方位角（雷达传感器坐标系） |
| 6 | `elevation` | 4 | F (float32) | **rad** | **必需** | 俯仰角（雷达传感器坐标系） |
| 7 | `RCS` | 4 | F (float32) | **dBsm** | **必需** | 雷达散射截面积。物理值域 [-100, 129.37] |
| 8 | `SNR` | 4 | F (float32) | **dB** | 可选 | 信噪比。物理值域 [0, 63.75] |
| 9 | `ambgt` | 4 | F (float32) | **m/s** | **必需** | 速度模糊窗宽（非模糊速度区间宽度），典型值 3-15 ，就是FFT下多普勒的最大不模糊度范围，上限减下限|
| 10 | `exist_prob` | 1 | U (uint8) | [0,255] | **必需** | 存在概率，0-100，阈值: ≥30 保留，255=无效；如果没有，可以写成100 |
| 11 | `multi_tgt_prob` | 1 | U (uint8) | [0,255] | 可选 | 多目标概率，0-100，255=无效；如果没有，可以写成100 |
| 12 | `ambgt_prob` | 1 | U (uint8) | [0,255] | **必需** | 模糊概率，0-100，阈值: ≥40 保留，255=无效；如果没有，可以写成100 |
| 13 | `raw_doppler` | 4 | F (float32) | **m/s** | **必需** | 原始多普勒径向速度（雷达传感器坐标系，经硬件一次解模糊），即解模糊后的速度 |
| 14 | `idx` | 1 | U (uint8) | - | **必需** | 多普勒解模糊索引。如果是正的或0，idx=倍数+128；如果是负的，idx=192-倍数；255=无效(SNA=Spurious/Not Available，杂波/无效目标点)会被过滤；0=多普勒解模糊失败 |

**必需性说明:**
- **必需**: 缺少会导致过滤或解模糊失败
- **推荐**: 强烈建议提供，缺失有回退方案
- **可选**: 缺失字段自动填零，不影响核心功能

### 6.4 CSV 示例

```
x y z range azimuth elevation RCS SNR ambgt exist_prob multi_tgt_prob ambgt_prob raw_doppler idx
4.052 -0.17 0.677 0 0.423884 0.094428 -36.0 14.75 21.82 10 100 77 -0.04 128
4.052 -0.17 0.677 0 0.423884 0.094428 -36.0 14.75 21.82 10 100 77 -0.04 128
```

对应字段解读：

| 字段 | 值 | 含义 |
|------|-----|------|
| x | 4.052 | 前方 4.052m |
| y | -0.17 | 右侧 0.17m |
| z | 0.677 | 上方 0.677m |
| range | 0 | 径向距离 0m（此例为近场标定点） |
| azimuth | 0.4239 | 方位角 ≈24.3° |
| elevation | 0.0944 | 俯仰角 ≈5.4° |
| RCS | -36.0 | 雷达截面积 -36 dBsm |
| SNR | 14.75 | 信噪比 14.75 dB |
| ambgt | 21.82 | 速度模糊窗宽 21.82 m/s |
| exist_prob | 10 | 存在概率 10（< 30 会被过滤） |
| multi_tgt_prob | 100 | 多目标概率 100 |
| ambgt_prob | 77 | 模糊概率 77（≥ 40 保留） |
| raw_doppler | -0.04 | 多普勒速度 -0.04 m/s |
| idx | 128 | 多普勒索引 128（有效值） |

### 6.5 关键过滤规则

以下条件的点会被剔除（数据准备时可预先清洗，也可交由程序过滤）：

| 过滤器 | 条件 | 默认阈值 |
|--------|------|----------|
| ROI | x/y/z 超出包围盒 | x∈[-80,200], y∈[-40,40], z∈[-15,15] m |
| 高度 | `\|z\| >= \|x\| × 0.1 + 2` | 自适应 |
| RCS | 10m 内: RCS ≤ -40; 10m 外: RCS ≤ -20 | dBsm |
| 存在概率 | `exist_prob < 30` | - |
| idx | `idx == 255` (SNA) | - |
| 模糊概率 | `ambgt_prob < 40` | - |

---

## 7. 雷达目标 CSV 文件

### 7.1 文件格式

标准 CSV，首行为列名 header，逗号分隔，无引号。

### 7.2 文件命名

文件名（不含 `.csv` 后缀）为该帧的时间戳，单位**微秒 (μs)**。

```
<timestamp_us>.csv
```

例如: `1083029893.csv` 表示时间戳 1083029893 μs。

> 如果雷达目标内部 `timestamp` 字段有效（>0），优先使用内部时间戳；否则回退到文件名。

### 7.3 字段定义

| # | 字段名 | SIZE | TYPE | 单位 | 必需性 | 描述 |
|---|--------|------|------|------|:------:|------|
| 1 | `object_id` | 8 | F (uint64_t) | - | **必需** | 目标物的跟踪id |
| 2 | `tracked_times` | 8 | F (uint64_t) | - | **必需** | 目标物的跟踪帧数 |
| 3 | `score` | 4 | F (float32) | - | **必需** | 得分，置信度，0-1 （障碍物概率）|
| 4 | `x` | 4 | F (float32) | **m** | **必需** | 目标物距离，车辆坐标系，bbox的中心点 |
| 5 | `y` | 4 | F (float32) | **m** | **必需** | 目标物距离，车辆坐标系，bbox的中心点 |
| 6 | `z` | 4 | F (float32) | **m** | **必需** | 目标物距离，车辆坐标系，bbox的中心点，没有可以置0 |
| 7 | `l` | 4 | F (float32) | **m** | **必需** | 目标物bbox的长度，没有可以置0  |
| 8 | `w` | 4 | F (float32) | **m** | **必需** | 目标物bbox的宽度，没有可以置0  |
| 9 | `h` | 4 | F (float32) | **m** | **必需** | 目标物bbox的高度，没有可以置0  |
| 10 | `yaw` | 4 | F (float32) | **rad** | **必需** | 目标物的航向角，车辆坐标系 |
| 11 | `vx_absolute` | 4 | F (float32) | **m/s** | **必需**  | 目标物的对地速度，车辆坐标系 |
| 12 | `vy_absolute` | 4 | F (float32) | **m/s** | **必需** | 目标物的对地速度，车辆坐标系 |
| 13 | `vz_absolute` | 4 | F (float32) | **m/s** | **必需** | 目标物的对地速度，车辆坐标系，没有可以置0  |
| 14 | `moving_state` | 1 | U (uint8) | **enum** | **必需** | 0：moving, 1: stationary, 2: oncoming, 3: cross, 4: stopped, 255: unknown |

**必需性说明:**
- **必需**: 缺少会导致过滤或解模糊失败
- **推荐**: 强烈建议提供，缺失有回退方案
- **可选**: 缺失字段自动填零，不影响核心功能

### 7.4 CSV 示例

```
object_id tracked_times score x y z l w h yaw vx_absolute vy_absolute vz_absolute moving_state
9 20 0.85 10.523464 0.423884 0 6.0 4.75 0 2.548623 15.783946 0.387693 0 0
9 20 0.85 10.523464 0.423884 0 6.0 4.75 0 2.548623 15.783946 0.387693 0 0
```

对应字段解读：

| 字段 | 值 | 含义 |
|------|-----|------|
| object_id | 9 | 目标物的跟踪id: 9 |
| tracked_times | 20 | 目标物的跟踪帧数: 20 |
| score | 0.85 | 障碍物概率: 85% |
| x | 10.523464 | 目标物距离-x: 10.523464 m |
| y | 0.423884 | 目标物距离-y: 0.423884 m |
| z | 0 | 目标物距离-z: 0 m |
| l | 6.0 | 目标物bbox的长度: 6.0 m |
| w | 4.75 | 目标物bbox的宽度：4.75 m |
| h | 0 | 目标物bbox的高度：0 m |
| yaw | 2.548623 | 目标物的航向角: 2.548623 rad |
| vx_absolute | 15.783946 | 目标物的对地速-x: 15.783946 m/s |
| vy_absolute | 0.387693 | 目标物的对地速-y: 0.387693 m/s  |
| vz_absolute | 0 | 目标物的对地速-z: 0 m/s |
| moving_state | 0 | moving_state：moving |

---

## 8. 摄像头图片文件

### 8.1 文件格式

标准 jpg 图片。

### 8.2 文件命名

文件名（不含 `.jpg` 后缀）为该帧的时间戳，单位**微秒 (μs)**。

```
<timestamp_us>.jpg
```

例如: `1083029893.jpg` 表示时间戳 1083029893 μs。

### 8.3 时间频率要求

- **推荐频率:** ≥30 Hz（即间隔 ≤33ms）

---

## 9. 坐标系定义

### 9.1 车辆坐标系 (Vehicle Frame / Body Frame)

```
        X (前方)
        ↑
        |
        |
Y ←─────● (后轴中心接地点)
        |
        |
        Z: 垂直向上 (右手坐标系)
```

| 轴 | 方向 | 正值含义 |
|----|------|---------|
| X | 车辆正前方 | 前方 |
| Y | 车辆左侧 | 左侧 |
| Z | 垂直向上 | 上方 |
| 原点 | 后轴中心接地点 | - |

### 9.2 Local 坐标系 (开机坐标系)

- **原点:** 车辆上电时的后轴中心位置
- **轴方向:** 与上电时刻的车辆坐标系一致
- **用途:** 位置 (`pos_local_x/y/z`) 和四元数 (`quaternion_local`) 相对此系表示

### 9.3 雷达传感器坐标系

各雷达有独立的传感器坐标系，通过标定外参 (`r_s2b`, `t_s2b`) 与车辆系关联。

- `azimuth`、`elevation`、`range`、`raw_doppler` 均在各自的雷达传感器坐标系下
- `x`、`y`、`z` 已通过标定转到车辆坐标系

### 9.4 四元数约定 (`quaternion_local` 详解)

#### 物理含义

`quaternion_local` 是一个单位四元数 `(x, y, z, w)`，表示**从车辆坐标系 (Vehicle Frame) 到开机坐标系 (Local Frame) 的旋转变换**，即 **R_vehicle→local**：

```
p_local = q × p_vehicle × q⁻¹
```

或等价地用旋转矩阵表示：

```
R = quaternion_local.toRotationMatrix()    // 3×3 矩阵
p_local = R × p_vehicle
```

#### 直觉理解

想象车辆在行驶过程中逐渐转弯：

```
                     开机位置 (Local 原点)
                     ●──→ X_local
                     |
                     ↓ Y_local


                              当前位置
                              ●──→ X_vehicle (车头方向，已偏转 θ)
                              |
                              ↓ Y_vehicle
```

- **开机时刻:** 车辆系与 Local 系完全重合，此时 `quaternion_local = (0, 0, 0, 1)` (单位四元数，无旋转)
- **行驶过程中:** 车辆转了 yaw 角 θ，`quaternion_local` 编码了这个累积旋转
- **含义:** "如果我有一个在当前车辆系下的向量，乘以这个四元数就能得到它在开机系下的表达"

#### 数学约定 (Eigen Hamilton)

```
q = w + xi + yj + zk

构造函数: Eigen::Quaterniond(w, x, y, z)  // 注意: w 在前!
内存布局: [x, y, z, w]                     // 内存中 x 在前
归一化:   x² + y² + z² + w² = 1
```

#### 纯旋转示例

| 运动场景           | quaternion (x, y, z, w) | 说明                   |
|--------------------|-------------------------|------------------------|
| 开机瞬间（未动）   | (0, 0, 0, 1)           | 无旋转，两系重合       |
| 左转 90° (yaw)     | (0, 0, 0.707, 0.707)   | 绕 Z 轴转 π/2         |
| 右转 30° (yaw)     | (0, 0, -0.259, 0.966)  | 绕 Z 轴转 -π/6        |
| 上坡 5° (pitch)    | (0, 0.0436, 0, 0.999)  | 绕 Y 轴转 5°          |
| 任意行驶           | (x, y, z, w)           | 累积的 roll+pitch+yaw  |

**纯 Yaw 旋转公式:** 车辆绕 Z 轴旋转 θ 角（左转为正）：

```
quaternion_local_x = 0
quaternion_local_y = 0
quaternion_local_z = sin(θ/2)
quaternion_local_w = cos(θ/2)
```

#### 正逆变换对照表

| 表达式                                              | 含义                 | 用途                     |
|-----------------------------------------------------|----------------------|--------------------------|
| `quaternion_local`                                  | R_vehicle→local      | 将车辆系向量转到 Local 系 |
| `quaternion_local.conjugate()`                      | R_local→vehicle      | 将 Local 系向量转到车辆系 |
| `quaternion_local.toRotationMatrix()`               | 3×3 旋转矩阵 R_v→l  | 矩阵形式的正变换          |
| `quaternion_local.toRotationMatrix().transpose()`   | R_l→v               | 逆变换矩阵               |

#### 在 Tracker 中的使用方式

**1. 速度坐标系变换（ROS 版本）：**

```cpp
// velocity_local: Local 系下的自车速度
// 需要转到 Vehicle 系（tracker 内部统一用 Vehicle 系速度）
// quaternion_local.conjugate() = R_local→vehicle

Eigen::Vector3d veh_vel = quaternion_local.conjugate() * velocity_local;
```

> **注意:** CSV 版本中 `linear_vel_x/y/z` 已经是 Vehicle 系，无需此变换。

**2. 帧间相对变换（运动补偿）：**

```cpp
R_a = pose_a.rotation.toRotationMatrix();  // R_vehicle_a → local
R_b = pose_b.rotation.toRotationMatrix();  // R_vehicle_b → local

// 从时刻 a 的车辆系 → 时刻 b 的车辆系
T_a2b.rotation    = R_b^T × R_a;           // R_vehicle_a → vehicle_b
T_a2b.translation = R_b^T × (pos_a - pos_b);
```

**3. 自车速度投影到雷达系（Doppler 解模糊）：**

```cpp
// radar2car.linear() = R_sensor→vehicle
// transpose()        = R_vehicle→sensor
R_vehicle2sensor = radar2car.linear().transpose();
ego_vel_sensor = R_vehicle2sensor * ego_linear_velocity;
```

#### 从实际数据验证

CSV 中的一行：

```
quaternion_local_x = -0.00127607
quaternion_local_y =  0.016321
quaternion_local_z =  0.000420713
quaternion_local_w =  0.999866
```

解读：
- `w ≈ 1.0` → 旋转角度很小（刚开机，车辆几乎未动）
- `y ≈ 0.0163` → 主旋转分量在 Y 轴，pitch ≈ 2×arcsin(0.0163) ≈ 1.87°
- `x ≈ -0.0013` → 微量 roll ≈ -0.15°
- `z ≈ 0.0004` → 微量 yaw ≈ 0.05°

验证：对应同行的 `euler_local_pitch = 0.0326 rad ≈ 1.87°` ✓

---
