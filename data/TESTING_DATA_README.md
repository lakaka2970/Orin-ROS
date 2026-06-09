# 测试数据说明

模拟数据文件，严格遵循 `参考/FT_radar_dataset_requirement.md` 定义的格式。

## 文件清单

| 文件 | 格式 | 说明 |
|------|------|------|
| `sample_ego_motion.csv` | CSV | 自车运动数据，7 字段，10 帧 @ 50Hz 间隔 |
| `sample_det_list.csv` | CSV | 雷达检测列表，14 字段，10 个检测点 |
| `sample_obj_list.csv` | CSV | 3D 目标列表，14 字段，5 个目标 |
| `sample_radar_pointcloud.pcd` | PCD v0.7 | 雷达点云，14 字段，10 个点，ASCII 格式 |
| `calibration/radar_front_center_ft.yaml` | YAML | 前方中央雷达标定文件 |

## 数据类型对照

### ego_motion.csv（自车运动）

| 字段 | 类型 | 单位 |
|------|------|------|
| `timestamp_us` | uint64 | μs |
| `vx` | double | m/s |
| `yaw_rate` | double | rad/s |
| `steering_angle` | double | rad |
| `ax` | double | m/s² |
| `ay` | double | m/s² |
| `gear` | uint8 | — |

### det_list.csv / PCD（检测列表）

| 字段 | 类型 | 单位 |
|------|------|------|
| `x, y, z` | float32 | m (车辆系) |
| `range` | float32 | m (雷达系) |
| `azimuth` | float32 | rad |
| `elevation` | float32 | rad |
| `RCS` | float32 | dBsm |
| `SNR` | float32 | dB |
| `ambgt` | float32 | m/s |
| `exist_prob` | uint8 | [0,255] |
| `multi_tgt_prob` | uint8 | [0,255] |
| `ambgt_prob` | uint8 | [0,255] |
| `raw_doppler` | float32 | m/s |
| `idx` | uint8 | — |

### obj_list.csv（3D 目标列表）

| 字段 | 类型 | 单位 |
|------|------|------|
| `object_id` | uint64 | — |
| `tracked_times` | uint64 | — |
| `score` | float32 | [0,1] |
| `x, y, z` | float32 | m |
| `l, w, h` | float32 | m |
| `yaw` | float32 | rad |
| `vx/vy/vz_absolute` | float32 | m/s |
| `moving_state` | uint8 | enum |

## 使用方式

这些数据文件可用于：

1. **格式验证** — 确保 Logging 节点输出的 CSV 格式与样本一致
2. **离线回放** — 后续开发的 CSV Replay 节点可直接读取这些文件并发布到 ROS2 话题
3. **数据导入测试** — 验证读取端（如数据查看工具）能正确解析 14 字段格式
4. **标定加载** — Logging 节点可通过参数 `calibration_file` 指定标定文件路径
