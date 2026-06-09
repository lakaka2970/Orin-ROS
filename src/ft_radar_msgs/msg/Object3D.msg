# ============================================================================
# 3D 目标消息
# ============================================================================
# 与 FT_radar_dataset_requirement.md 第 7 节雷达目标 CSV 字段完全对齐，共 14 个字段。
#
# moving_state 枚举:
#   0: moving      — 运动
#   1: stationary  — 静止
#   2: oncoming    — 迎面
#   3: cross       — 横穿
#   4: stopped     — 停止
#   255: unknown   — 未知
# ============================================================================

# ---- 跟踪信息 ----
uint64  object_id             # 目标跟踪 ID
uint64  tracked_times         # 跟踪帧数

# ---- 检测置信度 ----
float32 score                 # 置信度 / 障碍物概率 [0, 1]

# ---- 3D 包围盒（车辆坐标系, 前左上, 后轴中心接地点） ----
float32 x                     # bbox 中心 X (m), 前方为正
float32 y                     # bbox 中心 Y (m), 左侧为正
float32 z                     # bbox 中心 Z (m), 上方为正, 无可置 0
float32 l                     # bbox 长度 (m), 无可置 0
float32 w                     # bbox 宽度 (m), 无可置 0
float32 h                     # bbox 高度 (m), 无可置 0
float32 yaw                   # 航向角 (rad), 车辆坐标系

# ---- 运动状态（对地速度, 车辆坐标系） ----
float32 vx_absolute           # 对地速度 X (m/s), 车辆系
float32 vy_absolute           # 对地速度 Y (m/s), 车辆系
float32 vz_absolute           # 对地速度 Z (m/s), 车辆系, 无可置 0

# ---- 运动分类 ----
uint8   moving_state          # 运动状态枚举:
                              #   0: moving
                              #   1: stationary
                              #   2: oncoming
                              #   3: cross
                              #   4: stopped
                              #   255: unknown
