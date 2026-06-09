# ============================================================================
# 自车运动数据消息
# ============================================================================
# 与 FT_radar_dataset_requirement.md 第 3 节 Egomotion CSV 字段完全对齐，共 7 个数据字段。
#
# 时间戳: header.stamp 中的时间戳单位为微秒 (μs), 与 CSV 中 timestamp_us 对应。
# 坐标系: Local 坐标系（车辆上电时的后轴中心位置）
#
# 默认值机制:
#   当车辆总线数据丢失或超时时，节点自动切换为默认值，
#   并将 is_default 标志置为 true，供下游节点识别。
# ============================================================================

std_msgs/Header header        # 全局统一时间戳（微秒精度）
                              # header.stamp = timestamp_us
                              # header.frame_id = 'base_link'

float64 vx                    # 车速 (m/s), Local 系, 必需
float64 yaw_rate              # 偏航角速度 (rad/s), Local 系, 必需
float64 steering_angle        # 转向角度 (rad), Local 系, 必需
float64 ax                    # 纵向加速度 (m/s²), Local 系, 必需
float64 ay                    # 横向加速度 (m/s²), Local 系, 必需
uint8   gear                  # 挡位: 0=R, 1=D, 2=N, 3=P, 必需

bool    is_default            # 是否为默认值:
                              #   false = 来自总线/传感器的有效数据
                              #   true  = 因数据丢失/超时而使用的默认值
