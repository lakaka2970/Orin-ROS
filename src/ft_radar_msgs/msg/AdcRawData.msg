# ============================================================================
# 雷达 ADC 原始数据消息
# ============================================================================
# 数据维度: [num_rows × num_chirps_per_row × num_samples_per_chirp] × int16
# 典型配置: 512 rows × 16 chirps/row × 2048 samples/chirp × 2 bytes = 32 MB/帧
# 帧率: 15 Hz
# 总吞吐: ~480 MB/s
#
# 时间戳要求:
#   header.stamp 必须在 v4l2 读取数据的第一时间注入，
#   使用 CLOCK_MONOTONIC 或硬件 PTP 时钟，精度为微秒 (μs)。
#   后续所有节点和 Logging 以此时间戳为准，不得覆盖。
# ============================================================================

std_msgs/Header header                # 全局统一时间戳（微秒精度，header.stamp 透传）

uint32 num_rows                       # 行数，典型值 512
uint32 num_chirps_per_row             # 每行的 chirp 数量，典型值 16
uint32 num_samples_per_chirp          # 每个 chirp 的采样点数，典型值 2048

int16[] data                          # ADC 采样数据，原始 int16 数组
                                      # 一维展开顺序: [row][chirp_per_row][sample]
                                      # 总长度: num_rows × num_chirps_per_row × num_samples_per_chirp
                                      # 典型值: 512 × 16 × 2048 = 16,777,216 个采样点
