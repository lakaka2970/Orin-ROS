# ============================================================================
# 雷达 ADC 原始数据消息
# ============================================================================
# 数据维度: [num_rows × num_chirps_per_row × num_samples_per_chirp] × int16
# 典型配置 (16T16R ctrx0+ctrx1 合并): 1024 chirps × 8 RX × 2048 samples × 2 bytes = 32 MiB/帧
# 帧率: 15 Hz
# 总吞吐: ~480 MiB/s
#
# 时间戳要求:
#   header.stamp 必须在 v4l2 读取数据的第一时间注入，
#   使用 CLOCK_MONOTONIC 或硬件 PTP 时钟，精度为微秒 (μs)。
#   后续所有节点和 Logging 以此时间戳为准，不得覆盖。
# ============================================================================

std_msgs/Header header                # 全局统一时间戳（微秒精度，header.stamp 透传）

uint32 num_rows                       # 总 chirp 数，典型值 1024 (512 chirps × 2 groups)
uint32 num_chirps_per_row             # RX 天线数，典型值 8 (ctrx0:4 + ctrx1:4)
uint32 num_samples_per_chirp          # 每个 chirp 的采样点数，典型值 2048

uint8[] data                          # ADC 采样数据，原始字节流 (little-endian int16)
                                      # 语义: int16[chirps×rx×sample]; 传输用 uint8 走 memcpy 快速路径
                                      # 消费者: np.frombuffer(msg.data, dtype=np.int16).reshape(...)
                                      # 总字节: num_rows × num_chirps_per_row × num_samples_per_chirp × 2
                                      # 典型值: 1024 × 8 × 2048 × 2 = 33,554,432 字节 (32 MiB)
