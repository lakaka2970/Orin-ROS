# ============================================================================
# ADC 文件路径消息 (V2 架构)
# ============================================================================
# 替代 AdcRawData.msg: rx 节点同步写入文件后发布路径,
# DDS 带宽从 480 MB/s 降至 ~20 KB/s (降低 99.99%).
#
# 时间戳: header.stamp = V4L2 硬件时间戳 (CLOCK_MONOTONIC, 微秒精度)
#         打在每帧 ADC MIPI 接收完成时刻, 下游透传禁止覆盖.
# ============================================================================

std_msgs/Header header

string file_path              # ADC 文件绝对路径 (e.g. /mnt/nvme/adc_data/1626789000000.bin)
uint64 file_size              # 文件大小 (bytes), 典型值 33554432 (32 MiB)
uint32 num_rows               # 总 chirp 数, 典型值 1024 (512 chirps × 2 groups)
uint32 num_chirps_per_row     # RX 天线数, 典型值 8 (ctrx0:4 + ctrx1:4)
uint32 num_samples_per_chirp  # 每 chirp 采样点数, 典型值 2048
bool file_ready               # 文件是否完全写入 (fsync 完成)
