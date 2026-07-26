# ============================================================================
# Camera 文件路径消息 (V2 架构)
# ============================================================================
# camera_rx 节点将 MJPEG 帧直接写入 .jpg 文件后发布路径.
# 消除 OpenCV decode+encode 开销, MJPEG 直接写盘.
#
# 时间戳: header.stamp = V4L2 硬件时间戳 (CLOCK_MONOTONIC, 微秒精度)
#         打在每帧图片接收完成时刻.
# ============================================================================

std_msgs/Header header

string file_path              # 文件绝对路径 (e.g. /mnt/nvme/camera_front_center/1626789000000.jpg)
uint64 file_size              # 文件大小 (bytes)
uint32 width                  # 图像宽度 (px), 典型值 320
uint32 height                 # 图像高度 (px), 典型值 240
string encoding               # 编码格式: "jpeg" | "raw"
bool file_ready               # 文件是否完全写入
