# ============================================================================
# RSP 中间数据: RD Cell List + Rx NCI
# ============================================================================
# 对标 FT FVR60_XD Radar Dataset Requirement:
#   §7 雷达RdCellList — 每帧 CSV (本消息提供全部字段)
#   §8 雷达RX_NCI    — 每帧 BIN (本消息提供原始 float32 数据)
#
# 数据量: ~5 MiB/帧, 10 Hz → ~50 MiB/s
# ============================================================================

std_msgs/Header header                # 透传 ADC 时间戳
uint16 frame_id                       # 雷达帧序号
uint32 frame_timestamp_us             # 帧时间戳 (微秒)
uint8  idle_time_idx                  # 空闲时间索引 (固定 0)

# ---- RD Cell List (§7.3 字段 3-12, 并行数组) ----
uint16 num_cells                      # u16NofRdCell: 当前帧 RD Cell 数量
uint16[] rb_list                      # u16Rb:  距离 bin 索引
uint16[] db_list                      # u16Db:  多普勒 bin 索引

# f32PowRbNci_Q7dB[3] — 距离向 NCI 功率 (3 通道) -> pow_rb_nci_0/1/2
float32[] pow_rb_nci_0               # [num_cells] ch0
float32[] pow_rb_nci_1               # [num_cells] ch1
float32[] pow_rb_nci_2               # [num_cells] ch2

# f32PowDbNci_Q7dB[3] — 多普勒向 NCI 功率 (3 通道) -> pow_db_nci_0/1/2
float32[] pow_db_nci_0               # [num_cells] ch0
float32[] pow_db_nci_1               # [num_cells] ch1
float32[] pow_db_nci_2               # [num_cells] ch2

float32[] peak_power_list             # f32PeakPowVchNci_Q7dB: 峰值功率
float32[] noise_power_list            # f32NoiseNci_Q7dB: 噪声功率
uint8[]   valid_flag_list             # u8RdValidFlag: RD 有效性标志
uint8[]   peak_flag_list              # u8RdPeakFlag:  RD 峰值标志

# sVch[256] — 通道复数数据 (complex int32, 256 对)
# 存储为 int32 real + int32 imag 交错, 共 num_cells × 256 × 2 × 4 字节
uint8[] channel_data_bytes            # 复数 int32 对, 按 cell 顺序排列

# ---- Rx NCI (§8 BIN 文件数据) ----
uint16 rx_nci_rows                    # 多普勒维大小 (典型 512)
uint16 rx_nci_cols                    # 距离维大小   (典型 1025)
uint8[] rx_nci_data                   # float32 原始字节, 长度 = rows * cols * 4
