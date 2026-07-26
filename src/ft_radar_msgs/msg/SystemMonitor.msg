# ============================================================================
# 系统监控消息 (V2 架构)
# ============================================================================
# system_monitor 节点以 1 Hz 发布, 涵盖磁盘/内存/进程/时钟/帧周期.
# ============================================================================

std_msgs/Header header

# ── 磁盘 I/O ──
float64 disk_read_rate        # 磁盘读取速率 (MB/s)
float64 disk_write_rate       # 磁盘写入速率 (MB/s)
float64 disk_utilization      # 磁盘利用率 (%)

# ── 内存 ──
uint64 memory_total           # 总内存 (MB)
uint64 memory_used            # 已用内存 (MB)
uint64 memory_available       # 可用内存 (MB)
float64 memory_percent        # 内存使用率 (%)

# ── 进程 ──
uint32 process_count          # 监控进程数
string[] process_names        # 进程名称列表
float64[] process_cpu_percents    # 进程 CPU 使用率 (%)
float64[] process_memory_percents # 进程内存使用率 (%)

# ── 时钟 ──
float64 clock_drift_us        # ROS Time vs System Time 漂移 (微秒)

# ── 帧周期统计 ──
float64 adc_frame_period_mean     # ADC 帧周期均值 (ms)
float64 adc_frame_period_std      # ADC 帧周期标准差 (ms)
uint32 adc_frame_drop_count       # ADC 累计丢帧数
float64 camera_frame_period_mean  # Camera 帧周期均值 (ms)
float64 camera_frame_period_std   # Camera 帧周期标准差 (ms)
uint32 camera_frame_drop_count    # Camera 累计丢帧数
