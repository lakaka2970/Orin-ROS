# ============================================================================
# 雷达检测目标点消息 (v2 — 41 字段完整对齐)
# ============================================================================
# 坐标系定义:
#   车辆系: X=前方, Y=左侧, Z=上方, 原点=后轴中心接地点
#   雷达系: azimuth/elevation/range/rad_vel_abs 在雷达传感器坐标系下
# ============================================================================

# ---- 空间位置（车辆坐标系） ----
float32 x                     # 4  f32XPos         目标 X 坐标（纵向，单位 m）
float32 y                     # 5  f32YPos         目标 Y 坐标（横向，单位 m）
float32 z                     # 6  f32ZPos         目标 Z 坐标（高度，单位 m）

# ---- 雷达原视测量（雷达传感器坐标系） ----
float32 rad_vel_abs           # 7  f32RadVelAbs    绝对径向速度（m/s）
float32 range                 # 8  f32Range        目标径向距离（m）
float32 speed                 # 9  f32Speed        目标合速度（m/s）
float32 azimuth_ang           # 10 f32AzimuthAng   方位角（单位：弧度）
float32 ele_ang               # 11 f32EleAng       俯仰角（单位：弧度）
uint8   idx                   # 多普勒解模糊索引,
                              # idx >= 0: idx = 倍数 + 128
                              # idx 负值: idx = 192 - 倍数
                              # idx = 255: SNA(杂波/无效点) → 过滤
                              # idx = 0: 多普勒解模糊失败
# ---- 信号特征 ----
float32 snr_db                # 12 f32SNRdB        信噪比（dB）
float32 rcs_db                # 13 f32RcsdB        雷达散射截面积 RCS（dBsm）
float32 power_db              # 14 f32PowerdB      回波信号功率（dB）

# ---- 检测标志位 ----
uint32 strategy_flag          # 15 u32StrategyFlag   检测策略标志位
uint32 obj_same_rv            # 16 u32ObjSameRV     同距离同速度目标标记
uint32 obj_quality            # 17 u32ObjQuality    目标质量等级
uint32 obj_track_flag         # 18 u32ObjTrackFlag  目标跟踪关联标志
uint32 ele_confident          # 19 u32EleConfident  俯仰角估计置信度
uint32 predict_det_flag       # 20 u32PredictDetflag 预测域检测标志

# ---- DOA 与角度关联 ----
uint32 doa_method             # 25 u32DOAMethod         DOA 波达方向估计算法标识
uint32 asso_angle_filter_id   # 26 u32AssoAngleFilterId 关联角度滤波器 ID

# ---- RD 索引 ----
uint16 rd_cell_idx            # 27 u16RdCellIdx     距离多普勒 RD 单元总索引
uint16 range_idx              # 28 u16RangeIdx      距离维索引
uint16 doppler_idx            # 29 u16DopplerIdx    多普勒速度维索引
uint8  azimuth_idx            # 30 u8AzimuthIdx     方位角角度索引
uint8  elevation_idx          # 31 u8ElevationIdx   俯仰角角度索引

# ---- 峰值与 SNR ----
uint16 peak_val               # 32 u16PeakVal         RD 谱峰值强度
uint16 sin_azim_snr_lin       # 33 u16SinAzimSNRLin   方位 sin 域线性信噪比
uint16 sin_elev_snr_lin       # 34 u16SinElevSNRLin   俯仰 sin 域线性信噪比

# ---- 速度解模糊 ----
int8   vel_amb_fac            # 35 s8VelAmbFac       速度解模糊因子

# ---- 枚举状态 ----
uint8  det_ambig_state        # 36 eDetAmbigState    目标速度模糊状态枚举
uint8  det_motion_pat         # 37 eDetMotionPat     目标运动模式枚举（静止 / 运动）

# ---- 置信度与帧间标志 ----
uint8  det_conf               # 38 u8DetConf          检测点基础置信度
uint8  inter_frame_flag       # 39 u8InterFrameFlag   帧间处理标志
uint8  asso_trk_num           # 40 u8AssoTrkNum       关联跟踪目标数量
uint8  chanl_phase_max        # 41 u8ChanlPhaseMax    天线通道最大相位差
