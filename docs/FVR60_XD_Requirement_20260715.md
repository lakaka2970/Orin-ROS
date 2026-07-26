# FVR60_XD Requirement

## Radar

1. 波形调整
- 最远距离，从350m改为320m
- rangebin远端丢弃，从5%改为10%
- 保持4倍跳频
- HPF，从300KHz改为1200KHz

2. 跳频补偿码验证与应用
- 暗箱验证，发现导致doppler peak扩展，IFX认为是RTS导致的
- 场地动目标验证，doppler peak是否与理论一致，即peak收窄

3. EBD应用（用于解速度模糊）
- 增加EBD ramp，填入idle time index (0~35)
- 验证513 ramp是否可以被ORIN接收，期望ramp维依然可以保持512 ramp

4. 级联性能评估
- 评估master slave级联的通道一致性，是否需要启用IFX建议的所有校准措施
- IFX建议的校准包括：
	- 初始化Tx/Rx校准
	- 温度变化10℃进行Tx/Rx校准
	- Master/Slave 相位校准（基于IFX产线数据）
	- Master/Slave 重叠通道相位校准
	- 所有虚拟通道0°校准（幅度、相位）


## ORIN

1. Ubuntu/ROS2版本升级 -- Pending
- JetPack 5.1.2 (L4T R35.4.1)，Ubuntu 20.04
- Ubuntu 22.04 and higher version may have other unexpected issue
- ROS Foxy，原生适配Ubuntu 20.04，ROS Humble需要重新编译，以匹配Ubuntu 20.04

2. 数采功能logging
- Logging的数据包括：ADC, Ego, Video(Image), RX NCI, RD Cell List, Det List, Obj List
- Logging分为四种模式：
	- ADC Mode: ADC, Ego, Video(Image), Det List, Obj List
	- RD Cell List Mode: Ego, Video(Image), RX NCI, RD Cell List, Det List, Obj List
	- Det List Mode: Ego, Video(Image), Det List, Obj List
	- Idle Mode: None
- ADC logging，只有在ADC Mode时，在ADC Rx Node进程中，将ADC数据从DDR写入到eMMC磁盘，
  进程的CPU Loading需要很低，避免多余的数据copy，在一二十ms内完成每帧bin文件生成，
  timestamp使用global timestamp，需要打在每帧ADC MIPI接收完成的时刻
- RX NCI & RD Cell List logging，只有在RD Cell List Mode时，在RSP Node进程中，
  将RX NCI & RD Cell List写入到eMMC磁盘，进程的CPU Loading需要很低，避免多余数据copy，
  在几ms内完成每帧数据文件生成（如果csv文件无法降低耗时，考虑使用其他文件格式，如bin文件），
  timestamp沿用ADC的timestamp
- Video(Image) logging，使用非常小的图片尺寸，需要搞明白V4L2接收到JPEG图片数据后，为啥还进行了encoding,
  目前使用硬件加速器完成了encoding，进程的CPU Loading需要很低，避免多余的数据copy，
  在几ms内完成每帧图片文件生成，timestamp使用global timestamp，需要打在每帧图片接收完成的时刻，
  录制完成后，将image合并为一个avi视频文件
- Ego logging，接收多个CAN报文更新Ego数据buffer，每20ms将最新的Ego Buffer数据写入到csv文件中，
  写入时间需要非常短，1ms以内，timestamp使用global timestamp，需要打在每20ms发布的时刻
- 不允许存在丢帧，且所有数据的timestamp在同一个时间区间内，单位是us
- 根据磁盘存储的空余大小，限制录取最大帧数

3. RSP实时处理
- 66ms内完成一帧的点云处理，可以限制远距离处理，保证66ms处理时长
- RSP的ADC输入，需要从DDR直接获取，且DDR上需要设置队列，增加鲁棒性
- RSP处理主体在GPU上运行，CPU尽量降低loading

4. RVIZ可视化
- 支持ORIN可视化点云、目标和图片，CPU Loading需要很低


5. ETH通信（奔驰协议）
- 发送点云（最大4096）
- 接收车身信息
- 时间同步
- 点云数据转换
- Ego数据转换
- 验证ORIN万兆ETH与MB千兆ETH连接


6. CAN接收车身信息（FT测试车通信矩阵）
- 接收车身信息


7. ROS节点重构
- ROS节点包括：ADC Rx, Camera Rx, Vehicle Data Rx, RSP, Rviz, ETH
- ROS节点分为以下3种模式：
	- FT Debug Mode (with logging cfg): ADC Rx, Camera Rx, Vehicle Data Rx, RSP, Rviz
	- MB Debug Mode (with logging cfg): ADC Rx, Camera Rx, ETH, RSP, Rviz
	- MB Running Mode: ADC Rx, ETH, RSP
