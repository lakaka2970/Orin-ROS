#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达信号处理节点 —— Python/MIL 实现 (R SP MIL Python)
================================================================================
融合 ADC 数据和车辆数据，执行雷达信号处理（模拟实现），
解决速度模糊问题，输出检测目标列表。

订阅话题：
  /ft/adc_data       PointCloud2    雷达 ADC 原始点云
  /ft/vehicle_data   TwistStamped   车辆动态数据

发布话题：
  /ft/det_list_py    PointCloud2    检测目标列表 (x, y, z, velocity, snr)

连接关系：
  ← ADC Rx (sub)
  ← Vehicle Data Rx (sub)
  → Rviz_radar (pub)
  → 3D Object Detection (pub)
  → Logging (pub)

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 处理参数 ----------
PROCESSING_FPS = 10.0        # 处理帧率 (Hz)
SNR_THRESHOLD  = 10.0        # 信噪比阈值 (dB)，低于此值的点被过滤
VELOCITY_SCALE = 0.5         # 速度估算缩放因子

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TwistStamped


# ============================================================================
# 工具函数
# ============================================================================

def create_header(frame_id: str, stamp) -> Header:
    """创建 ROS2 消息头"""
    h = Header()
    h.frame_id = frame_id
    h.stamp = stamp
    return h


def create_det_list_pointcloud2(points: np.ndarray, frame_id: str,
                                 stamp) -> PointCloud2:
    """
    创建检测列表 PointCloud2。

    points: [N, 5] — (x, y, z, velocity_mps, snr_db)
    """
    N = len(points)

    fields = [
        PointField(name='x',        offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',        offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',        offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='snr',      offset=16, datatype=PointField.FLOAT32, count=1),
    ]

    cloud = points.astype(np.float32)

    msg = PointCloud2()
    msg.header = create_header(frame_id, stamp)
    msg.height = 1
    msg.width = N
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 20
    msg.row_step = 20 * N
    msg.is_dense = True
    msg.data = cloud.tobytes() if N > 0 else b''
    return msg


# ============================================================================
# ROS2 节点
# ============================================================================

class RspMilPythonNode(Node):
    """
    Python 雷达信号处理节点

    订阅话题：
      /ft/adc_data       PointCloud2
      /ft/vehicle_data   TwistStamped

    发布话题：
      /ft/det_list_py    PointCloud2
    """

    def __init__(self):
        super().__init__('rsp_mil_python')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('processing_fps', PROCESSING_FPS)
        self.declare_parameter('snr_threshold',  SNR_THRESHOLD)
        self.declare_parameter('velocity_scale', VELOCITY_SCALE)
        self.declare_parameter('fixed_frame',    FIXED_FRAME)

        self.processing_fps = float(self.get_parameter('processing_fps').value)
        self.snr_threshold  = float(self.get_parameter('snr_threshold').value)
        self.velocity_scale = float(self.get_parameter('velocity_scale').value)
        self.fixed_frame    = self.get_parameter('fixed_frame').value

        # ---------- 数据缓存 ----------
        self._latest_adc = None
        self._latest_vehicle = None

        # ---------- 订阅者 ----------
        self.sub_adc = self.create_subscription(
            PointCloud2, '/ft/adc_data', self._on_adc, 10)
        self.sub_vehicle = self.create_subscription(
            TwistStamped, '/ft/vehicle_data', self._on_vehicle, 10)

        # ---------- 发布者 ----------
        self.pub_det = self.create_publisher(PointCloud2, '/ft/det_list_py', 10)

        # ---------- 处理定时器 ----------
        period = 1.0 / self.processing_fps
        self.timer = self.create_timer(period, self._on_process)
        self.frame_count = 0

        self.get_logger().info(
            f'R SP MIL Python 启动: {self.processing_fps:.0f} Hz, '
            f'SNR 阈值={self.snr_threshold} dB')

    # ------------------------------------------------------------------
    # 数据回调
    # ------------------------------------------------------------------

    def _on_adc(self, msg: PointCloud2):
        """接收 ADC 原始数据"""
        self._latest_adc = msg

    def _on_vehicle(self, msg: TwistStamped):
        """接收车辆动态数据"""
        self._latest_vehicle = msg

    # ------------------------------------------------------------------
    # 处理回调
    # ------------------------------------------------------------------

    def _on_process(self):
        """
        模拟雷达信号处理 pipeline：
        1. 读取缓存的 ADC 数据
        2. SNR 滤波（去噪）
        3. 车辆速度补偿（解决速度模糊）
        4. 发布检测目标列表
        """
        if self._latest_adc is None:
            return

        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()

        # 解析 ADC PointCloud2 → numpy
        adc_data = np.frombuffer(self._latest_adc.data, dtype=np.float32)
        if len(adc_data) == 0:
            return
        adc_data = adc_data.reshape(-1, 4)      # [x, y, z, intensity]

        # 获取车辆速度用于补偿
        ego_vx = 0.0
        ego_vy = 0.0
        if self._latest_vehicle is not None:
            ego_vx = self._latest_vehicle.twist.linear.x
            ego_vy = self._latest_vehicle.twist.linear.y

        # 提取各字段
        x = adc_data[:, 0]
        y = adc_data[:, 1]
        z = adc_data[:, 2]
        intensity = adc_data[:, 3]

        # ---- SNR 滤波 ----
        mask = intensity >= self.snr_threshold
        x, y, z = x[mask], y[mask], z[mask]
        snr = intensity[mask]

        if len(x) == 0:
            self.get_logger().debug(
                f'[RSP-PY] 帧 #{self.frame_count}: 所有点低于 SNR 阈值，跳过')
            return

        # ---- 模拟速度估算（含速度补偿） ----
        distances = np.sqrt(x ** 2 + y ** 2 + z ** 2)
        # 径向速度 ≈ 距离变化率 - 车辆速度分量（简化模型）
        radial_velocity = self.velocity_scale * (distances * 0.1 - ego_vx)

        # 构造检测列表 [N, 5]: (x, y, z, velocity, snr)
        det_points = np.column_stack([x, y, z, radial_velocity, snr])

        msg = create_det_list_pointcloud2(det_points, self.fixed_frame, stamp)
        self.pub_det.publish(msg)

        self.get_logger().info(
            f'[RSP-PY] 帧 #{self.frame_count}: '
            f'输入 {len(adc_data)} 点 → SNR 过滤后 {len(det_points)} 点 → '
            f'车速补偿 ego_vx={ego_vx:.1f} m/s → 发布 Det List')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('R SP MIL Python 已停止')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = RspMilPythonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('R SP MIL Python 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
