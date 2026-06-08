#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达 ADC 数据接收节点 (ADC Rx)
================================================================================
模拟通过 v4l2 接口从雷达硬件采集原始 ADC 数据，发布为 PointCloud2。

发布话题：
  /ft/adc_data   PointCloud2   雷达原始点云数据（range, azimuth, elevation, intensity）

连接关系：
  → R SP MIL Python (sub)
  → R SP Cuda (sub)
  → Logging (sub)

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 模拟雷达参数 ----------
RADAR_FPS       = 10.0       # 雷达帧率 (Hz)
NUM_TARGETS     = 50         # 每帧模拟目标数
RANGE_MAX       = 300.0      # 最大探测距离 (m)
RANGE_MIN       = 1.0        # 最小探测距离 (m)
AZIMUTH_RANGE   = 90.0       # 方位角范围 (±°，度)
ELEVATION_RANGE = 15.0       # 俯仰角范围 (±°，度)

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
import tf2_ros
from geometry_msgs.msg import TransformStamped


# ============================================================================
# 工具函数
# ============================================================================

def create_header(frame_id: str, stamp) -> Header:
    """创建 ROS2 消息头"""
    h = Header()
    h.frame_id = frame_id
    h.stamp = stamp
    return h


def create_pointcloud2_radar(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """
    创建雷达 ADC 数据的 PointCloud2。

    points: [N, 4] — (range_m, azimuth_rad, elevation_rad, intensity_db)
    存储为 x=range*cos(el)*cos(az), y=range*cos(el)*sin(az), z=range*sin(el)
    """
    N = len(points)

    ranges     = points[:, 0]
    azimuths   = points[:, 1]
    elevations = points[:, 2]
    intensities = points[:, 3]

    # 球坐标 → 笛卡尔坐标
    x = ranges * np.cos(elevations) * np.cos(azimuths)
    y = ranges * np.cos(elevations) * np.sin(azimuths)
    z = ranges * np.sin(elevations)

    fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    cloud = np.column_stack([x, y, z, intensities]).astype(np.float32)

    msg = PointCloud2()
    msg.header = create_header(frame_id, stamp)
    msg.height = 1
    msg.width = N
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * N
    msg.is_dense = True
    msg.data = cloud.tobytes() if N > 0 else b''
    return msg


# ============================================================================
# ROS2 节点
# ============================================================================

class AdcRxNode(Node):
    """
    雷达 ADC 数据接收节点 —— 模拟 v4l2 采集，发布 PointCloud2

    发布话题：
      /ft/adc_data   PointCloud2   雷达原始点云数据
    """

    def __init__(self):
        super().__init__('adc_rx')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('radar_fps',       RADAR_FPS)
        self.declare_parameter('num_targets',     NUM_TARGETS)
        self.declare_parameter('range_max',       RANGE_MAX)
        self.declare_parameter('range_min',       RANGE_MIN)
        self.declare_parameter('azimuth_range',   AZIMUTH_RANGE)
        self.declare_parameter('elevation_range', ELEVATION_RANGE)
        self.declare_parameter('fixed_frame',     FIXED_FRAME)

        self.radar_fps       = float(self.get_parameter('radar_fps').value)
        self.num_targets     = int(self.get_parameter('num_targets').value)
        self.range_max       = float(self.get_parameter('range_max').value)
        self.range_min       = float(self.get_parameter('range_min').value)
        self.azimuth_range   = float(self.get_parameter('azimuth_range').value)
        self.elevation_range = float(self.get_parameter('elevation_range').value)
        self.fixed_frame     = self.get_parameter('fixed_frame').value

        # ---------- 静态 TF：radar → map ----------
        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = self.fixed_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.5       # 雷达安装高度
        tf_msg.transform.rotation.w = 1.0
        self._tf_static.sendTransform(tf_msg)

        # ---------- 发布者 ----------
        self.pub_adc = self.create_publisher(PointCloud2, '/ft/adc_data', 10)

        # ---------- 定时器 ----------
        period = 1.0 / self.radar_fps
        self.timer = self.create_timer(period, self._on_timer)
        self.frame_count = 0

        self.get_logger().info(
            f'ADC Rx 启动: {self.radar_fps:.0f} Hz, '
            f'{self.num_targets} 目标/帧, '
            f'距离 [{self.range_min}, {self.range_max}] m')

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        """定时生成模拟雷达点云数据并发布"""
        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()

        # 生成随机雷达目标（球坐标）
        n = self.num_targets
        ranges = np.random.uniform(self.range_min, self.range_max, n)
        azimuths = np.random.uniform(
            -np.radians(self.azimuth_range),
            np.radians(self.azimuth_range), n)
        elevations = np.random.uniform(
            -np.radians(self.elevation_range),
            np.radians(self.elevation_range), n)
        # 强度模拟：近目标信号强，远目标信号弱，叠加高斯噪声
        intensities = 20.0 * np.log10(self.range_max / (ranges + 1e-3))
        intensities += np.random.normal(0, 3, n)
        intensities = np.clip(intensities, 0.0, 80.0)

        points = np.column_stack([ranges, azimuths, elevations, intensities])

        msg = create_pointcloud2_radar(points, self.fixed_frame, stamp)
        self.pub_adc.publish(msg)
        self.get_logger().debug(
            f'ADC Rx 帧 #{self.frame_count}: 发布 {n} 个 ADC 数据点')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('ADC Rx 已停止')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = AdcRxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ADC Rx 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
