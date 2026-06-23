#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 相机数据接收节点 (Camera Rx)
================================================================================
模拟通过 v4l2 接口从相机硬件采集视频帧，发布为 sensor_msgs/Image。

规格:
  - 帧率: 30 Hz
  - 数据格式: TBD（当前为占位模拟实现）
  - 时间戳: 全局统一，微秒 (μs) 精度

话题:
  发布: /camera/image_raw    sensor_msgs/Image

连接关系:
  → Rviz_Image (sub)
  → Logging (sub)

预留参数:
  - line: 当前为占位值 0，待确认
  - nof_line: 当前为占位值 0，待确认

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 相机参数 ----------
CAMERA_FPS      = 30        # 帧率 (Hz)
IMAGE_WIDTH     = 4         # 图像宽度 (px)  — 空数据模式, 不占用硬盘空间
IMAGE_HEIGHT    = 4         # 图像高度 (px)  — 空数据模式, 不占用硬盘空间

# ---------- 预留参数 ----------
LINE            = 0         # 待确认，预留接口
NOF_LINE        = 0         # 待确认，预留接口

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'camera'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import tf2_ros
from geometry_msgs.msg import TransformStamped

from ft_framework.common import monotonic_us_stamp

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


# ============================================================================
# ROS2 节点
# ============================================================================

class CameraRxNode(Node):
    """
    相机数据接收节点

    发布话题:
      /camera/image_raw    sensor_msgs/Image

    功能说明:
      - 模拟 v4l2 采集相机图像帧
      - 图像格式为占位实现（测试图案），具体相机格式待确认
      - 注入全局统一时间戳（微秒精度）
    """

    def __init__(self):
        super().__init__('camera_rx')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('fps', CAMERA_FPS)
        self.declare_parameter('image_width', IMAGE_WIDTH)
        self.declare_parameter('image_height', IMAGE_HEIGHT)
        self.declare_parameter('line', LINE)
        self.declare_parameter('nof_line', NOF_LINE)
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.fps          = float(self.get_parameter('fps').value)
        self.image_width  = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.line         = int(self.get_parameter('line').value)
        self.nof_line     = int(self.get_parameter('nof_line').value)
        self.fixed_frame  = self.get_parameter('fixed_frame').value

        self.get_logger().info(
            f'Camera Rx: line={self.line}, nof_line={self.nof_line} (预留参数, 待确认)')

        # ---------- 静态 TF：camera → radar ----------
        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        sec, nsec = monotonic_us_stamp()
        tf_msg.header.stamp.sec = sec
        tf_msg.header.stamp.nanosec = nsec
        tf_msg.header.frame_id = 'radar'
        tf_msg.child_frame_id = self.fixed_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 1.2       # 相机安装高度
        tf_msg.transform.rotation.w = 1.0
        self._tf_static.sendTransform(tf_msg)

        # ---------- cv_bridge ----------
        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().error('cv_bridge 未安装，Camera Rx 无法工作！')
            raise RuntimeError('cv_bridge is required')

        # ---------- 发布者 ----------
        self.pub_img = self.create_publisher(Image, '/camera/image_raw', 10)

        # ---------- 定时器 ----------
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self._on_timer)
        self.frame_count = 0

        self.get_logger().info(
            f'Camera Rx 启动: {self.fps:.0f} Hz, '
            f'{self.image_width}x{self.image_height}')

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        """
        发布极小空图像以维持节点拓扑, 不生成测试图案, 不占用硬盘空间.
        4×4×3 = 48 bytes/frame, 30 Hz ≈ 1.4 KB/s.
        """
        self.frame_count += 1
        sec, nsec = monotonic_us_stamp()

        # 极小空图像
        img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        # 发布图像
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.frame_id = self.fixed_frame
        img_msg.header.stamp.sec = sec
        img_msg.header.stamp.nanosec = nsec
        self.pub_img.publish(img_msg)
        self.get_logger().debug(
            f'Camera Rx 帧 #{self.frame_count}: timestamp={sec}.{nsec:09d}')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info(f'Camera Rx 已停止（共处理 {self.frame_count} 帧）')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraRxNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'Camera Rx 启动失败: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
