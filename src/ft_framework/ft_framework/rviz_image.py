#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 图像 RViz 可视化节点 (Rviz_Image) — V2 架构
================================================================================
V2 变更: 订阅 /camera/file_path (CameraFilePath), 从文件读取 JPEG 图像.
         消除对 /camera/image_raw (sensor_msgs/Image) 的依赖.

话题:
  订阅: /camera/file_path              ft_radar_msgs/CameraFilePath
  发布: /visualization/camera/display  sensor_msgs/Image

作者: zhengyuan.liu
日期: 2026-07-26 (V2)
================================================================================
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ft_radar_msgs.msg import CameraFilePath

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

SHOW_OVERLAY = True
FIXED_FRAME  = 'camera'


class RvizImageNode(Node):

    def __init__(self):
        super().__init__('rviz_image')

        self.declare_parameter('show_overlay', SHOW_OVERLAY)
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.show_overlay = bool(self.get_parameter('show_overlay').value)
        self.fixed_frame  = self.get_parameter('fixed_frame').value

        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().error('cv_bridge 未安装，Rviz_Image 无法工作！')
            raise RuntimeError('cv_bridge is required')

        # V2: 订阅 CameraFilePath (文件路径消息)
        qos = rclpy.qos.QoSProfile(
            depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            CameraFilePath, '/camera/file_path', self._on_file_path, qos)
        self.pub_display = self.create_publisher(
            Image, '/visualization/camera/display', 10)

        self.frame_count = 0
        self.get_logger().info('Rviz_Image V2 启动 (订阅 /camera/file_path)')

    def _on_file_path(self, msg: CameraFilePath):
        if not msg.file_ready or not msg.file_path:
            return

        self.frame_count += 1

        # 从文件读取 JPEG 图像
        img = cv2.imread(msg.file_path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f'图像读取失败: {msg.file_path}')
            return

        if self.show_overlay:
            h, w = img.shape[:2]
            overlay = img.copy()
            cv2.rectangle(overlay, (0, 0), (w, 45), (0, 0, 0), -1)
            img = cv2.addWeighted(img, 0.7, overlay, 0.3, 0)

            ts_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
            cv2.putText(
                img, f'Frame: {self.frame_count} | TS: {ts_us} us',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        display_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        display_msg.header = msg.header
        self.pub_display.publish(display_msg)

    def destroy_node(self):
        self.get_logger().info('Rviz_Image 已停止')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RvizImageNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'Rviz_Image 启动失败: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
