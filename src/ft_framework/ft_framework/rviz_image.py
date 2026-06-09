#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 图像 RViz 可视化节点 (Rviz_Image)
================================================================================
接收相机原始图像帧，添加叠加信息后发布。

话题:
  订阅: /camera/image_raw          sensor_msgs/Image
  发布: /visualization/camera/display  sensor_msgs/Image

连接关系:
  ← Camera Rx (sub)

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

SHOW_OVERLAY = True          # 是否显示叠加信息（帧号、时间戳等）
FIXED_FRAME  = 'camera'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


class RvizImageNode(Node):
    """图像可视化节点"""

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

        self.sub_video = self.create_subscription(
            Image, '/camera/image_raw', self._on_video, 10)
        self.pub_display = self.create_publisher(
            Image, '/visualization/camera/display', 10)

        self.frame_count = 0
        self.get_logger().info('Rviz_Image 启动')

    def _on_video(self, msg: Image):
        self.frame_count += 1

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        if self.show_overlay:
            h, w = img.shape[:2]
            overlay = img.copy()
            cv2.rectangle(overlay, (0, 0), (w, 45), (0, 0, 0), -1)
            img = cv2.addWeighted(img, 0.7, overlay, 0.3, 0)

            timestamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            cv2.putText(
                img, f'Frame: {self.frame_count} | Time: {timestamp_s:.3f}s',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(
                img, 'Camera View (Rviz_Image)',
                (w - 320, h - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (200, 200, 200), 1)

        display_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        display_msg.header = msg.header
        self.pub_display.publish(display_msg)
        self.get_logger().debug(f'Rviz_Image 帧 #{self.frame_count}: 发布显示图像')

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
