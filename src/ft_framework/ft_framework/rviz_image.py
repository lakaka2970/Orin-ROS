#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 图像 RViz 可视化节点 (Rviz_Image)
================================================================================
接收相机原始视频帧，添加叠加信息后发布为 RViz 可视化图像。

订阅话题：
  /ft/video_raw   Image   原始视频帧

发布话题：
  /ft/video_display   Image   带叠加信息的显示图像

连接关系：
  ← Camera Rx (sub)

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 显示参数 ----------
SHOW_OVERLAY = True          # 是否显示叠加信息（帧号、时间戳等）

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'camera'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


# ============================================================================
# ROS2 节点
# ============================================================================

class RvizImageNode(Node):
    """
    图像可视化节点 —— 接收原始视频，叠加信息后发布

    订阅话题：
      /ft/video_raw   Image

    发布话题：
      /ft/video_display   Image
    """

    def __init__(self):
        super().__init__('rviz_image')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('show_overlay', SHOW_OVERLAY)
        self.declare_parameter('fixed_frame',  FIXED_FRAME)

        self.show_overlay = bool(self.get_parameter('show_overlay').value)
        self.fixed_frame  = self.get_parameter('fixed_frame').value

        # ---------- cv_bridge ----------
        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().error('cv_bridge 未安装，Rviz_Image 无法工作！')
            raise RuntimeError('cv_bridge is required')

        # ---------- 订阅者 ----------
        self.sub_video = self.create_subscription(
            Image, '/ft/video_raw', self._on_video, 10)

        # ---------- 发布者 ----------
        self.pub_display = self.create_publisher(
            Image, '/ft/video_display', 10)

        self.frame_count = 0
        self.get_logger().info('Rviz_Image 启动')

    # ------------------------------------------------------------------
    # 视频回调
    # ------------------------------------------------------------------

    def _on_video(self, msg: Image):
        """接收原始视频帧，叠加信息后转发"""
        self.frame_count += 1

        # 转换 ROS Image → OpenCV
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        if self.show_overlay:
            h, w = img.shape[:2]

            # 半透明顶部信息栏
            overlay = img.copy()
            cv2.rectangle(overlay, (0, 0), (w, 45), (0, 0, 0), -1)
            img = cv2.addWeighted(img, 0.7, overlay, 0.3, 0)

            # 帧号和时间戳
            timestamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            cv2.putText(img,
                        f'Frame: {self.frame_count} | Time: {timestamp_s:.3f}s',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)

            # 底部来源标识
            cv2.putText(img, 'Camera View (Rviz_Image)',
                        (w - 320, h - 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (200, 200, 200), 1)

        # 转换 OpenCV → ROS Image 并发布
        display_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        display_msg.header = msg.header
        self.pub_display.publish(display_msg)
        self.get_logger().debug(
            f'Rviz_Image 帧 #{self.frame_count}: 发布显示图像')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Rviz_Image 已停止')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

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
