#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 相机数据接收节点 (Camera Rx)
================================================================================
模拟通过 v4l2 接口从相机硬件采集视频帧，发布为 sensor_msgs/Image。

发布话题：
  /ft/video_raw   Image   原始视频帧

连接关系：
  → Rviz_Image (sub)
  → Logging (sub)

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 相机参数 ----------
CAMERA_FPS   = 15.0         # 相机帧率 (Hz)
IMAGE_WIDTH  = 1280         # 图像宽度 (px)
IMAGE_HEIGHT = 720          # 图像高度 (px)

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

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


# ============================================================================
# ROS2 节点
# ============================================================================

class CameraRxNode(Node):
    """
    相机数据接收节点 —— 模拟 v4l2 采集，发布 Image

    发布话题：
      /ft/video_raw   Image   原始视频帧
    """

    def __init__(self):
        super().__init__('camera_rx')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('camera_fps',   CAMERA_FPS)
        self.declare_parameter('image_width',  IMAGE_WIDTH)
        self.declare_parameter('image_height', IMAGE_HEIGHT)
        self.declare_parameter('fixed_frame',  FIXED_FRAME)

        self.camera_fps   = float(self.get_parameter('camera_fps').value)
        self.image_width  = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.fixed_frame  = self.get_parameter('fixed_frame').value

        # ---------- 静态 TF：camera → radar ----------
        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
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
        self.pub_img = self.create_publisher(Image, '/ft/video_raw', 10)

        # ---------- 定时器 ----------
        period = 1.0 / self.camera_fps
        self.timer = self.create_timer(period, self._on_timer)
        self.frame_count = 0

        self.get_logger().info(
            f'Camera Rx 启动: {self.camera_fps:.0f} Hz, '
            f'{self.image_width}x{self.image_height}')

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        """生成模拟测试图案并发布"""
        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()

        w, h = self.image_width, self.image_height

        # 绘制测试图案：渐变色背景 + 移动圆 + 帧号文字
        img = np.zeros((h, w, 3), dtype=np.uint8)

        # 渐变背景（模拟不同光照条件）
        for y in range(h):
            color_val = int(128 + 64 * np.sin(y * 0.01 + self.frame_count * 0.1))
            img[y, :] = [color_val, 200 - color_val // 2, 128]

        # 移动的圆（模拟场景中的动态物体）
        cx = int(w * 0.5 + 200 * np.sin(self.frame_count * 0.05))
        cy = int(h * 0.5 + 100 * np.cos(self.frame_count * 0.07))
        cv2.circle(img, (cx, cy), 40, (0, 255, 255), 2)

        # 十字准星（模拟标定参考）
        cv2.line(img, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (0, 255, 0), 1)
        cv2.line(img, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (0, 255, 0), 1)

        # 帧号文字
        cv2.putText(img, f'Frame: {self.frame_count}',
                    (30, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (255, 255, 255), 2)
        cv2.putText(img, 'Camera Rx (simulated v4l2)',
                    (30, h - 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (200, 200, 200), 1)

        # 发布图像
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.frame_id = self.fixed_frame
        img_msg.header.stamp = stamp
        self.pub_img.publish(img_msg)
        self.get_logger().debug(
            f'Camera Rx 帧 #{self.frame_count}: 发布 {w}x{h} 图像')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Camera Rx 已停止')
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
