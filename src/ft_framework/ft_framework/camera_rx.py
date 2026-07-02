#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 相机数据接收节点 (Camera Rx)
================================================================================
通过 V4L2 接口从 USB UVC 摄像头 (Rmoncam A2 1080P) 采集视频帧，
发布为 sensor_msgs/Image。

规格:
  - 设备: /dev/video0 (UVC, 可通过 device_path 参数配置)
  - 编码: MJPEG → BGR8 (OpenCV 自动解码)
  - 最高: 1920×1080 @ 30 fps (MJPEG)
  - 时间戳: 全局统一，微秒 (μs) 精度

话题:
  发布: /camera/image_raw    sensor_msgs/Image

连接关系:
  → Rviz_Image (sub)
  → Logging (sub)

作者: zhengyuan.liu
日期: 2026.6.8
更新: 2026.7.1 — 接入真实 V4L2 摄像头 (Rmoncam A2 1080P)
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 相机参数 ----------
CAMERA_FPS      = 30            # 帧率 (Hz)
IMAGE_WIDTH     = 1920          # 图像宽度 (px)
IMAGE_HEIGHT    = 1080          # 图像高度 (px)
DEVICE_PATH     = "/dev/video0" # V4L2 设备路径
PIXEL_FORMAT    = "MJPG"        # 像素格式: MJPG (30fps@1080p) | YUYV (5fps@1080p)

# ---------- 预留参数 ----------
LINE            = 0             # 待确认，预留接口
NOF_LINE        = 0             # 待确认，预留接口

# ---------- 重连参数 ----------
RECONNECT_INTERVAL_FRAMES = 90  # 断线后每隔 N 帧尝试重连 (3s @ 30fps)

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'camera'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import numpy as np
import threading
import time

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
# V4L2 相机驱动
# ============================================================================

class V4l2Camera:
    """通过 OpenCV V4L2 后端管理 USB UVC 摄像头。

    特性:
      - 支持 MJPEG / YUYV 像素格式
      - 断线自动重连 (按帧间隔)
      - 读取失败时返回 None，调用方自行降级处理
    """

    def __init__(self, device_path: str, width: int, height: int,
                 fps: float, pixel_format: str = "MJPG"):
        self.device_path  = device_path
        self.width        = width
        self.height       = height
        self.fps          = fps
        self.pixel_format = pixel_format
        self.cap = None
        self._fourcc_code = self._make_fourcc(pixel_format)

    @staticmethod
    def _make_fourcc(fmt: str) -> int:
        """将字符串像素格式转换为 OpenCV FOURCC 码。"""
        if len(fmt) == 4:
            return cv2.VideoWriter_fourcc(fmt[0], fmt[1], fmt[2], fmt[3])
        return cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')

    def open(self) -> bool:
        """打开 V4L2 设备并配置格式/分辨率/帧率。成功返回 True。"""
        self.close()

        self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            return False

        # 配置顺序: FOURCC → 分辨率 → 帧率
        self.cap.set(cv2.CAP_PROP_FOURCC, self._fourcc_code)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # 读取实际生效的参数
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = ''.join(chr((actual_fourcc >> i*8) & 0xFF) for i in range(4)) \
            if actual_fourcc else 'unknown'

        self.log_info = (
            f'V4L2 已连接: {self.device_path} '
            f'{actual_w}x{actual_h} {fourcc_str} @ {actual_fps:.0f} fps'
        )
        return True

    def read(self):
        """读取一帧。成功返回 (True, frame)，失败返回 (False, None)。"""
        if self.cap is None or not self.cap.isOpened():
            return False, None
        return self.cap.read()

    def is_opened(self) -> bool:
        return self.cap is not None and self.cap.isOpened()

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __del__(self):
        self.close()


# ============================================================================
# ROS2 节点
# ============================================================================

class CameraRxNode(Node):
    """
    相机数据接收节点 — V4L2 真实摄像头采集

    发布话题:
      /camera/image_raw    sensor_msgs/Image

    功能说明:
      - 通过 V4L2 (OpenCV 后端) 从 USB UVC 摄像头采集图像帧
      - MJPEG → BGR8 自动解码
      - 断线自动重连，重连期间发布最后一个有效帧 (或空图像)
      - 注入全局统一时间戳（微秒精度）
    """

    def __init__(self):
        super().__init__('camera_rx')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('fps', CAMERA_FPS)
        self.declare_parameter('image_width', IMAGE_WIDTH)
        self.declare_parameter('image_height', IMAGE_HEIGHT)
        self.declare_parameter('device_path', DEVICE_PATH)
        self.declare_parameter('pixel_format', PIXEL_FORMAT)
        self.declare_parameter('line', LINE)
        self.declare_parameter('nof_line', NOF_LINE)
        self.declare_parameter('fixed_frame', FIXED_FRAME)
        self.declare_parameter('reconnect_interval_frames',
                               RECONNECT_INTERVAL_FRAMES)

        self.fps            = float(self.get_parameter('fps').value)
        self.image_width    = int(self.get_parameter('image_width').value)
        self.image_height   = int(self.get_parameter('image_height').value)
        self.device_path    = self.get_parameter('device_path').value
        self.pixel_format   = self.get_parameter('pixel_format').value
        self.line           = int(self.get_parameter('line').value)
        self.nof_line       = int(self.get_parameter('nof_line').value)
        self.fixed_frame    = self.get_parameter('fixed_frame').value
        self.reconnect_interval = int(
            self.get_parameter('reconnect_interval_frames').value)

        # ---------- V4L2 相机 ----------
        self.camera = V4l2Camera(
            device_path  = self.device_path,
            width        = self.image_width,
            height       = self.image_height,
            fps          = self.fps,
            pixel_format = self.pixel_format,
        )

        self._camera_opened = self.camera.open()
        if self._camera_opened:
            self.get_logger().info(self.camera.log_info)
        else:
            self.get_logger().warn(
                f'V4L2 设备打开失败: {self.device_path} — 将发布空图像占位')

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

        # ---------- 空图像降级 (相机未连接时使用) ----------
        self._empty_img = np.zeros(
            (self.image_height, self.image_width, 3), dtype=np.uint8)
        self._last_valid_img = self._empty_img

        # ---------- 发布者 ----------
        self.pub_img = self.create_publisher(Image, '/camera/image_raw', 10)

        # ---------- 轮询线程 (硬件驱动, 替代定时器) ----------
        self._stop_event = threading.Event()
        self._poll_thread = threading.Thread(target=self._polling_loop, daemon=True)
        self._poll_thread.start()
        self.frame_count = 0

        self.get_logger().info(
            f'Camera Rx 启动: {self.fps:.0f} Hz, '
            f'{self.image_width}x{self.image_height}, '
            f'设备={self.device_path}, 格式={self.pixel_format}')

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _polling_loop(self):
        """轮询线程: 从 V4L2 摄像头阻塞读取帧并立即发布.

        self.camera.read() 为阻塞调用, 自然限制循环频率为摄像头实际发送频率.
        断线处理: 每隔 reconnect_interval_frames 帧尝试重连一次.
        """
        while rclpy.ok() and not self._stop_event.is_set():
            self.frame_count += 1
            stamp = self.get_clock().now().to_msg()

            img = self._empty_img

            if self._camera_opened:
                ret, frame = self.camera.read()  # 阻塞读取, 相机实际帧率驱动
                if ret and frame is not None:
                    img = frame
                    self._last_valid_img = frame
                else:
                    # 读取失败 — 可能断线，尝试重连
                    if self.frame_count % self.reconnect_interval == 0:
                        self.get_logger().warn(
                            f'V4L2 读取失败 (第{self.frame_count}帧)，尝试重连...')
                        self._camera_opened = self.camera.open()
                        if self._camera_opened:
                            self.get_logger().info(
                                f'V4L2 重连成功: {self.camera.log_info}')
                    # 降级: 使用最后一帧有效图像
                    img = self._last_valid_img
                    # 读取失败时无硬件阻塞, 短暂休眠避免 CPU 100%
                    time.sleep(0.001)
            else:
                # 初始未连接 — 定期重试
                if self.frame_count % self.reconnect_interval == 0:
                    self._camera_opened = self.camera.open()
                    if self._camera_opened:
                        self.get_logger().info(
                            f'V4L2 连接成功: {self.camera.log_info}')
                # 相机未连接时无硬件阻塞, 短暂休眠避免 CPU 100%
                time.sleep(0.001)

            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header.frame_id = self.fixed_frame
            img_msg.header.stamp = stamp
            self.pub_img.publish(img_msg)

            if self.frame_count == 1:
                status = 'V4L2 真实摄像头' if self._camera_opened else '相机未连接 — 空图像占位'
                self.get_logger().info(f'Camera Rx: {status}')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._stop_event.set()
        self.camera.close()  # unblock any pending camera.read()
        if hasattr(self, '_poll_thread') and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=2.0)
        self.get_logger().info(
            f'Camera Rx 已停止（共处理 {self.frame_count} 帧）')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraRxNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'Camera Rx 启动失败: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
