#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达数据 RViz 可视化节点 (Rviz_radar)
================================================================================
汇聚雷达检测列表（Python版 + CUDA版）、3D目标列表和标尺数据，
发布可在 RViz 中显示的可视化消息。

订阅话题：
  /ft/det_list_py   PointCloud2    Python版检测目标列表
  /ft/det_list_cu   PointCloud2    CUDA版检测目标列表
  /ft/obj_list      MarkerArray    3D目标检测结果
  /ft/ruler         MarkerArray    标尺参考数据

发布话题：
  /ft/radar_display    PointCloud2    合并后的雷达点云（按高度着色）
  /ft/radar_boxes      MarkerArray    目标框 + 检测标记
  /ft/radar_colorbar   Image          高度色带
  /ft/radar_frame_info MarkerArray    帧信息文字

连接关系：
  ← R SP MIL Python (sub)
  ← R SP Cuda (sub)
  ← 3D Object Detection (sub)
  ← Rviz_Ruler (sub)

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 显示参数 ----------
MIN_Z           = -5.0       # 高度色带下限 (m)
MAX_Z           = 15.0       # 高度色带上限 (m)
MARKER_LIFETIME = 1.0        # Marker 生命周期 (s)
PUBLISH_HZ      = 10.0       # 可视化发布频率 (Hz)

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import PointCloud2, PointField, Image
from visualization_msgs.msg import Marker, MarkerArray

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


# ============================================================================
# 工具函数
# ============================================================================

def create_header(frame_id: str, stamp) -> Header:
    """创建 ROS2 消息头"""
    h = Header()
    h.frame_id = frame_id
    h.stamp = stamp
    return h


def jet_color_scalar(t: float):
    """
    t in [0, 1] → (r, g, b) in [0, 255]，jet 色图
    """
    t = float(np.clip(t, 0.0, 1.0))
    r = float(np.clip(1.5 - abs(4 * t - 3), 0, 1))
    g = float(np.clip(1.5 - abs(4 * t - 2), 0, 1))
    b = float(np.clip(1.5 - abs(4 * t - 1), 0, 1))
    return (int(r * 255), int(g * 255), int(b * 255))


def height_to_rgb_array(z_arr: np.ndarray, z_min: float,
                         z_max: float) -> np.ndarray:
    """
    高度数组 → RGB 颜色数组 [N, 3] uint8，向量化 jet 色图
    """
    t = (z_arr - z_min) / (z_max - z_min + 1e-6)
    t = np.clip(t, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def create_pointcloud2_rgb(points_xyz: np.ndarray, colors_rgb: np.ndarray,
                            frame_id: str, stamp) -> PointCloud2:
    """
    创建带 RGB 颜色的 PointCloud2，直接构造二进制数据。
    points_xyz: [N, 3] float32
    colors_rgb: [N, 3] uint8 (R, G, B)
    """
    N = len(points_xyz)

    fields = [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    msg = PointCloud2()
    msg.header = create_header(frame_id, stamp)
    msg.height = 1
    msg.width = N
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * N
    msg.is_dense = True

    if N == 0:
        msg.data = b''
        return msg

    # 向量化打包 RGB → uint32 → float32 (RViz packed RGB 格式)
    packed_rgb = (colors_rgb[:, 0].astype(np.uint32) << 16 |
                  colors_rgb[:, 1].astype(np.uint32) << 8 |
                  colors_rgb[:, 2].astype(np.uint32))
    rgb_f32 = packed_rgb.view(np.float32)

    cloud = np.column_stack([
        points_xyz[:, :3].astype(np.float32),
        rgb_f32.reshape(-1, 1)
    ]).astype(np.float32)
    msg.data = cloud.tobytes()
    return msg


def create_colorbar_image(z_min: float, z_max: float,
                           img_height: int = 300,
                           bar_width: int = 30,
                           label_width: int = 65) -> np.ndarray:
    """
    生成高度色带图像（BGR uint8），用于 RViz 中显示。
    """
    img = np.zeros((img_height, bar_width + label_width, 3), dtype=np.uint8)

    # 色带条
    for i in range(img_height):
        t = 1.0 - i / max(img_height - 1, 1)
        r, g, b = jet_color_scalar(t)
        img[i, :bar_width] = [b, g, r]

    # 刻度线和标签
    for i in range(6):
        t = i / 5.0
        z_val = z_min + t * (z_max - z_min)
        y = int((1.0 - t) * (img_height - 1))
        cv2.line(img, (bar_width, y), (bar_width + 6, y), (255, 255, 255), 1)
        cv2.putText(img, f'{z_val:.2f}m',
                    (bar_width + 8, min(y + 5, img_height - 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

    cv2.putText(img, 'Z(m)', (2, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    return img


# ============================================================================
# ROS2 节点
# ============================================================================

class RvizRadarNode(Node):
    """
    雷达数据 RViz 可视化节点

    订阅话题：
      /ft/det_list_py     PointCloud2
      /ft/det_list_cu     PointCloud2
      /ft/obj_list        MarkerArray
      /ft/ruler           MarkerArray

    发布话题：
      /ft/radar_display     PointCloud2
      /ft/radar_boxes       MarkerArray
      /ft/radar_colorbar    Image
      /ft/radar_frame_info  MarkerArray
    """

    def __init__(self):
        super().__init__('rviz_radar')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('min_z',           MIN_Z)
        self.declare_parameter('max_z',           MAX_Z)
        self.declare_parameter('marker_lifetime', MARKER_LIFETIME)
        self.declare_parameter('publish_hz',      PUBLISH_HZ)
        self.declare_parameter('fixed_frame',     FIXED_FRAME)

        self.min_z           = float(self.get_parameter('min_z').value)
        self.max_z           = float(self.get_parameter('max_z').value)
        self.marker_lifetime = float(self.get_parameter('marker_lifetime').value)
        self.publish_hz      = float(self.get_parameter('publish_hz').value)
        self.fixed_frame     = self.get_parameter('fixed_frame').value

        # ---------- 色带图像（仅生成一次） ----------
        self._colorbar_img = create_colorbar_image(self.min_z, self.max_z)

        # ---------- cv_bridge ----------
        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().warn('cv_bridge 未安装，色带将不发布')

        # ---------- 数据缓存 ----------
        self._latest_det_py = None
        self._latest_det_cu = None
        self._latest_obj    = None
        self._latest_ruler  = None

        # ---------- 订阅者（四个数据源） ----------
        self.sub_det_py = self.create_subscription(
            PointCloud2, '/ft/det_list_py', self._on_det_py, 10)
        self.sub_det_cu = self.create_subscription(
            PointCloud2, '/ft/det_list_cu', self._on_det_cu, 10)
        self.sub_obj = self.create_subscription(
            MarkerArray, '/ft/obj_list', self._on_obj, 10)
        self.sub_ruler = self.create_subscription(
            MarkerArray, '/ft/ruler', self._on_ruler, 10)

        # ---------- 发布者 ----------
        self.pub_points   = self.create_publisher(
            PointCloud2, '/ft/radar_display', 10)
        self.pub_boxes    = self.create_publisher(
            MarkerArray, '/ft/radar_boxes', 10)
        self.pub_colorbar = self.create_publisher(
            Image, '/ft/radar_colorbar', 10)
        self.pub_info     = self.create_publisher(
            MarkerArray, '/ft/radar_frame_info', 10)

        # ---------- 综合发布定时器 ----------
        period = 1.0 / self.publish_hz
        self.timer = self.create_timer(period, self._on_publish)
        self.frame_count = 0

        self.get_logger().info(
            f'Rviz_radar 启动: {self.publish_hz:.0f} Hz, '
            f'高度色带 [{self.min_z}, {self.max_z}] m')

    # ------------------------------------------------------------------
    # 数据回调
    # ------------------------------------------------------------------

    def _on_det_py(self, msg: PointCloud2):
        """接收 Python 版检测列表"""
        self._latest_det_py = msg

    def _on_det_cu(self, msg: PointCloud2):
        """接收 CUDA 版检测列表"""
        self._latest_det_cu = msg

    def _on_obj(self, msg: MarkerArray):
        """接收 3D 目标检测结果"""
        self._latest_obj = msg

    def _on_ruler(self, msg: MarkerArray):
        """接收标尺数据"""
        self._latest_ruler = msg

    # ------------------------------------------------------------------
    # 综合发布回调
    # ------------------------------------------------------------------

    def _on_publish(self):
        """汇聚所有数据源，发布可视化消息"""
        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()

        # ---- 合并 Python 和 CUDA 检测点云 ----
        all_xyz = []
        for det_msg in [self._latest_det_py, self._latest_det_cu]:
            if det_msg is None:
                continue
            data = np.frombuffer(det_msg.data, dtype=np.float32)
            if len(data) == 0:
                continue
            # 每个点 5 个字段 (x, y, z, velocity, snr)，取前 3 个
            pts = data.reshape(-1, 5)
            all_xyz.append(pts[:, :3])

        if all_xyz:
            combined = np.vstack(all_xyz)
            colors = height_to_rgb_array(combined[:, 2], self.min_z, self.max_z)
            pc_msg = create_pointcloud2_rgb(
                combined, colors, self.fixed_frame, stamp)
            self.pub_points.publish(pc_msg)

        # ---- 合并并发布 3D 目标框 + 标尺数据 ----
        merged_markers = MarkerArray()
        if self._latest_obj is not None:
            merged_markers.markers.extend(self._latest_obj.markers)
        if self._latest_ruler is not None:
            merged_markers.markers.extend(self._latest_ruler.markers)
        if merged_markers.markers:
            self.pub_boxes.publish(merged_markers)

        # ---- 发布色带 ----
        if self.bridge is not None:
            try:
                cbar_msg = self.bridge.cv2_to_imgmsg(
                    self._colorbar_img, encoding='bgr8')
                cbar_msg.header = create_header(self.fixed_frame, stamp)
                self.pub_colorbar.publish(cbar_msg)
            except Exception as e:
                self.get_logger().error(f'色带发布失败: {e}')

        # ---- 发布帧信息文字 ----
        py_count = 0
        if self._latest_det_py is not None:
            data = np.frombuffer(self._latest_det_py.data, dtype=np.float32)
            py_count = len(data) // 5
        cu_count = 0
        if self._latest_det_cu is not None:
            data = np.frombuffer(self._latest_det_cu.data, dtype=np.float32)
            cu_count = len(data) // 5

        info = Marker()
        info.header = create_header(self.fixed_frame, stamp)
        info.ns = 'radar_frame_info'
        info.id = 0
        info.type = Marker.TEXT_VIEW_FACING
        info.action = Marker.ADD
        info.pose.position.x = 15.0
        info.pose.position.y = 15.0
        info.pose.position.z = 3.0
        info.pose.orientation.w = 1.0
        info.scale.z = 0.8
        info.color.r = 1.0
        info.color.g = 1.0
        info.color.b = 1.0
        info.color.a = 1.0
        info.text = (f'Det_PY: {py_count} | Det_CU: {cu_count} | '
                     f'Frame: {self.frame_count}')
        lt_sec = self.marker_lifetime
        info.lifetime = Duration(
            sec=int(lt_sec), nanosec=int((lt_sec % 1) * 1e9))

        info_arr = MarkerArray()
        info_arr.markers.append(info)
        self.pub_info.publish(info_arr)

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Rviz_radar 已停止')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = RvizRadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Rviz_radar 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
