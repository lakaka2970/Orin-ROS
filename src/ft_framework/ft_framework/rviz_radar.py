#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达数据 RViz 可视化节点 (Rviz_radar)
================================================================================
汇聚 DetList、ObjList、Ruler 三路数据源，发布可用于 RViz 显示的可视化消息。

话题:
  订阅:
    /processing/radar/det_list       ft_radar_msgs/DetList     (RSP 检测列表，主)
    /processing/radar/det_list_cuda  ft_radar_msgs/DetList     (RSP 检测列表，CUDA 双路)
    /perception/objects              ft_radar_msgs/ObjList     (3D 目标)
    /visualization/ruler             visualization_msgs/MarkerArray (标尺)

  发布:
    /visualization/radar/display     sensor_msgs/PointCloud2     (着色点云)
    /visualization/radar/boxes       visualization_msgs/MarkerArray (目标框+标尺)
    /visualization/radar/colorbar    sensor_msgs/Image           (高度色带)
    /visualization/radar/frame_info  visualization_msgs/MarkerArray (帧信息)

连接关系:
  ← RSP MIL Python / RSP Cuda (sub)
  ← 3D Object Detection (sub)
  ← Rviz_Ruler (sub)

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 显示参数 ----------
MIN_Z           = -5.0        # 高度色带下限 (m)
MAX_Z           = 15.0        # 高度色带上限 (m)
MARKER_LIFETIME = 1.0         # Marker 生命周期 (s)
PUBLISH_HZ      = 10.0        # 可视化发布频率 (Hz)

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

from ft_radar_msgs.msg import DetList, ObjList

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


# ============================================================================
# 工具函数
# ============================================================================

def create_header(frame_id: str, stamp) -> Header:
    h = Header()
    h.frame_id = frame_id
    h.stamp = stamp
    return h


def jet_color_scalar(t: float):
    t = float(np.clip(t, 0.0, 1.0))
    r = float(np.clip(1.5 - abs(4 * t - 3), 0, 1))
    g = float(np.clip(1.5 - abs(4 * t - 2), 0, 1))
    b = float(np.clip(1.5 - abs(4 * t - 1), 0, 1))
    return (int(r * 255), int(g * 255), int(b * 255))


def height_to_rgb_array(z_arr: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    t = (z_arr - z_min) / (z_max - z_min + 1e-6)
    t = np.clip(t, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def create_pointcloud2_rgb(points_xyz: np.ndarray, colors_rgb: np.ndarray,
                            frame_id: str, stamp) -> PointCloud2:
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
    img = np.zeros((img_height, bar_width + label_width, 3), dtype=np.uint8)
    for i in range(img_height):
        t = 1.0 - i / max(img_height - 1, 1)
        r, g, b = jet_color_scalar(t)
        img[i, :bar_width] = [b, g, r]
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
    """雷达数据 RViz 可视化节点"""

    def __init__(self):
        super().__init__('rviz_radar')

        self.declare_parameter('min_z', MIN_Z)
        self.declare_parameter('max_z', MAX_Z)
        self.declare_parameter('marker_lifetime', MARKER_LIFETIME)
        self.declare_parameter('publish_hz', PUBLISH_HZ)
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.min_z           = float(self.get_parameter('min_z').value)
        self.max_z           = float(self.get_parameter('max_z').value)
        self.marker_lifetime = float(self.get_parameter('marker_lifetime').value)
        self.publish_hz      = float(self.get_parameter('publish_hz').value)
        self.fixed_frame     = self.get_parameter('fixed_frame').value

        self._colorbar_img = create_colorbar_image(self.min_z, self.max_z)
        self.bridge = CvBridge() if CvBridge is not None else None

        # ---------- 数据缓存 ----------
        self._latest_det     = None
        self._latest_det_cu  = None
        self._latest_obj     = None
        self._latest_ruler   = None

        # ---------- 订阅 ----------
        self.create_subscription(
            DetList, '/processing/radar/det_list', self._on_det, 10)
        self.create_subscription(
            DetList, '/processing/radar/det_list_cuda', self._on_det_cu, 10)
        self.create_subscription(
            ObjList, '/perception/objects', self._on_obj, 10)
        self.create_subscription(
            MarkerArray, '/visualization/ruler', self._on_ruler, 10)

        # ---------- 发布 ----------
        self.pub_points   = self.create_publisher(
            PointCloud2, '/visualization/radar/display', 10)
        self.pub_boxes    = self.create_publisher(
            MarkerArray, '/visualization/radar/boxes', 10)
        self.pub_colorbar = self.create_publisher(
            Image, '/visualization/radar/colorbar', 10)
        self.pub_info     = self.create_publisher(
            MarkerArray, '/visualization/radar/frame_info', 10)

        self.timer = self.create_timer(1.0 / self.publish_hz, self._on_publish)
        self.frame_count = 0

        self.get_logger().info(
            f'Rviz_radar 启动: {self.publish_hz:.0f} Hz, '
            f'色带 [{self.min_z}, {self.max_z}] m')

    # ------------------------------------------------------------------
    # 数据回调
    # ------------------------------------------------------------------

    def _on_det(self, msg: DetList):
        self._latest_det = msg

    def _on_det_cu(self, msg: DetList):
        self._latest_det_cu = msg

    def _on_obj(self, msg: ObjList):
        self._latest_obj = msg

    def _on_ruler(self, msg: MarkerArray):
        self._latest_ruler = msg

    # ------------------------------------------------------------------
    # 综合发布
    # ------------------------------------------------------------------

    def _on_publish(self):
        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()

        # ---- 合并所有 DetList → 点云 ----
        all_xyz = []
        for det in [self._latest_det, self._latest_det_cu]:
            if det is None:
                continue
            for p in det.points:
                all_xyz.append([p.x, p.y, p.z])

        if all_xyz:
            combined = np.array(all_xyz, dtype=np.float32)
            colors = height_to_rgb_array(combined[:, 2], self.min_z, self.max_z)
            pc_msg = create_pointcloud2_rgb(
                combined, colors, self.fixed_frame, stamp)
            self.pub_points.publish(pc_msg)

        # ---- 合并 ObjList + Ruler → 目标框 ----
        merged = MarkerArray()
        if self._latest_obj is not None:
            # 从 ObjList 生成 MarkerArray
            for obj in self._latest_obj.objects:
                m = Marker()
                m.header = create_header(self.fixed_frame, stamp)
                m.ns = 'obj_3d_boxes'
                m.id = int(obj.object_id)
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = obj.x
                m.pose.position.y = obj.y
                m.pose.position.z = obj.z + obj.h / 2.0
                m.pose.orientation.w = 1.0
                m.scale.x = obj.l
                m.scale.y = obj.w
                m.scale.z = obj.h
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.5
                m.lifetime = Duration(sec=int(self.marker_lifetime),
                                      nanosec=int((self.marker_lifetime % 1) * 1e9))
                merged.markers.append(m)

        if self._latest_ruler is not None:
            merged.markers.extend(self._latest_ruler.markers)
        if merged.markers:
            self.pub_boxes.publish(merged)

        # ---- 色带 ----
        if self.bridge is not None:
            try:
                cbar_msg = self.bridge.cv2_to_imgmsg(
                    self._colorbar_img, encoding='bgr8')
                cbar_msg.header = create_header(self.fixed_frame, stamp)
                self.pub_colorbar.publish(cbar_msg)
            except Exception as e:
                self.get_logger().error(f'色带发布失败: {e}')

        # ---- 帧信息 ----
        det_count = len(self._latest_det.points) if self._latest_det else 0
        obj_count = len(self._latest_obj.objects) if self._latest_obj else 0
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
        info.color.r = 1.0; info.color.g = 1.0; info.color.b = 1.0; info.color.a = 1.0
        info.text = f'Det: {det_count} | Obj: {obj_count} | Frame: {self.frame_count}'
        lt = self.marker_lifetime
        info.lifetime = Duration(sec=int(lt), nanosec=int((lt % 1) * 1e9))
        info_arr = MarkerArray()
        info_arr.markers.append(info)
        self.pub_info.publish(info_arr)

    def destroy_node(self):
        self.get_logger().info('Rviz_radar 已停止')
        super().destroy_node()


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
