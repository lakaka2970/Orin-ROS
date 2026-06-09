#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 3D 目标检测节点 (3D Object Detection)
================================================================================
基于雷达检测列表 (det_list) 利用 AI 模型进行 3D 目标检测与分类（模拟实现）。

规格:
  - 输入: DetList (14 字段检测点)
  - 输出: ObjList (14 字段 3D 目标)，与 FT_radar_dataset_requirement 第 7 节完全对齐
  - 算法: 欧氏聚类模拟 AI 检测

话题:
  订阅: /processing/radar/det_list    ft_radar_msgs/DetList
  发布: /perception/objects            ft_radar_msgs/ObjList
        /visualization/radar/boxes     visualization_msgs/MarkerArray (RViz 显示用)

连接关系:
  ← RSP MIL Python (sub)
  → Rviz_radar (pub)
  → Logging (pub)

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 检测参数 ----------
CLUSTER_DISTANCE = 5.0        # 欧氏聚类距离阈值 (m)
MIN_CLUSTER_SIZE = 3          # 最小簇大小（点数）
DEFAULT_BOX_H   = 2.0         # 默认目标框高度 (m)
MARKER_LIFETIME = 1.0         # RViz Marker 生命周期 (s)

# ---------- 模拟 AI 检测参数 ----------
MOVING_STATE_MAP = {          # moving_state 枚举
    0: 'moving', 1: 'stationary', 2: 'oncoming',
    3: 'cross', 4: 'stopped', 255: 'unknown'
}

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import numpy as np

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from ft_radar_msgs.msg import DetList, ObjList, Object3D
from ft_framework.common import monotonic_us_stamp, create_header


def simple_clustering(points_xyz: np.ndarray, dist_threshold: float) -> list:
    """
    简单欧氏距离聚类（BFS 实现）。
    points_xyz: [N, 3] — (x, y, z)
    返回: list of np.ndarray，每个元素是一个簇的点集
    """
    if len(points_xyz) == 0:
        return []

    clusters = []
    visited = np.zeros(len(points_xyz), dtype=bool)

    for i in range(len(points_xyz)):
        if visited[i]:
            continue
        indices = [i]
        visited[i] = True
        head = 0
        while head < len(indices):
            idx = indices[head]
            dists = np.linalg.norm(points_xyz - points_xyz[idx], axis=1)
            neighbors = np.where((dists < dist_threshold) & (~visited))[0]
            for n in neighbors:
                visited[n] = True
                indices.append(n)
            head += 1
        if len(indices) >= 1:
            clusters.append(points_xyz[indices])

    return clusters


def create_box_marker(center: tuple, size: tuple, yaw: float,
                       obj_id: int, frame_id: str, stamp,
                       lifetime_sec: float) -> Marker:
    """创建 3D 目标框 Marker（CUBE 类型）"""
    lt = Duration(sec=int(lifetime_sec),
                  nanosec=int((lifetime_sec % 1) * 1e9))

    m = Marker()
    m.header = create_header(frame_id, stamp)
    m.ns = 'obj_3d_boxes'
    m.id = obj_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = center[0]
    m.pose.position.y = center[1]
    m.pose.position.z = center[2]
    m.pose.orientation.w = 1.0
    m.scale.x = size[0]
    m.scale.y = size[1]
    m.scale.z = size[2]
    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.5
    m.lifetime = lt
    return m


def create_label_marker(center: tuple, text: str, obj_id: int,
                         frame_id: str, stamp, lifetime_sec: float) -> Marker:
    """创建文字标签 Marker（TEXT_VIEW_FACING 类型）"""
    lt = Duration(sec=int(lifetime_sec),
                  nanosec=int((lifetime_sec % 1) * 1e9))
    m = Marker()
    m.header = create_header(frame_id, stamp)
    m.ns = 'obj_3d_labels'
    m.id = obj_id + 10000
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = center[0]
    m.pose.position.y = center[1]
    m.pose.position.z = center[2] + DEFAULT_BOX_H + 0.5
    m.pose.orientation.w = 1.0
    m.scale.z = 0.8
    m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
    m.text = text
    m.lifetime = lt
    return m


# ============================================================================
# ROS2 节点
# ============================================================================

class ObjectDetection3DNode(Node):
    """
    3D 目标检测节点

    话题:
      订阅: /processing/radar/det_list    (DetList)
      发布: /perception/objects            (ObjList, 数据输出)
            /visualization/radar/boxes     (MarkerArray, RViz 可视化)

    功能说明:
      - 接收 DetList 检测点
      - 欧氏聚类模拟 AI 检测
      - 输出 ObjList 含 14 字段（与 FT_radar_dataset_requirement 对齐）
      - 同时发布 MarkerArray 供 RViz 显示
    """

    def __init__(self):
        super().__init__('object_detection_3d')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('cluster_distance', CLUSTER_DISTANCE)
        self.declare_parameter('min_cluster_size', MIN_CLUSTER_SIZE)
        self.declare_parameter('marker_lifetime', MARKER_LIFETIME)
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.cluster_distance = float(self.get_parameter('cluster_distance').value)
        self.min_cluster_size = int(self.get_parameter('min_cluster_size').value)
        self.marker_lifetime  = float(self.get_parameter('marker_lifetime').value)
        self.fixed_frame      = self.get_parameter('fixed_frame').value

        # ---------- 订阅 ----------
        self.sub_det = self.create_subscription(
            DetList, '/processing/radar/det_list', self._on_det_list, 10)

        # ---------- 发布（数据 + 可视化） ----------
        self.pub_objects = self.create_publisher(
            ObjList, '/perception/objects', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/visualization/radar/boxes', 10)

        self.frame_count = 0
        self._next_obj_id = 1

        self.get_logger().info(
            f'3D Object Detection 启动: '
            f'聚类距离={self.cluster_distance}m, '
            f'最小簇={self.min_cluster_size} [AI 模拟]')

    # ------------------------------------------------------------------
    # 检测回调
    # ------------------------------------------------------------------

    def _on_det_list(self, msg: DetList):
        """
        处理检测列表:
          1. 解析 DetPoint 数组
          2. 欧氏聚类（模拟 AI 检测）
          3. 过滤小簇
          4. 生成 ObjList 14 字段 + MarkerArray
          5. 发布
        """
        self.frame_count += 1

        n_points = len(msg.points)
        if n_points == 0:
            return

        # ---- 提取 (x, y, z) 用于聚类 ----
        xyz_array = np.array([
            [p.x, p.y, p.z] for p in msg.points
        ], dtype=np.float32)

        # ---- 欧氏聚类 ----
        clusters = simple_clustering(xyz_array, self.cluster_distance)
        valid_clusters = [c for c in clusters if len(c) >= self.min_cluster_size]

        # ---- 构造 ObjList ----
        obj_list = ObjList()
        obj_list.header.stamp.sec = msg.header.stamp.sec  # 透传原始时间戳
        obj_list.header.stamp.nanosec = msg.header.stamp.nanosec
        obj_list.header.frame_id = self.fixed_frame

        # ---- 构造 MarkerArray（RViz 显示） ----
        marker_arr = MarkerArray()
        for ns in ('obj_3d_boxes', 'obj_3d_labels'):
            dm = Marker()
            dm.header = create_header(self.fixed_frame, msg.header.stamp)
            dm.ns = ns
            dm.action = Marker.DELETEALL
            marker_arr.markers.append(dm)

        for i, cluster in enumerate(valid_clusters):
            obj_id = self._next_obj_id + i
            min_xyz = cluster.min(axis=0)
            max_xyz = cluster.max(axis=0)

            cx = (min_xyz[0] + max_xyz[0]) / 2.0
            cy = (min_xyz[1] + max_xyz[1]) / 2.0
            cz = (min_xyz[2] + max_xyz[2]) / 2.0
            L = max(max_xyz[0] - min_xyz[0], 1.0)
            W = max(max_xyz[1] - min_xyz[1], 1.0)
            H = max(max_xyz[2] - min_xyz[2], DEFAULT_BOX_H)

            # ---- 填充 Object3D 14 字段 ----
            obj = Object3D()
            obj.object_id = obj_id
            obj.tracked_times = 1
            obj.score = float(np.clip(
                len(cluster) / 20.0, 0, 1))               # 簇越大→置信度越高
            obj.x = cx
            obj.y = cy
            obj.z = cz
            obj.l = L
            obj.w = W
            obj.h = H
            obj.yaw = 0.0                                  # 模拟
            obj.vx_absolute = float(np.random.uniform(-5, 15))
            obj.vy_absolute = float(np.random.uniform(-2, 2))
            obj.vz_absolute = 0.0
            obj.moving_state = 0 if abs(obj.vx_absolute) > 1 else 1
            obj_list.objects.append(obj)

            # ---- Marker 用于 RViz 显示 ----
            marker_arr.markers.append(create_box_marker(
                (cx, cy, cz + H / 2), (L, W, H), 0.0,
                obj_id, self.fixed_frame, msg.header.stamp,
                self.marker_lifetime))
            marker_arr.markers.append(create_label_marker(
                (cx, cy, cz), f'Obj:{obj_id}',
                obj_id, self.fixed_frame, msg.header.stamp,
                self.marker_lifetime))

        # ---- 发布 ----
        if obj_list.objects:
            self.pub_objects.publish(obj_list)
        if marker_arr.markers:
            self.pub_markers.publish(marker_arr)

        self._next_obj_id += len(valid_clusters)

        self.get_logger().info(
            f'[3D-DET] 帧 #{self.frame_count}: '
            f'{n_points} 检测点 → {len(clusters)} 簇 → '
            f'{len(valid_clusters)} 有效目标 (≥{self.min_cluster_size}点)')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info(f'3D Object Detection 已停止（共处理 {self.frame_count} 帧）')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('3D Object Detection 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
