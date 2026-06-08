#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 3D 目标检测节点 (3D Object Detection)
================================================================================
基于雷达检测列表 (det_list) 利用 AI 模型进行 3D 目标检测与分类（模拟实现）。
使用简单欧氏聚类替代深度学习模型，生成 3D 目标框和 ID 标签。

订阅话题：
  /ft/det_list_py   PointCloud2    Python版检测目标列表

发布话题：
  /ft/obj_list      MarkerArray    3D 目标检测结果（目标框 + 类别标签）

连接关系：
  ← R SP MIL Python (sub)
  → Rviz_radar (pub)

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 检测参数 ----------
CLUSTER_DISTANCE = 5.0        # 聚类距离阈值 (m)
MIN_CLUSTER_SIZE = 3          # 最小簇大小（点数），少于此值的簇被忽略
BOX_HEIGHT       = 2.0        # 默认目标框高度 (m)
MARKER_LIFETIME  = 1.0        # Marker 生命周期 (s)

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# ============================================================================
# 工具函数
# ============================================================================

def create_header(frame_id: str, stamp) -> Header:
    """创建 ROS2 消息头"""
    h = Header()
    h.frame_id = frame_id
    h.stamp = stamp
    return h


def simple_clustering(points: np.ndarray, dist_threshold: float) -> list:
    """
    简单的欧氏距离聚类（BFS 实现）。

    points: [N, 3] — (x, y, z)
    返回: list of np.ndarray，每个元素是一个簇的点集
    """
    if len(points) == 0:
        return []

    clusters = []
    visited = np.zeros(len(points), dtype=bool)

    for i in range(len(points)):
        if visited[i]:
            continue

        # BFS 聚类
        cluster_indices = [i]
        visited[i] = True
        head = 0

        while head < len(cluster_indices):
            idx = cluster_indices[head]
            # 计算当前点到所有未访问点的欧氏距离
            dists = np.linalg.norm(points - points[idx], axis=1)
            neighbors = np.where((dists < dist_threshold) & (~visited))[0]
            for n in neighbors:
                visited[n] = True
                cluster_indices.append(n)
            head += 1

        if len(cluster_indices) >= 1:
            clusters.append(points[cluster_indices])

    return clusters


def create_obj_marker(cluster_pts: np.ndarray, obj_id: int,
                       frame_id: str, stamp,
                       lifetime_sec: float) -> Marker:
    """
    根据点云簇创建 3D 目标框 Marker（CUBE 类型）。
    """
    min_xyz = cluster_pts.min(axis=0)
    max_xyz = cluster_pts.max(axis=0)

    cx = (min_xyz[0] + max_xyz[0]) / 2.0
    cy = (min_xyz[1] + max_xyz[1]) / 2.0
    cz = (min_xyz[2] + max_xyz[2]) / 2.0

    L = max(max_xyz[0] - min_xyz[0], 1.0)      # 长度（沿 X）
    W = max(max_xyz[1] - min_xyz[1], 1.0)      # 宽度（沿 Y）
    H = max(max_xyz[2] - min_xyz[2], BOX_HEIGHT)  # 高度

    lt = Duration(
        sec=int(lifetime_sec),
        nanosec=int((lifetime_sec - int(lifetime_sec)) * 1e9))

    m = Marker()
    m.header = create_header(frame_id, stamp)
    m.ns = 'obj_boxes'
    m.id = obj_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = cx
    m.pose.position.y = cy
    m.pose.position.z = cz + H / 2.0
    m.pose.orientation.w = 1.0
    m.scale.x = L
    m.scale.y = W
    m.scale.z = H
    # 半透明绿色
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.color.a = 0.5
    m.lifetime = lt

    return m


def create_obj_label(cx: float, cy: float, cz: float, obj_id: int,
                      frame_id: str, stamp, lifetime_sec: float) -> Marker:
    """
    创建目标 ID 文字标签（TEXT_VIEW_FACING 类型）。
    """
    lt = Duration(
        sec=int(lifetime_sec),
        nanosec=int((lifetime_sec - int(lifetime_sec)) * 1e9))

    m = Marker()
    m.header = create_header(frame_id, stamp)
    m.ns = 'obj_labels'
    m.id = obj_id + 10000          # 偏移避免与框 ID 冲突
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = cx
    m.pose.position.y = cy
    m.pose.position.z = cz + BOX_HEIGHT + 0.5
    m.pose.orientation.w = 1.0
    m.scale.z = 0.8
    # 黄色文字
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.color.a = 1.0
    m.text = f'Obj:{obj_id}'
    m.lifetime = lt

    return m


# ============================================================================
# ROS2 节点
# ============================================================================

class ObjectDetection3DNode(Node):
    """
    3D 目标检测节点（模拟 AI 模型 —— 基于欧氏聚类）

    订阅话题：
      /ft/det_list_py   PointCloud2

    发布话题：
      /ft/obj_list      MarkerArray
    """

    def __init__(self):
        super().__init__('object_detection_3d')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('cluster_distance', CLUSTER_DISTANCE)
        self.declare_parameter('min_cluster_size', MIN_CLUSTER_SIZE)
        self.declare_parameter('box_height',       BOX_HEIGHT)
        self.declare_parameter('marker_lifetime',  MARKER_LIFETIME)
        self.declare_parameter('fixed_frame',      FIXED_FRAME)

        self.cluster_distance = float(
            self.get_parameter('cluster_distance').value)
        self.min_cluster_size = int(
            self.get_parameter('min_cluster_size').value)
        self.box_height       = float(self.get_parameter('box_height').value)
        self.marker_lifetime  = float(
            self.get_parameter('marker_lifetime').value)
        self.fixed_frame      = self.get_parameter('fixed_frame').value

        # ---------- 订阅者 ----------
        self.sub_det = self.create_subscription(
            PointCloud2, '/ft/det_list_py', self._on_det_list, 10)

        # ---------- 发布者 ----------
        self.pub_obj = self.create_publisher(MarkerArray, '/ft/obj_list', 10)

        self.frame_count = 0
        self.get_logger().info(
            f'3D Object Detection 启动: '
            f'聚类距离={self.cluster_distance}m, '
            f'最小簇大小={self.min_cluster_size} [AI 模拟]')

    # ------------------------------------------------------------------
    # 检测回调
    # ------------------------------------------------------------------

    def _on_det_list(self, msg: PointCloud2):
        """
        处理检测列表：
        1. 解析 Detection List PointCloud2
        2. 欧氏聚类（模拟 AI 目标检测）
        3. 过滤小簇
        4. 生成 3D 目标框 + 标签
        5. 发布 MarkerArray
        """
        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()

        # 解析点云
        data = np.frombuffer(msg.data, dtype=np.float32)
        if len(data) == 0:
            return
        pts_5 = data.reshape(-1, 5)       # (x, y, z, velocity, snr)
        pts_xyz = pts_5[:, :3]            # 仅取空间坐标用于聚类

        # 欧氏聚类
        clusters = simple_clustering(pts_xyz, self.cluster_distance)

        # 过滤小簇
        valid_clusters = [
            c for c in clusters if len(c) >= self.min_cluster_size
        ]

        # ---- 先发 DELETEALL 清除旧框 ----
        del_arr = MarkerArray()
        for ns in ('obj_boxes', 'obj_labels'):
            dm = Marker()
            dm.header = create_header(self.fixed_frame, stamp)
            dm.ns = ns
            dm.action = Marker.DELETEALL
            del_arr.markers.append(dm)
        self.pub_obj.publish(del_arr)

        # ---- 发布新目标框和标签 ----
        marr = MarkerArray()
        for i, cluster in enumerate(valid_clusters):
            # 目标框
            marr.markers.append(
                create_obj_marker(cluster, i, self.fixed_frame, stamp,
                                  self.marker_lifetime))
            # 标签（使用簇中心）
            cx = (cluster[:, 0].min() + cluster[:, 0].max()) / 2.0
            cy = (cluster[:, 1].min() + cluster[:, 1].max()) / 2.0
            cz = (cluster[:, 2].min() + cluster[:, 2].max()) / 2.0
            marr.markers.append(
                create_obj_label(cx, cy, cz, i, self.fixed_frame, stamp,
                                 self.marker_lifetime))

        if marr.markers:
            self.pub_obj.publish(marr)

        self.get_logger().info(
            f'[3D-DET] 帧 #{self.frame_count}: '
            f'{len(pts_xyz)} 检测点 → {len(clusters)} 个簇 → '
            f'{len(valid_clusters)} 个有效目标 (≥{self.min_cluster_size}点)')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('3D Object Detection 已停止')
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
