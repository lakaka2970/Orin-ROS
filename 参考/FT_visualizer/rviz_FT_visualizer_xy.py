#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达数据 XY 视图可视化工具
================================================================================
读取 FVR50 CSV 点云文件和 Trks CSV 航迹文件，逐帧发布到 RViz：
  - /ft/points_xy    PointCloud2  点云（按高度 zpos 着色）
  - /ft/track_boxes  MarkerArray  航迹框（来自 Trks 文件）
  - /ft/colorbar     Image        高度色带
  - /ft/frame_info   MarkerArray  当前帧 ID 文字标注

manual 模式键盘控制（终端输入后回车）：
  n / Enter   下一帧
  p           上一帧
  r           跳回第一帧
  q           退出

作者：Zhengyuan.Liu
日期：2026.5.9
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 数据文件路径（相对于本脚本所在目录） ----------
import os as _os
_HERE = _os.path.dirname(_os.path.abspath(__file__))
POINTS_CSV = _os.path.join(_HERE, '立着轮胎', 'FVR50_20260430 11-07-38.csv')
TRACKS_CSV = _os.path.join(_HERE, '立着轮胎', 'FVR50_20260430 11-07-38Trks.csv')

# ---------- 播放控制 ----------
PLAY_MODE  = 'auto'   # 'auto'：自动逐帧播放；'manual'：键盘控制
RATE_HZ    = 5.0      # 自动播放帧率（Hz）
LOOP       = True     # 播放到末尾后是否循环

# ---------- 显示范围过滤（单位：米） ----------
X_MIN      = -100.0   # 前向显示起始距离（负值表示雷达后方）
X_MAX      =  300.0   # 前向显示最大距离
Y_RANGE    =  100.0   # 横向显示范围（±Y_RANGE，即左右各 Y_RANGE 米）
Z_FILTER_MIN = -9999.0  # 高度下限过滤：低于此值的点不显示（-9999 = 不过滤）
Z_FILTER_MAX =  9999.0  # 高度上限过滤：高于此值的点不显示（9999 = 不过滤）

# ---------- 高度色带范围（单位：米） ----------
# 设为 -9999 时自动取全局 2%/98% 百分位
Z_MIN      = -9999.0
Z_MAX      = -9999.0

# ---------- 点云数据过滤 ----------
# DOAMethod 筛选：仅显示该列值等于 DOA_FILTER 的点
# 设为 -1 时不过滤，显示全部点
DOA_FILTER = 1        # 通常 1 = 有效 DOA 点

# ---------- 航迹显示开关 ----------
# True：显示航迹框和 ID 标签；False：隐藏所有航迹
SHOW_TRACKS = True

# ---------- 关注目标高亮 ----------
# 将指定 assoTrkID 的点云显示为红色，设为 -1 时不高亮
FOCUS_ID    = -1

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import math
import sys
import threading
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd
import cv2

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField, Image
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from geometry_msgs.msg import TransformStamped

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


def jet_color_scalar(t: float) -> Tuple[int, int, int]:
    """t in [0,1] → (r, g, b) in [0,255]，jet 色图"""
    t = float(np.clip(t, 0.0, 1.0))
    r = float(np.clip(1.5 - abs(4 * t - 3), 0, 1))
    g = float(np.clip(1.5 - abs(4 * t - 2), 0, 1))
    b = float(np.clip(1.5 - abs(4 * t - 1), 0, 1))
    return (int(r * 255), int(g * 255), int(b * 255))


def height_to_rgb_array(z_arr: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    """高度数组 → RGB 颜色数组 [N, 3] uint8，向量化 jet 色图"""
    t = (z_arr - z_min) / (z_max - z_min + 1e-6)
    t = np.clip(t, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def create_pointcloud2_rgb(points_xyz: np.ndarray, colors_rgb: np.ndarray,
                            frame_id: str, stamp) -> PointCloud2:
    """
    创建带 RGB 颜色的 PointCloud2，直接构造二进制数据（避免 .tolist() 慢路径）。
    colors_rgb: [N, 3] uint8，顺序为 R G B。
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

    # 向量化打包 RGB → uint32 → float32（RViz packed RGB 格式）
    packed_rgb = (colors_rgb[:, 0].astype(np.uint32) << 16 |
                  colors_rgb[:, 1].astype(np.uint32) << 8  |
                  colors_rgb[:, 2].astype(np.uint32))
    rgb_f32 = packed_rgb.view(np.float32)

    xyz = points_xyz[:, :3].astype(np.float32)
    cloud = np.column_stack([xyz, rgb_f32.reshape(-1, 1)]).astype(np.float32)
    msg.data = cloud.tobytes()
    return msg


def create_colorbar_image(z_min: float, z_max: float,
                           img_height: int = 300,
                           bar_width: int = 30,
                           label_width: int = 65) -> np.ndarray:
    """生成高度色带图像（BGR uint8）"""
    img = np.zeros((img_height, bar_width + label_width, 3), dtype=np.uint8)
    for i in range(img_height):
        t = 1.0 - i / max(img_height - 1, 1)
        r, g, b = jet_color_scalar(t)
        img[i, :bar_width] = [b, g, r]
    n_labels = 5
    for i in range(n_labels + 1):
        t = i / n_labels
        z_val = z_min + t * (z_max - z_min)
        y = int((1.0 - t) * (img_height - 1))
        cv2.line(img, (bar_width, y), (bar_width + 6, y), (255, 255, 255), 1)
        cv2.putText(img, f'{z_val:.2f}m', (bar_width + 8, min(y + 5, img_height - 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    cv2.putText(img, 'Z(m)', (2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    return img


# ============================================================================
# 数据加载
# ============================================================================

def load_points_csv(path: str) -> Dict[int, np.ndarray]:
    """
    加载点云 CSV，按 frameID 分组。
    返回 {frameID: ndarray [N, 5]}，列为 (xpos, ypos, zpos, assoTrkID, DOAMethod)。
    """
    df = pd.read_csv(path)
    df.columns = df.columns.str.strip()

    required = ['frameID', 'xpos', 'ypos', 'zpos', 'assoTrkID', 'DOAMethod']
    for col in required:
        if col not in df.columns:
            raise ValueError(f'点云 CSV 缺少列：{col}，实际列：{df.columns.tolist()}')

    result: Dict[int, np.ndarray] = {}
    for fid, grp in df.groupby('frameID'):
        arr = grp[['xpos', 'ypos', 'zpos', 'assoTrkID', 'DOAMethod']].to_numpy(dtype=np.float32)
        result[int(fid)] = arr
    return result


def load_tracks_csv(path: str) -> Dict[int, pd.DataFrame]:
    """
    加载航迹 CSV 文件，按 frameID 分组。
    返回 {frameID: DataFrame}。
    """
    df = pd.read_csv(path)
    df.columns = df.columns.str.strip()

    required = ['frameID', 'objID', 'objXPos', 'objYPos',
                'objBoxLength', 'objBoxWidth', 'f32ObjBoxHeight', 'headingAngle']
    for col in required:
        if col not in df.columns:
            raise ValueError(f'航迹文件缺少列：{col}，实际列：{df.columns.tolist()}')

    result: Dict[int, pd.DataFrame] = {}
    for fid, grp in df.groupby('frameID'):
        result[int(fid)] = grp.reset_index(drop=True)
    return result


# ============================================================================
# 航迹框 Marker
# ============================================================================

def _safe_float(val, default: float = 0.0) -> float:
    """将值转为 float，NaN/None 返回 default"""
    try:
        v = float(val)
        return default if math.isnan(v) else v
    except (TypeError, ValueError):
        return default


def create_track_box_markers(trk_row: pd.Series, marker_id: int,
                              frame_id: str, stamp, lifetime_sec: float) -> List[Marker]:
    """根据航迹行创建 2D 矩形框 + 文字标签 Marker 列表"""
    cx      = _safe_float(trk_row['objXPos'])
    cy      = _safe_float(trk_row['objYPos'])
    lgt_off = _safe_float(trk_row.get('objBoxCenterLgt', 0.0))
    lat_off = _safe_float(trk_row.get('objBoxCenterLat', 0.0))
    L       = _safe_float(trk_row['objBoxLength'])
    W       = _safe_float(trk_row['objBoxWidth'])
    H       = _safe_float(trk_row['f32ObjBoxHeight'])
    yaw     = _safe_float(trk_row['headingAngle'])
    obj_id  = int(_safe_float(trk_row['objID']))

    if L < 0.01 or W < 0.01:
        return []

    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    # 框中心 = 参考点 + 纵向/横向偏移
    bx = cx + lgt_off * cos_y - lat_off * sin_y
    by = cy + lgt_off * sin_y + lat_off * cos_y

    local = np.array([[+L/2, +W/2], [+L/2, -W/2],
                       [-L/2, -W/2], [-L/2, +W/2]], dtype=np.float32)
    R = np.array([[cos_y, -sin_y], [sin_y, cos_y]], dtype=np.float32)
    corners = (R @ local.T).T + np.array([bx, by], dtype=np.float32)

    lt = Duration(sec=int(lifetime_sec),
                  nanosec=int((lifetime_sec - int(lifetime_sec)) * 1e9))

    box = Marker()
    box.header = create_header(frame_id, stamp)
    box.ns = 'track_boxes'
    box.id = marker_id
    box.type = Marker.LINE_STRIP
    box.action = Marker.ADD
    box.pose.orientation.w = 1.0
    box.scale.x = 0.12
    box.color.r = 0.0
    box.color.g = 1.0
    box.color.b = 1.0
    box.color.a = 1.0
    box.lifetime = lt

    for i in range(5):
        p = Point()
        idx = i % 4
        p.x, p.y, p.z = float(corners[idx, 0]), float(corners[idx, 1]), 0.0
        box.points.append(p)

    txt = Marker()
    txt.header = create_header(frame_id, stamp)
    txt.ns = 'track_text'
    txt.id = marker_id + 10000
    txt.type = Marker.TEXT_VIEW_FACING
    txt.action = Marker.ADD
    txt.pose.position.x = bx
    txt.pose.position.y = by
    txt.pose.position.z = max(H / 2.0 + 0.3, 0.5)
    txt.pose.orientation.w = 1.0
    txt.scale.z = 0.6
    txt.color.r = 1.0
    txt.color.g = 1.0
    txt.color.b = 0.0
    txt.color.a = 1.0
    txt.text = f'ID:{obj_id}'
    txt.lifetime = lt

    return [box, txt]


# ============================================================================
# ROS2 节点
# ============================================================================

class FTVisualizerXY(Node):
    """
    FT 雷达数据 XY 视图可视化节点

    发布话题：
      /ft/points_xy    PointCloud2   点云（按高度着色）
      /ft/track_boxes  MarkerArray   航迹框
      /ft/colorbar     Image         高度色带
      /ft/frame_info   MarkerArray   帧 ID 文字标注
    """

    _AUTO_SENTINEL = -9999.0

    def __init__(self):
        super().__init__('ft_visualizer_xy')

        # ROS2 参数声明（默认值来自顶部配置区，可被 ros2 run --ros-args -p 覆盖）
        self.declare_parameter('points_csv',   POINTS_CSV)
        self.declare_parameter('tracks_csv',   TRACKS_CSV)
        self.declare_parameter('fixed_frame',  FIXED_FRAME)
        self.declare_parameter('rate',         RATE_HZ)
        self.declare_parameter('loop',         LOOP)
        self.declare_parameter('play_mode',    PLAY_MODE)
        self.declare_parameter('z_min',        Z_MIN)
        self.declare_parameter('z_max',        Z_MAX)
        self.declare_parameter('x_max',        X_MAX)
        self.declare_parameter('x_min',        X_MIN)
        self.declare_parameter('y_range',      Y_RANGE)
        self.declare_parameter('z_filter_min', Z_FILTER_MIN)
        self.declare_parameter('z_filter_max', Z_FILTER_MAX)
        self.declare_parameter('doa_filter',   DOA_FILTER)
        self.declare_parameter('show_tracks',  SHOW_TRACKS)
        self.declare_parameter('focus_id',     FOCUS_ID)

        self.points_csv   = self.get_parameter('points_csv').value
        self.tracks_csv   = self.get_parameter('tracks_csv').value
        self.fixed_frame  = self.get_parameter('fixed_frame').value
        self.rate         = float(self.get_parameter('rate').value)
        self.loop         = bool(self.get_parameter('loop').value)
        self.play_mode    = self.get_parameter('play_mode').value
        z_min_param       = float(self.get_parameter('z_min').value)
        z_max_param       = float(self.get_parameter('z_max').value)
        self.x_max        = float(self.get_parameter('x_max').value)
        self.x_min        = float(self.get_parameter('x_min').value)
        self.y_range      = float(self.get_parameter('y_range').value)
        self.z_filter_min = float(self.get_parameter('z_filter_min').value)
        self.z_filter_max = float(self.get_parameter('z_filter_max').value)
        self.doa_filter   = int(self.get_parameter('doa_filter').value)
        self.show_tracks  = bool(self.get_parameter('show_tracks').value)
        self.focus_id     = int(self.get_parameter('focus_id').value)

        self.get_logger().info(f'加载点云：{self.points_csv}')
        self.points_by_frame = load_points_csv(self.points_csv)

        self.get_logger().info(f'加载航迹：{self.tracks_csv}')
        self.tracks_by_frame = load_tracks_csv(self.tracks_csv)

        self.frame_ids = sorted(self.points_by_frame.keys())
        self.get_logger().info(f'共 {len(self.frame_ids)} 帧，frameID 范围：'
                               f'{self.frame_ids[0]} ~ {self.frame_ids[-1]}')

        all_z = np.concatenate([pts[:, 2] for pts in self.points_by_frame.values()])
        self.z_min = (float(np.percentile(all_z, 2))
                      if z_min_param == self._AUTO_SENTINEL else z_min_param)
        self.z_max = (float(np.percentile(all_z, 98))
                      if z_max_param == self._AUTO_SENTINEL else z_max_param)
        self.get_logger().info(f'高度范围：{self.z_min:.3f} ~ {self.z_max:.3f} m')
        self.get_logger().info(f'显示范围：x=[{self.x_min:.1f}, {self.x_max:.1f}] m  '
                               f'y=[{-self.y_range:.1f}, {self.y_range:.1f}] m')
        self.get_logger().info(f'DOA 过滤：{"全部显示" if self.doa_filter < 0 else f"仅 DOAMethod={self.doa_filter}"}  '
                               f'航迹显示：{"开" if self.show_tracks else "关"}')

        self._colorbar_img = create_colorbar_image(self.z_min, self.z_max)

        # 发布静态 TF：radar → map（identity），解决 RViz 坐标系转换报错
        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = self.fixed_frame
        tf_msg.transform.rotation.w = 1.0
        self._tf_static.sendTransform(tf_msg)

        self.pub_pts   = self.create_publisher(PointCloud2, '/ft/points_xy',   10)
        self.pub_boxes = self.create_publisher(MarkerArray, '/ft/track_boxes', 10)
        self.pub_cbar  = self.create_publisher(Image,       '/ft/colorbar',    10)
        self.pub_info  = self.create_publisher(MarkerArray, '/ft/frame_info',  10)

        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().warn('cv_bridge 未安装，色带将不发布')

        self.index = 0
        self._publish_pending = (self.play_mode == 'manual')
        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)

        self.get_logger().info(f'播放模式：{self.play_mode}  帧率：{self.rate} Hz')
        if self.play_mode == 'manual':
            self.get_logger().info('手动模式键盘控制（终端输入后回车）：'
                                   'n/Enter=下一帧  p=上一帧  r=重置  q=退出')
            self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
            self._stdin_thread.start()

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        if not self.frame_ids:
            return

        if self.play_mode == 'auto':
            if self.index >= len(self.frame_ids):
                if self.loop:
                    self.index = 0
                    self.get_logger().info('循环播放：重新开始')
                else:
                    self.get_logger().info('播放完成')
                    return
            self._publish_frame(self.index)
            self.index += 1
        else:
            if self._publish_pending:
                idx = max(0, min(self.index, len(self.frame_ids) - 1))
                self._publish_frame(idx)
                self._publish_pending = False

    # ------------------------------------------------------------------
    # 单帧发布
    # ------------------------------------------------------------------

    def _publish_frame(self, idx: int):
        fid   = self.frame_ids[idx]
        stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f'发布帧 frameID={fid}  ({idx + 1}/{len(self.frame_ids)})')

        pts = self.points_by_frame[fid]   # [N, 5]: xpos, ypos, zpos, assoTrkID, DOAMethod

        # DOAMethod 过滤（-1 = 不过滤）
        if self.doa_filter >= 0:
            pts = pts[pts[:, 4].astype(np.int32) == self.doa_filter]

        # 空间范围过滤（含 Z 轴高度过滤）
        mask = ((pts[:, 0] >= self.x_min) & (pts[:, 0] <= self.x_max) &
                (pts[:, 1] >= -self.y_range) & (pts[:, 1] <= self.y_range) &
                (pts[:, 2] >= self.z_filter_min) & (pts[:, 2] <= self.z_filter_max))
        pts = pts[mask]

        xyz    = pts[:, :3]
        colors = height_to_rgb_array(pts[:, 2], self.z_min, self.z_max)
        if self.focus_id >= 0:
            focus_mask = (pts[:, 3].astype(np.int32) == self.focus_id)
            colors[focus_mask] = [255, 0, 0]
        self.pub_pts.publish(create_pointcloud2_rgb(xyz, colors, self.fixed_frame, stamp))

        # 先发 DELETEALL 清除上一帧旧框，再发新框
        del_msg = MarkerArray()
        for ns in ('track_boxes', 'track_text'):
            dm = Marker()
            dm.header = create_header(self.fixed_frame, stamp)
            dm.ns = ns
            dm.action = Marker.DELETEALL
            del_msg.markers.append(dm)
        self.pub_boxes.publish(del_msg)

        lifetime = 2.0 / self.rate + 0.5
        if self.show_tracks:
            marr = MarkerArray()
            if fid in self.tracks_by_frame:
                for mid, (_, row) in enumerate(self.tracks_by_frame[fid].iterrows()):
                    marr.markers.extend(
                        create_track_box_markers(row, mid, self.fixed_frame, stamp, lifetime))
            if marr.markers:
                self.pub_boxes.publish(marr)

        if self.bridge is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(self._colorbar_img, encoding='bgr8')
                img_msg.header = create_header(self.fixed_frame, stamp)
                self.pub_cbar.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'色带发布失败：{e}')

        info_marker = Marker()
        info_marker.header = create_header(self.fixed_frame, stamp)
        info_marker.ns = 'frame_info'
        info_marker.id = 0
        info_marker.type = Marker.TEXT_VIEW_FACING
        info_marker.action = Marker.ADD
        info_marker.pose.position.x = 20.0
        info_marker.pose.position.y = 20.0
        info_marker.pose.position.z = 2.0
        info_marker.pose.orientation.w = 1.0
        info_marker.scale.x = 1.0
        info_marker.scale.z = 1.2
        info_marker.color.r = 1.0
        info_marker.color.g = 1.0
        info_marker.color.b = 1.0
        info_marker.color.a = 1.0
        info_marker.text = f'Frame: {fid}  ({idx + 1}/{len(self.frame_ids)})'
        info_marker.lifetime = Duration(sec=int(lifetime), nanosec=int((lifetime % 1) * 1e9))
        info_arr = MarkerArray()
        info_arr.markers.append(info_marker)
        self.pub_info.publish(info_arr)

    # ------------------------------------------------------------------
    # stdin 键盘线程（manual 模式）
    # ------------------------------------------------------------------

    def _stdin_loop(self):
        """后台线程：读取 stdin 控制帧播放"""
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                cmd = line.strip().lower()
                n = len(self.frame_ids)
                if cmd in ('n', ''):
                    self.index = min(self.index + 1, n - 1)
                    self._publish_pending = True
                elif cmd == 'p':
                    self.index = max(self.index - 1, 0)
                    self._publish_pending = True
                elif cmd == 'r':
                    self.index = 0
                    self._publish_pending = True
                elif cmd == 'q':
                    self.get_logger().info('用户输入 q，退出')
                    self.destroy_node()
                    rclpy.shutdown()
                    break
            except Exception:
                break

    def destroy_node(self):
        self.get_logger().info('FT XY Visualizer 已停止')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = FTVisualizerXY()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
