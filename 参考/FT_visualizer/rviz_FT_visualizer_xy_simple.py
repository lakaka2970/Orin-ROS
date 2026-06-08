#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 雷达数据 XY 视图可视化工具（简化版）
================================================================================
读取简化格式点云 CSV 文件，逐帧发布到 RViz（俯视图）：
  - /ft/points_xy   PointCloud2  点云（按高度 zpos 着色）
  - /ft/colorbar    Image        高度色带
  - /ft/frame_info  MarkerArray  当前帧 ID 文字标注
  - /ft/ruler1      MarkerArray  坐标尺 1（独立配置）
  - /ft/ruler2      MarkerArray  坐标尺 2（独立配置）
  - /ft/video       Image        同步视频画面

点云 CSV 格式（列顺序固定）：
  A列：frameID
  B列：xpos
  C列：ypos
  D列：zpos

manual 模式键盘控制（终端输入后回车）：
  n / Enter   下一帧
  p           上一帧
  r           跳回第一帧
  q           退出

作者：Zhengyuan.Liu
日期：2026.5.13
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 数据文件路径（相对于本脚本所在目录） ----------
import os as _os
_HERE = _os.path.dirname(_os.path.abspath(__file__))
POINTS_CSV = _os.path.join(_HERE, 'data', 'FVR60_20260430 11-07-38.csv')

# ---------- 播放控制 ----------
PLAY_MODE  = 'auto'   # 'auto'：自动逐帧播放；'manual'：键盘控制
RATE_HZ    = 5.0      # 自动播放帧率（Hz）
LOOP       = True     # 播放到末尾后是否循环

# ---------- 显示范围过滤（单位：米） ----------
X_MIN      = -100.0   # 前向显示起始距离（负值表示雷达后方）
X_MAX      =  300.0   # 前向显示最大距离
Y_RANGE    =  100.0   # 横向显示范围（±Y_RANGE，即左右各 Y_RANGE 米）
Z_FILTER_MIN = -1.0  # 高度下限过滤：低于此值的点不显示（-9999 = 不过滤）
Z_FILTER_MAX =  9999.0  # 高度上限过滤：高于此值的点不显示（9999 = 不过滤）

# ---------- 高度色带范围（单位：米） ----------
# 设为 -9999 时自动取全局 2%/98% 百分位
Z_MIN      = -9999.0
Z_MAX      = -9999.0

# ---------- 坐标尺 1（发布到 /ft/ruler1） ----------
SHOW_RULER_1     = True     # 是否显示坐标尺 1
RULER_1_AXIS     = 'x'      # 坐标尺方向：'x' 沿 X 轴 / 'y' 沿 Y 轴
RULER_1_OFFSET   = 0.0      # 坐标尺沿正交方向的偏移量（m）
RULER_1_INTERVAL = 50.0     # 相邻标记间隔（m）
RULER_1_LENGTH   = 300.0    # 坐标尺总长度（m），从坐标原点向正方向延伸
RULER_1_FONT     = 1.0      # 字体大小（scale.z）
RULER_1_COLOR    = [0.8, 0.8, 0.8]   # RGB 颜色（0~1）

# ---------- 坐标尺 2（发布到 /ft/ruler2） ----------
SHOW_RULER_2     = False    # 是否显示坐标尺 2
RULER_2_AXIS     = 'y'      # 坐标尺方向：'x' 沿 X 轴 / 'y' 沿 Y 轴
RULER_2_OFFSET   = 0.0      # 坐标尺沿正交方向的偏移量（m）
RULER_2_INTERVAL = 50.0     # 相邻标记间隔（m）
RULER_2_LENGTH   = 200.0    # 坐标尺总长度（m），从坐标原点向正方向延伸
RULER_2_FONT     = 1.0      # 字体大小（scale.z）
RULER_2_COLOR    = [0.8, 0.8, 0.8]   # RGB 颜色（0~1）

# ---------- 同步视频播放 ----------
# 填入视频地址，设为空字符串 "" 时不播放视频
VIDEO_PATH = _os.path.join(_HERE, 'data', 'FVR60_20260430 11-07-38.avi')   # 例如：_os.path.join(_HERE, 'video', 'camera.avi')

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'radar'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import sys
import threading
from typing import Dict, Tuple

import numpy as np
import pandas as pd
import cv2

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
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
    """创建带 RGB 颜色的 PointCloud2"""
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
    for i in range(6):
        t = i / 5
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
    加载简化格式点云 CSV，按 frameID 分组。
    列格式固定：A=frameID, B=xpos, C=ypos, D=zpos（列名不限，按位置读取）。
    返回 {frameID: ndarray [N, 3]}，列为 (xpos, ypos, zpos)。
    """
    df = pd.read_csv(path, header=0)
    if df.shape[1] < 4:
        raise ValueError(f'点云 CSV 至少需要 4 列（frameID, xpos, ypos, zpos），'
                         f'实际只有 {df.shape[1]} 列')

    # 按列位置取值，兼容任意列名
    frame_col = df.iloc[:, 0].astype(int)
    xyz_data  = df.iloc[:, 1:4].to_numpy(dtype=np.float32)

    result: Dict[int, np.ndarray] = {}
    for fid in frame_col.unique():
        mask = (frame_col == fid).to_numpy()
        result[int(fid)] = xyz_data[mask]
    return result


def create_ruler_markers(axis: str, offset: float, interval: float,
                          length: float, font_scale: float, color: list,
                          frame_id: str, stamp) -> MarkerArray:
    """创建坐标尺标记（仅数字，无刻度线）"""
    marr = MarkerArray()
    n_marks = int(length / interval) + 1
    for i in range(n_marks):
        val = i * interval
        m = Marker()
        m.header = create_header(frame_id, stamp)
        m.ns = 'ruler'
        m.id = i
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.scale.z = font_scale
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        if axis == 'x':
            m.pose.position.x = val
            m.pose.position.y = offset
        else:
            m.pose.position.x = offset
            m.pose.position.y = val
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.text = f'{val:.0f}'
        marr.markers.append(m)
    return marr


def load_video_frames(path: str):
    """读取 AVI 视频所有帧，返回 (frames_list, count) 或 (None, 0)"""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return None, 0
    frames = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(frame)
    cap.release()
    return frames, len(frames)


# ============================================================================
# ROS2 节点
# ============================================================================

class FTVisualizerXYSimple(Node):
    """
    FT 雷达数据 XY 视图可视化节点（简化版，无航迹）

    发布话题：
      /ft/points_xy   PointCloud2   点云（按高度着色）
      /ft/colorbar    Image         高度色带
      /ft/frame_info  MarkerArray   帧 ID 文字标注
      /ft/ruler1      MarkerArray   坐标尺 1
      /ft/ruler2      MarkerArray   坐标尺 2
      /ft/video       Image         同步视频画面
    """

    _AUTO_SENTINEL = -9999.0

    def __init__(self):
        super().__init__('ft_visualizer_xy_simple')

        self.declare_parameter('points_csv',  POINTS_CSV)
        self.declare_parameter('fixed_frame', FIXED_FRAME)
        self.declare_parameter('rate',        RATE_HZ)
        self.declare_parameter('loop',        LOOP)
        self.declare_parameter('play_mode',   PLAY_MODE)
        self.declare_parameter('z_min',       Z_MIN)
        self.declare_parameter('z_max',       Z_MAX)
        self.declare_parameter('x_max',       X_MAX)
        self.declare_parameter('x_min',       X_MIN)
        self.declare_parameter('y_range',     Y_RANGE)
        self.declare_parameter('z_filter_min', Z_FILTER_MIN)
        self.declare_parameter('z_filter_max', Z_FILTER_MAX)
        self.declare_parameter('show_ruler_1',      SHOW_RULER_1)
        self.declare_parameter('ruler_1_axis',      RULER_1_AXIS)
        self.declare_parameter('ruler_1_offset',    RULER_1_OFFSET)
        self.declare_parameter('ruler_1_interval',  RULER_1_INTERVAL)
        self.declare_parameter('ruler_1_length',    RULER_1_LENGTH)
        self.declare_parameter('ruler_1_font',      RULER_1_FONT)
        self.declare_parameter('ruler_1_color',     RULER_1_COLOR)
        self.declare_parameter('show_ruler_2',      SHOW_RULER_2)
        self.declare_parameter('ruler_2_axis',      RULER_2_AXIS)
        self.declare_parameter('ruler_2_offset',    RULER_2_OFFSET)
        self.declare_parameter('ruler_2_interval',  RULER_2_INTERVAL)
        self.declare_parameter('ruler_2_length',    RULER_2_LENGTH)
        self.declare_parameter('ruler_2_font',      RULER_2_FONT)
        self.declare_parameter('ruler_2_color',     RULER_2_COLOR)
        self.declare_parameter('video_path',        VIDEO_PATH)

        self.points_csv  = self.get_parameter('points_csv').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.rate        = float(self.get_parameter('rate').value)
        self.loop        = bool(self.get_parameter('loop').value)
        self.play_mode   = self.get_parameter('play_mode').value
        z_min_param      = float(self.get_parameter('z_min').value)
        z_max_param      = float(self.get_parameter('z_max').value)
        self.x_max       = float(self.get_parameter('x_max').value)
        self.x_min       = float(self.get_parameter('x_min').value)
        self.y_range     = float(self.get_parameter('y_range').value)
        self.z_filter_min = float(self.get_parameter('z_filter_min').value)
        self.z_filter_max = float(self.get_parameter('z_filter_max').value)
        self.show_ruler_1      = bool(self.get_parameter('show_ruler_1').value)
        self.ruler_1_axis      = self.get_parameter('ruler_1_axis').value
        self.ruler_1_offset    = float(self.get_parameter('ruler_1_offset').value)
        self.ruler_1_interval  = float(self.get_parameter('ruler_1_interval').value)
        self.ruler_1_length    = float(self.get_parameter('ruler_1_length').value)
        self.ruler_1_font      = float(self.get_parameter('ruler_1_font').value)
        self.ruler_1_color     = [float(v) for v in self.get_parameter('ruler_1_color').value]
        self.show_ruler_2      = bool(self.get_parameter('show_ruler_2').value)
        self.ruler_2_axis      = self.get_parameter('ruler_2_axis').value
        self.ruler_2_offset    = float(self.get_parameter('ruler_2_offset').value)
        self.ruler_2_interval  = float(self.get_parameter('ruler_2_interval').value)
        self.ruler_2_length    = float(self.get_parameter('ruler_2_length').value)
        self.ruler_2_font      = float(self.get_parameter('ruler_2_font').value)
        self.ruler_2_color     = [float(v) for v in self.get_parameter('ruler_2_color').value]
        self.video_path        = self.get_parameter('video_path').value

        self.get_logger().info(f'加载点云：{self.points_csv}')
        self.points_by_frame = load_points_csv(self.points_csv)

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

        self._colorbar_img = create_colorbar_image(self.z_min, self.z_max)

        # 发布静态 TF：radar → map（identity），解决 RViz 坐标系转换报错
        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = self.fixed_frame
        tf_msg.transform.rotation.w = 1.0
        self._tf_static.sendTransform(tf_msg)

        self.pub_pts   = self.create_publisher(PointCloud2, '/ft/points_xy',  10)
        self.pub_cbar  = self.create_publisher(Image,       '/ft/colorbar',   10)
        self.pub_info  = self.create_publisher(MarkerArray, '/ft/frame_info', 10)
        self.pub_ruler1 = self.create_publisher(MarkerArray, '/ft/ruler1',    10)
        self.pub_ruler2 = self.create_publisher(MarkerArray, '/ft/ruler2',    10)
        self.pub_video = self.create_publisher(Image,       '/ft/video',      10)

        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().warn('cv_bridge 未安装，色带/视频将不发布')

        # 加载视频
        self._video_frames = None
        self._video_count = 0
        self._video_idx = 0
        self._video_timer = None
        if self.video_path and self.bridge is not None:
            self._video_frames, self._video_count = load_video_frames(self.video_path)
            if self._video_frames is not None and self._video_count > 0:
                self.get_logger().info(f'加载视频：{self.video_path}  ({self._video_count} 帧)')
            else:
                self.get_logger().warn(f'视频加载失败：{self.video_path}')

        self.index = 0
        self._publish_pending = (self.play_mode == 'manual')
        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)

        self.get_logger().info(f'播放模式：{self.play_mode}  帧率：{self.rate} Hz')
        if self.play_mode == 'auto' and self._video_frames is not None:
            # 视频帧率 = 视频总帧数 * 点云播放帧率 / 点云总帧数（与点云同步结束）
            video_fps = self._video_count * self.rate / len(self.frame_ids)
            self.get_logger().info(f'视频同步播放：{video_fps:.2f} Hz（{self._video_count} 帧）')
            self._video_timer = self.create_timer(1.0 / video_fps, self._on_video_timer)
        if self.play_mode == 'manual':
            self.get_logger().info('手动模式键盘控制（终端输入后回车）：'
                                   'n/Enter=下一帧  p=上一帧  r=重置  q=退出')
            self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
            self._stdin_thread.start()

    def _on_timer(self):
        if not self.frame_ids:
            return
        if self.play_mode == 'auto':
            if self.index >= len(self.frame_ids):
                if self.loop:
                    self.index = 0
                    self._video_idx = 0
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

    def _on_video_timer(self):
        """视频帧发布回调"""
        if self._video_frames is None or self._video_count == 0:
            return
        if self._video_idx >= self._video_count:
            return
        stamp = self.get_clock().now().to_msg()
        try:
            img_msg = self.bridge.cv2_to_imgmsg(self._video_frames[self._video_idx], encoding='bgr8')
            img_msg.header = create_header(self.fixed_frame, stamp)
            self.pub_video.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'视频帧发布失败：{e}')
        self._video_idx += 1

    def _publish_frame(self, idx: int):
        fid   = self.frame_ids[idx]
        stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f'发布帧 frameID={fid}  ({idx + 1}/{len(self.frame_ids)})')

        pts = self.points_by_frame[fid]   # [N, 3]: xpos, ypos, zpos

        # 空间范围过滤（含 Z 轴高度过滤）
        mask = ((pts[:, 0] >= self.x_min) & (pts[:, 0] <= self.x_max) &
                (pts[:, 1] >= -self.y_range) & (pts[:, 1] <= self.y_range) &
                (pts[:, 2] >= self.z_filter_min) & (pts[:, 2] <= self.z_filter_max))
        pts = pts[mask]

        colors = height_to_rgb_array(pts[:, 2], self.z_min, self.z_max)
        self.pub_pts.publish(create_pointcloud2_rgb(pts, colors, self.fixed_frame, stamp))

        if self.bridge is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(self._colorbar_img, encoding='bgr8')
                img_msg.header = create_header(self.fixed_frame, stamp)
                self.pub_cbar.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'色带发布失败：{e}')

        lifetime = 2.0 / self.rate + 0.5

        # 坐标尺 1
        if self.show_ruler_1:
            ruler_arr = create_ruler_markers(self.ruler_1_axis, self.ruler_1_offset,
                                              self.ruler_1_interval, self.ruler_1_length,
                                              self.ruler_1_font, self.ruler_1_color,
                                              self.fixed_frame, stamp)
            for m in ruler_arr.markers:
                m.lifetime = Duration(sec=int(lifetime), nanosec=int((lifetime % 1) * 1e9))
            self.pub_ruler1.publish(ruler_arr)

        # 坐标尺 2
        if self.show_ruler_2:
            ruler_arr = create_ruler_markers(self.ruler_2_axis, self.ruler_2_offset,
                                              self.ruler_2_interval, self.ruler_2_length,
                                              self.ruler_2_font, self.ruler_2_color,
                                              self.fixed_frame, stamp)
            for m in ruler_arr.markers:
                m.lifetime = Duration(sec=int(lifetime), nanosec=int((lifetime % 1) * 1e9))
            self.pub_ruler2.publish(ruler_arr)

        # 手动模式视频同步：按当前帧比例发布对应视频帧
        if self.play_mode == 'manual' and self._video_frames is not None and self._video_count > 0:
            vi = int(idx / max(len(self.frame_ids) - 1, 1) * (self._video_count - 1))
            try:
                img_msg = self.bridge.cv2_to_imgmsg(self._video_frames[vi], encoding='bgr8')
                img_msg.header = create_header(self.fixed_frame, stamp)
                self.pub_video.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'视频帧发布失败：{e}')
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

    def _stdin_loop(self):
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
        self.get_logger().info('FT XY Simple Visualizer 已停止')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = FTVisualizerXYSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
