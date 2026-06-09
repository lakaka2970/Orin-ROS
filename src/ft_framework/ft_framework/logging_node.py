#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 数据日志记录节点 (Logging)
================================================================================
集中订阅所有传感器数据和检测结果，按 FT_radar_dataset_requirement.md 规定的
目录结构和数据格式写入磁盘。

目录结构（仅前向中心雷达，无角雷达）:
  <dataset_root>/                             # 例: output/ft_dataset
  ├── ego_motion.csv                          # 自车运动，单文件追加
  ├── calibration/
  │   └── radar_front_center_ft.yaml          # 标定外参
  ├── pc_pcd_radar_front_center/             # 雷达点云 PCD（逐帧）
  │   └── <timestamp_us>.pcd
  ├── pc_csv_radar_front_center/             # 雷达点云 CSV（逐帧）
  │   └── <timestamp_us>.csv
  ├── obj_csv_radar/                         # 雷达目标 CSV（逐帧）
  │   └── <timestamp_us>.csv
  ├── camera_front_center/                   # 相机图像（逐帧）
  │   └── <timestamp_us>.jpg
  └── adc_data/                              # ADC 原始数据（逐帧，非spec但保留）
      └── <timestamp_us>.bin

规格:
  - 5 独立通道: ADC / Image / Det_List / Ego_Motion / Obj_List
  - 每个通道独立开关，支持运行时动态切换 (ros2 param set)
  - 帧数上限: ADC 100 帧, 其他 1000 帧（超过停止记录并告警）
  - 异步写入: 独立写入线程 + 队列
  - 文件名: {timestamp_us}.ext（微秒时间戳）
  - CSV: 逗号分隔，首行为列名 header

话题:
  订阅: /adc/raw_data             ft_radar_msgs/AdcRawData
        /camera/image_raw         sensor_msgs/Image
        /processing/radar/det_list  ft_radar_msgs/DetList
        /vehicle/ego_motion       ft_radar_msgs/EgoMotion
        /perception/objects       ft_radar_msgs/ObjList

作者: zhengyuan.liu
日期: 2026.6.9
================================================================================
"""

# ============================================================================
# ★ 用户配置区
# ============================================================================

# ---------- 全局参数 ----------
OUTPUT_DIR        = "output/ft_dataset"          # 输出根目录（相对于工作区根目录）
STATUS_INTERVAL   = 5.0                         # 状态输出间隔 (s)
FRAME_LIMIT_ADC   = 100                         # ADC 最大帧数
FRAME_LIMIT_OTHER = 1000                        # 其他通道最大帧数

# ---------- 5 个独立开关（默认全部开启） ----------
ENABLE_ADC        = True
ENABLE_IMAGE      = True
ENABLE_DET_LIST   = True
ENABLE_EGO_MOTION = True
ENABLE_OBJ_LIST   = True

# ---------- 标定文件 ----------
CALIBRATION_FILE = ""

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import os
import time
import threading
import queue
import array

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ft_radar_msgs.msg import AdcRawData, DetList, EgoMotion, ObjList

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from ft_framework.common import monotonic_us_stamp


# ============================================================================
# 时间戳工具
# ============================================================================

def get_timestamp_us(msg) -> int:
    """从 header.stamp 提取微秒时间戳"""
    return msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000


# ============================================================================
# 异步写入器
# ============================================================================

class AsyncWriter:
    """异步文件写入器。独立线程 + 队列，不阻塞 ROS2 主回调。"""

    def __init__(self):
        self._queue = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def enqueue(self, file_path: str, data: bytes, mode: str = 'wb'):
        self._queue.put((file_path, data, mode))

    def stop(self):
        self._stop_event.set()
        self._queue.put(None)
        self._thread.join(timeout=5)

    def _run(self):
        while not self._stop_event.is_set():
            try:
                item = self._queue.get(timeout=1)
                if item is None:
                    break
                fpath, data, mode = item
                os.makedirs(os.path.dirname(fpath), exist_ok=True)
                with open(fpath, mode) as f:
                    f.write(data)
            except queue.Empty:
                continue
            except Exception as e:
                print(f'[AsyncWriter] 写入失败 {fpath}: {e}')


# ============================================================================
# ROS2 节点
# ============================================================================

class LoggingNode(Node):
    """数据日志记录节点"""

    def __init__(self):
        super().__init__('logging_node')

        # ---- ROS2 参数 ----
        self.declare_parameter('output_dir', OUTPUT_DIR)
        self.declare_parameter('status_interval', STATUS_INTERVAL)
        self.declare_parameter('max_frames.adc', FRAME_LIMIT_ADC)
        self.declare_parameter('max_frames.image', FRAME_LIMIT_OTHER)
        self.declare_parameter('max_frames.det_list', FRAME_LIMIT_OTHER)
        self.declare_parameter('max_frames.ego_motion', FRAME_LIMIT_OTHER)
        self.declare_parameter('max_frames.obj_list', FRAME_LIMIT_OTHER)
        self.declare_parameter('enable_adc', ENABLE_ADC)
        self.declare_parameter('enable_image', ENABLE_IMAGE)
        self.declare_parameter('enable_det_list', ENABLE_DET_LIST)
        self.declare_parameter('enable_ego_motion', ENABLE_EGO_MOTION)
        self.declare_parameter('enable_obj_list', ENABLE_OBJ_LIST)
        self.declare_parameter('calibration_file', CALIBRATION_FILE)

        # 路径处理：相对路径相对于工作区根目录
        raw_dir = self.get_parameter('output_dir').value
        if os.path.isabs(raw_dir):
            self._root = raw_dir
        else:
            # 相对路径 → 基于当前工作目录
            self._root = os.path.abspath(raw_dir)
        self._status_interval = float(
            self.get_parameter('status_interval').value)
        self._calibration_file = self.get_parameter('calibration_file').value

        # ---- 帧数上限 ----
        self._max_frames = {
            'adc':        int(self.get_parameter('max_frames.adc').value),
            'image':      int(self.get_parameter('max_frames.image').value),
            'det_list':   int(self.get_parameter('max_frames.det_list').value),
            'ego_motion': int(self.get_parameter('max_frames.ego_motion').value),
            'obj_list':   int(self.get_parameter('max_frames.obj_list').value),
        }

        # ---- 创建子目录 ----
        self._dirs = {
            'adc':        os.path.join(self._root, 'adc_data'),
            'image':      os.path.join(self._root, 'camera_front_center'),
            'det_list_pcd': os.path.join(self._root, 'pc_pcd_radar_front_center'),
            'det_list_csv': os.path.join(self._root, 'pc_csv_radar_front_center'),
            'ego_motion': os.path.join(self._root),       # ego_motion.csv 在根
            'obj_list':   os.path.join(self._root, 'obj_csv_radar'),
            'calib':      os.path.join(self._root, 'calibration'),
        }
        for _, d in self._dirs.items():
            try:
                os.makedirs(d, exist_ok=True)
            except OSError as e:
                self.get_logger().error(f'无法创建目录 {d}: {e}')
                raise

        # ---- 异步写入器 ----
        self._writer = AsyncWriter()

        # ---- 帧计数器 ----
        self._frame_counts = {k: 0 for k in self._max_frames}
        self._frame_reached_limit = {k: False for k in self._max_frames}

        # ---- cv_bridge ----
        self._bridge = CvBridge() if CvBridge is not None else None
        if self._bridge is None:
            self.get_logger().warning('cv_bridge 未安装，Image 录制不可用')

        # ---- ego_motion.csv 初始化 ----
        self._init_ego_csv()

        # ---- 5 通道订阅 ----
        self.sub_adc = self.create_subscription(
            AdcRawData, '/adc/raw_data', self._on_adc, 10)
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, 10)
        self.sub_det = self.create_subscription(
            DetList, '/processing/radar/det_list', self._on_det_list, 10)
        self.sub_ego = self.create_subscription(
            EgoMotion, '/vehicle/ego_motion', self._on_ego, 10)
        self.sub_obj = self.create_subscription(
            ObjList, '/perception/objects', self._on_obj, 10)

        # ---- 标定文件复制 ----
        if self._calibration_file and os.path.exists(self._calibration_file):
            import shutil
            dst = os.path.join(
                self._dirs['calib'], 'radar_front_center_ft.yaml')
            shutil.copy2(self._calibration_file, dst)
            self.get_logger().info(f'标定文件已复制 → {dst}')

        # ---- 状态定时器 ----
        self.create_timer(self._status_interval, self._on_status)
        self._start_time = time.time()

        self.get_logger().info(
            f'Logging 启动 → {self._root} | '
            f'ADC上限={self._max_frames["adc"]}, '
            f'其他上限={self._max_frames["image"]}')

    # ==================================================================
    # 5 通道数据回调
    # ==================================================================

    def _on_adc(self, msg: AdcRawData):
        """ADC → adc_data/<timestamp_us>.bin（逐帧二进制）"""
        if not self._get_switch('enable_adc', 'adc'):
            return
        ts = get_timestamp_us(msg)
        # 帧头: 8B 时间戳 + 12B 元数据
        hdr = ts.to_bytes(8, 'little')
        hdr += msg.num_rows.to_bytes(4, 'little')
        hdr += msg.num_chirps_per_row.to_bytes(4, 'little')
        hdr += msg.num_samples_per_chirp.to_bytes(4, 'little')
        payload = array.array('h', msg.data).tobytes()
        fpath = os.path.join(self._dirs['adc'], f'{ts}.bin')
        self._writer.enqueue(fpath, hdr + payload, 'wb')
        self._frame_counts['adc'] += 1

    def _on_image(self, msg: Image):
        """Image → camera_front_center/<timestamp_us>.jpg"""
        if not self._get_switch('enable_image', 'image') or self._bridge is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ts = get_timestamp_us(msg)
            import cv2
            success, jpg = cv2.imencode('.jpg', cv_img)
            if success:
                fpath = os.path.join(self._dirs['image'], f'{ts}.jpg')
                self._writer.enqueue(fpath, jpg.tobytes(), 'wb')
                self._frame_counts['image'] += 1
        except Exception as e:
            self.get_logger().error(f'Image 写入失败: {e}')

    def _on_det_list(self, msg: DetList):
        """
        DetList → 同时输出:
          pc_pcd_radar_front_center/<timestamp_us>.pcd
          pc_csv_radar_front_center/<timestamp_us>.csv
        """
        if not self._get_switch('enable_det_list', 'det_list'):
            return
        ts = get_timestamp_us(msg)
        n = len(msg.points)
        if n == 0:
            return

        # ---- PCD (v0.7 ASCII) ----
        pcd_lines = [
            '# .PCD v0.7 - Point Cloud Data file format',
            'VERSION 0.7',
            'FIELDS x y z range azimuth elevation RCS SNR ambgt '
            'exist_prob multi_tgt_prob ambgt_prob raw_doppler idx',
            'SIZE 4 4 4 4 4 4 4 4 4 1 1 1 4 1',
            'TYPE F F F F F F F F F U U U F U',
            'COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1',
            f'WIDTH {n}',
            'HEIGHT 1',
            'VIEWPOINT 0 0 0 1 0 0 0',
            f'POINTS {n}',
            'DATA ascii',
        ]
        for p in msg.points:
            pcd_lines.append(
                f'{p.x} {p.y} {p.z} {p.range} {p.azimuth} {p.elevation} '
                f'{p.rcs} {p.snr} {p.ambgt} '
                f'{p.exist_prob} {p.multi_tgt_prob} {p.ambgt_prob} '
                f'{p.raw_doppler} {p.idx}')

        pcd_path = os.path.join(self._dirs['det_list_pcd'], f'{ts}.pcd')
        self._writer.enqueue(pcd_path, '\n'.join(pcd_lines).encode(), 'wb')

        # ---- CSV（逗号分隔） ----
        csv_lines = [
            'x,y,z,range,azimuth,elevation,RCS,SNR,ambgt,'
            'exist_prob,multi_tgt_prob,ambgt_prob,raw_doppler,idx'
        ]
        for p in msg.points:
            csv_lines.append(
                f'{p.x},{p.y},{p.z},{p.range},{p.azimuth},{p.elevation},'
                f'{p.rcs},{p.snr},{p.ambgt},'
                f'{p.exist_prob},{p.multi_tgt_prob},{p.ambgt_prob},'
                f'{p.raw_doppler},{p.idx}')

        csv_path = os.path.join(self._dirs['det_list_csv'], f'{ts}.csv')
        self._writer.enqueue(csv_path, '\n'.join(csv_lines).encode(), 'wb')

        self._frame_counts['det_list'] += 1

    def _on_ego(self, msg: EgoMotion):
        """EgoMotion → ego_motion.csv（单文件追加，逗号分隔）"""
        if not self._get_switch('enable_ego_motion', 'ego_motion'):
            return
        ts = get_timestamp_us(msg)
        line = (f'{ts},{msg.vx},{msg.yaw_rate},{msg.steering_angle},'
                f'{msg.ax},{msg.ay},{msg.gear}\n')
        fpath = os.path.join(self._dirs['ego_motion'], 'ego_motion.csv')
        self._writer.enqueue(fpath, line.encode(), 'ab')
        self._frame_counts['ego_motion'] += 1

    def _on_obj(self, msg: ObjList):
        """ObjList → obj_csv_radar/<timestamp_us>.csv（逗号分隔）"""
        if not self._get_switch('enable_obj_list', 'obj_list'):
            return
        ts = get_timestamp_us(msg)
        if len(msg.objects) == 0:
            return

        csv_lines = [
            'object_id,tracked_times,score,x,y,z,l,w,h,yaw,'
            'vx_absolute,vy_absolute,vz_absolute,moving_state'
        ]
        for obj in msg.objects:
            csv_lines.append(
                f'{obj.object_id},{obj.tracked_times},{obj.score},'
                f'{obj.x},{obj.y},{obj.z},{obj.l},{obj.w},{obj.h},{obj.yaw},'
                f'{obj.vx_absolute},{obj.vy_absolute},{obj.vz_absolute},'
                f'{obj.moving_state}')

        fpath = os.path.join(self._dirs['obj_list'], f'{ts}.csv')
        self._writer.enqueue(fpath, '\n'.join(csv_lines).encode(), 'wb')
        self._frame_counts['obj_list'] += 1

    # ==================================================================
    # 开关控制
    # ==================================================================

    def _get_switch(self, param_name: str, channel: str) -> bool:
        """
        双重检查: ① 通道开关 ② 帧数上限
        支持 bool 和 str 类型参数值（Launch 传入的为字符串）
        """
        val = self.get_parameter(param_name).value
        if isinstance(val, str):
            enabled = val.lower() in ('true', '1', 'yes')
        else:
            enabled = bool(val)
        if not enabled:
            return False
        if self._frame_reached_limit[channel]:
            return False
        fc = self._frame_counts[channel]
        if fc >= self._max_frames[channel]:
            if not self._frame_reached_limit[channel]:
                self._frame_reached_limit[channel] = True
                self.get_logger().warning(
                    f'{channel} 已达上限 ({self._max_frames[channel]}帧)，停止录制')
            return False
        return True

    # ==================================================================
    # ego_motion.csv 初始化
    # ==================================================================

    def _init_ego_csv(self):
        """ego_motion.csv 写入 CSV 表头"""
        try:
            fpath = os.path.join(self._dirs['ego_motion'], 'ego_motion.csv')
            os.makedirs(os.path.dirname(fpath), exist_ok=True)
            header = 'timestamp_us,vx,yaw_rate,steering_angle,ax,ay,gear\n'
            with open(fpath, 'w') as f:
                f.write(header)
            self.get_logger().info(f'ego_motion.csv → {fpath}')
        except OSError as e:
            self.get_logger().error(f'ego_motion.csv 初始化失败: {e}')

    # ==================================================================
    # 状态输出
    # ==================================================================

    def _on_status(self):
        elapsed = time.time() - self._start_time
        parts = [f'{k}={v}/{self._max_frames[k]}'
                 for k, v in self._frame_counts.items()]
        self.get_logger().info(
            f'[状态] {elapsed:.0f}s | ' + ' | '.join(parts))

    # ==================================================================
    # 销毁
    # ==================================================================

    def destroy_node(self):
        self._on_status()
        self._writer.stop()
        self.get_logger().info('Logging 已停止')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Logging 收到中断信号')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
