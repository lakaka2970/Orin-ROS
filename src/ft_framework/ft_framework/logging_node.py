#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 数据日志记录节点 (Logging)
================================================================================
集中订阅所有传感器数据和检测结果，支持 5 个独立开关控制各通道录制。

规格:
  - 5 独立通道: ADC / Image / Det_List / Ego_Motion / Obj_List
  - 每个通道独立开关，支持运行时动态切换 (ros2 param set)
  - 帧数上限: ADC 100 帧, 其他 1000 帧（超过停止记录并告警）
  - 异步写入: 独立写入线程 + 队列
  - 时间戳对齐: 以 ADC 帧时钟为主基准（如 ADC 未启用则使用 ROS 时间）

话题:
  订阅: /adc/raw_data             ft_radar_msgs/AdcRawData
        /camera/image_raw         sensor_msgs/Image
        /processing/radar/det_list  ft_radar_msgs/DetList
        /vehicle/ego_motion       ft_radar_msgs/EgoMotion
        /perception/objects       ft_radar_msgs/ObjList

输出文件:
  adc.bin             ADC 原始数据（二进制连续存储）
  {timestamp}.jpg     相机图像帧
  {timestamp}.csv     det_list 雷达检测列表
  {timestamp}.pcd     det_list 雷达检测点云
  ego_motion.csv      自车运动数据（单文件追加）
  {timestamp}.csv     obj_list 3D 目标列表

连接关系:
  ← ADC Rx (sub)
  ← Camera Rx (sub)
  ← RSP MIL Python (sub)
  ← Vehicle Data Rx (sub)
  ← 3D Object Detection (sub)

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 全局日志参数 ----------
OUTPUT_DIR       = "/data/ft_radar_dataset"   # 输出根目录（实际部署时修改）
STATUS_INTERVAL  = 5.0                        # 状态输出间隔 (s)
FRAME_LIMIT_ADC  = 100                        # ADC 最大帧数
FRAME_LIMIT_OTHER = 1000                      # 其他通道最大帧数

# ---------- 5 个独立开关（默认全部开启） ----------
ENABLE_ADC        = True
ENABLE_IMAGE      = True
ENABLE_DET_LIST   = True
ENABLE_EGO_MOTION = True
ENABLE_OBJ_LIST   = True

# ---------- 标定文件 ----------
CALIBRATION_FILE = ""                          # 标定文件路径，空=不加载

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import os
import time
import threading
import queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ft_radar_msgs.msg import AdcRawData, DetList, EgoMotion, ObjList

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


# ============================================================================
# 时间戳工具函数
# ============================================================================

from ft_framework.common import monotonic_us_stamp


def get_timestamp_us(msg) -> int:
    """
    从消息的 header.stamp 获取微秒时间戳。
    ROS2 Header.stamp 为 (sec, nanosec)，组合为微秒:
      timestamp_us = sec * 1_000_000 + nanosec / 1_000
    """
    return msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000


# ============================================================================
# 异步写入器
# ============================================================================

class AsyncWriter:
    """
    异步文件写入器。

    使用独立线程 + 队列，不阻塞 ROS2 主回调。
    """

    def __init__(self, output_dir: str):
        self._output_dir = output_dir
        self._queue = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)

    def enqueue(self, file_name: str, data: bytes, mode: str = 'wb'):
        """提交写入任务到队列（线程安全）"""
        self._queue.put((file_name, data, mode))

    def stop(self):
        """停止写入线程"""
        self._stop_event.set()
        self._queue.put(None)      # 哨兵
        self._thread.join(timeout=5)

    def _run(self):
        """后台写入线程"""
        while not self._stop_event.is_set():
            try:
                item = self._queue.get(timeout=1)
                if item is None:
                    break
                fname, data, mode = item
                fpath = os.path.join(self._output_dir, fname)
                with open(fpath, mode) as f:
                    f.write(data)
            except queue.Empty:
                continue
            except Exception as e:
                print(f'[AsyncWriter] 写入失败 {fname}: {e}')


# ============================================================================
# ROS2 节点
# ============================================================================

class LoggingNode(Node):
    """
    数据日志记录节点

    5 个独立开关分别控制各通道录制:
      enable_adc / enable_image / enable_det_list / enable_ego_motion / enable_obj_list

    所有开关支持运行时动态切换:
      ros2 param set /logging_node enable_adc false

    帧数上限:
      ADC: 100 帧，其他: 1000 帧
      达到上限后停止记录并输出警告日志
    """

    def __init__(self):
        super().__init__('logging_node')

        # ---------- ROS2 参数声明 ----------
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

        self._output_dir = self.get_parameter('output_dir').value
        self._status_interval = float(
            self.get_parameter('status_interval').value)
        self._calibration_file = self.get_parameter('calibration_file').value

        # ---------- 帧数上限 ----------
        self._max_frames = {
            'adc':        int(self.get_parameter('max_frames.adc').value),
            'image':      int(self.get_parameter('max_frames.image').value),
            'det_list':   int(self.get_parameter('max_frames.det_list').value),
            'ego_motion': int(self.get_parameter('max_frames.ego_motion').value),
            'obj_list':   int(self.get_parameter('max_frames.obj_list').value),
        }

        # ---------- 异步写入器 ----------
        self._writer = AsyncWriter(self._output_dir)

        # ---------- 帧计数器 ----------
        self._frame_counts = {k: 0 for k in self._max_frames}
        self._frame_reached_limit = {k: False for k in self._max_frames}

        # ---------- cv_bridge（用于 image→jpg） ----------
        self._bridge = CvBridge() if CvBridge is not None else None
        if self._bridge is None:
            self.get_logger().warn('cv_bridge 未安装，Image 录制不可用')

        # ---------- ego_motion.csv 初始化 ----------
        self._init_ego_csv()

        # ---------- 5 通道订阅 ----------
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

        # ---------- 标定文件处理 ----------
        if self._calibration_file and os.path.exists(self._calibration_file):
            import shutil
            dst = os.path.join(self._output_dir, 'calibration.yaml')
            shutil.copy2(self._calibration_file, dst)
            self.get_logger().info(f'标定文件已复制: {self._calibration_file} → {dst}')

        # ---------- 状态定时器 ----------
        self.create_timer(self._status_interval, self._on_status)
        self._start_time = time.time()

        self.get_logger().info(
            f'Logging 节点启动 | 输出: {self._output_dir} | '
            f'帧上限: ADC={self._max_frames["adc"]}, '
            f'其他={self._max_frames["image"]}')

    # ------------------------------------------------------------------
    # 5 通道数据回调
    # ------------------------------------------------------------------

    def _on_adc(self, msg: AdcRawData):
        """ADC 原始数据 → adc.bin（二进制追加）"""
        if not self._get_switch('enable_adc', 'adc'):
            return

        ts = get_timestamp_us(msg)
        # 帧头: 8 bytes 时间戳 + 12 bytes 元数据 + 数据
        header = ts.to_bytes(8, 'little')
        header += msg.num_chirps.to_bytes(4, 'little')
        header += msg.num_rx_antennas.to_bytes(4, 'little')
        header += msg.num_samples_per_chirp.to_bytes(4, 'little')

        import struct
        data_array = struct.pack(f'<{len(msg.data)}h', *msg.data)

        self._writer.enqueue('adc.bin', header + data_array, 'ab')
        self._frame_counts['adc'] += 1

    def _on_image(self, msg: Image):
        """相机图像 → {timestamp_us}.jpg"""
        if not self._get_switch('enable_image', 'image') or self._bridge is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ts = get_timestamp_us(msg)
            import cv2
            success, jpg_data = cv2.imencode('.jpg', cv_img)
            if success:
                self._writer.enqueue(f'{ts}.jpg', jpg_data.tobytes(), 'wb')
                self._frame_counts['image'] += 1
        except Exception as e:
            self.get_logger().error(f'Image 写入失败: {e}')

    def _on_det_list(self, msg: DetList):
        """检测列表 → {timestamp_us}.csv + {timestamp_us}.pcd"""
        if not self._get_switch('enable_det_list', 'det_list'):
            return
        ts = get_timestamp_us(msg)
        n = len(msg.points)
        if n == 0:
            return

        # ---- CSV ----
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
        self._writer.enqueue(f'{ts}.csv', '\n'.join(csv_lines).encode(), 'wb')

        # ---- PCD (ASCII) ----
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
        self._writer.enqueue(f'{ts}.pcd', '\n'.join(pcd_lines).encode(), 'wb')

        self._frame_counts['det_list'] += 1

    def _on_ego(self, msg: EgoMotion):
        """自车运动 → ego_motion.csv（单文件追加）"""
        if not self._get_switch('enable_ego_motion', 'ego_motion'):
            return
        ts = get_timestamp_us(msg)
        line = f'{ts},{msg.vx},{msg.yaw_rate},{msg.steering_angle},' \
               f'{msg.ax},{msg.ay},{msg.gear}\n'
        self._writer.enqueue('ego_motion.csv', line.encode(), 'ab')
        self._frame_counts['ego_motion'] += 1

    def _on_obj(self, msg: ObjList):
        """3D 目标列表 → {timestamp_us}.csv"""
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
        self._writer.enqueue(f'{ts}.csv', '\n'.join(csv_lines).encode(), 'wb')
        self._frame_counts['obj_list'] += 1

    # ------------------------------------------------------------------
    # 开关控制辅助
    # ------------------------------------------------------------------

    def _get_switch(self, param_name: str, channel: str) -> bool:
        """
        统一检查:
          1. 通道开关是否开启
          2. 帧数是否达到上限
        返回 True = 可以录制
        """
        if not self.get_parameter(param_name).value:
            return False
        if self._frame_reached_limit[channel]:
            return False
        frame_count = self._frame_counts[channel]
        max_frames = self._max_frames[channel]
        if frame_count >= max_frames:
            if not self._frame_reached_limit[channel]:
                self._frame_reached_limit[channel] = True
                self.get_logger().warn(
                    f'{channel} 已达帧数上限 ({max_frames})，停止录制')
            return False
        return True

    # ------------------------------------------------------------------
    # ego_motion.csv 初始化
    # ------------------------------------------------------------------

    def _init_ego_csv(self):
        """初始化 ego_motion.csv，写入 CSV 表头"""
        output_dir = self._output_dir
        os.makedirs(output_dir, exist_ok=True)
        csv_path = os.path.join(output_dir, 'ego_motion.csv')
        header = 'timestamp_us,vx,yaw_rate,steering_angle,ax,ay,gear\n'
        with open(csv_path, 'w') as f:
            f.write(header)
        self.get_logger().info(f'ego_motion.csv 初始化完成: {csv_path}')

    # ------------------------------------------------------------------
    # 状态输出
    # ------------------------------------------------------------------

    def _on_status(self):
        """定期输出录制统计"""
        elapsed = time.time() - self._start_time
        parts = [f'{k}={v}/{self._max_frames[k]}'
                 for k, v in self._frame_counts.items()]
        self.get_logger().info(
            f'[Logging 状态] 运行 {elapsed:.1f}s | ' + ' | '.join(parts))

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

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
        node.get_logger().info('Logging 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
