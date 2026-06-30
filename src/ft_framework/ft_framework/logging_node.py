#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 数据日志记录节点 (Logging)
================================================================================
集中订阅所有传感器数据和检测结果，按 FT_radar_dataset_requirement.md 规定的
目录结构和数据格式写入磁盘。

────────────────────────────────────────────────────────────────────────────────
快速导航
────────────────────────────────────────────────────────────────────────────────
  L 50:   用户配置区    (输出路径 / 帧数上限 / 通道开关 / 标定文件)
  L 95:   get_timestamp_us()     —— 从 ROS2 Header.stamp 提取微秒整数
  L108:   AsyncWriter            —— 异步文件写入器 (独立线程 + 队列)
  L155:   LoggingNode.__init__() —— 初始化 (参数 / 目录 / 订阅 / 定时器)
  L265:   _on_adc()              —— ADC 原始数据 → adc_data/<ts>.bin
  L279:   _on_image()            —— 相机图像     → camera_front_center/<ts>.jpg
  L295:   _on_det_list()         —— 雷达点云     → .pcd + .csv 双格式
  L323:   _build_det_list_task() —— PCD v0.7 / CSV 构建闭包 (异步线程)
  L372:   _on_ego()              —— 自车运动     → ego_motion.csv (追加)
  L384:   _on_obj()              —— 3D 目标      → obj_csv_radar/<ts>.csv
  L427:   _get_switch()          —— 双重门控 (开关 + 帧数上限)
  L454:   _init_ego_csv()        —— ego_motion 表头初始化
  L469:   _on_status()           —— 定期进度输出
  L481:   destroy_node()         —— 安全清理

────────────────────────────────────────────────────────────────────────────────
输出结构 (仅前向中心雷达)
────────────────────────────────────────────────────────────────────────────────
  <dataset_root>/                           # 默认: output/ft_dataset
  ├── ego_motion.csv                        # 单文件, 逐行追加
  ├── calibration/
  │   └── radar_front_center_ft.yaml        # 从外部路径复制
  ├── pc_pcd_radar_front_center/           # PCD v0.7 ASCII, 14 字段
  ├── pc_csv_radar_front_center/           # CSV, 逗号分隔, 14 列
  ├── obj_csv_radar/                       # CSV, 逗号分隔, 14 列
  ├── camera_front_center/                 # JPEG 图像
  └── adc_data/                            # 自定义二进制 (非 spec)

通道      开关参数            话题                          最大帧  输出
──────── ────────────────── ────────────────────────────  ────── ─────────────
ADC      enable_adc          /adc/raw_data                 100   <ts>.bin
Image    enable_image        /camera/image_raw            1000   <ts>.jpg
DetList  enable_det_list     /processing/radar/det_list   1000   <ts>.pcd+csv
DetCUDA  enable_det_list     /processing/radar/det_list_cuda 1000 <ts>_cuda.*
Ego      enable_ego_motion   /vehicle/ego_motion          1000   ego_motion.csv
Obj      enable_obj_list     /perception/objects          1000   <ts>.csv

作者: zhengyuan.liu
日期: 2026.6.12
================================================================================
"""

# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  1. 用户配置区                                                           ║
# ║     修改以下参数后重启节点生效，也可通过 YAML 或 Launch 参数覆盖。        ║
# ╚═══════════════════════════════════════════════════════════════════════════╝

# ── 输出路径 & 状态间隔 ──
OUTPUT_DIR      = "output/ft_dataset"    # 数据集根目录 (相对路径 = 相对 CWD)
STATUS_INTERVAL = 5.0                    # 状态日志输出间隔 (秒)

# ── 帧数上限 (达到即停止, 不循环覆盖) ──
FRAME_LIMIT_ADC   = 100                  # ADC: 16 MiB/帧 × 100 = ~1.6 GB
FRAME_LIMIT_OTHER = 1000                 # 其他 5 个通道统一上限

# ── 通道开关 (True = 录制, False = 静默) ──
#     运行时切换:  ros2 param set /logging_node enable_adc false
ENABLE_ADC        = True
ENABLE_IMAGE      = True
ENABLE_DET_LIST   = True
ENABLE_EGO_MOTION = True
ENABLE_OBJ_LIST   = True

# ── 标定文件源路径 (空字符串 = 不复制) ──
CALIBRATION_FILE = ""


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  2. 程序实现 (一般无需修改)                                              ║
# ╚═══════════════════════════════════════════════════════════════════════════╝

import os
import time
import threading
import queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ft_radar_msgs.msg import AdcRawData, DetList, EgoMotion, ObjList

# cv_bridge 的 pybind11 模块初始化依赖 cv2 先加载
import cv2

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from ft_framework.common import monotonic_us_stamp


# ═══════════════════════════════════════════════════════════════════════════
# 2.1  get_timestamp_us — 微秒时间戳提取
# ═══════════════════════════════════════════════════════════════════════════

def get_timestamp_us(msg) -> int:
    """
    从 ROS2 Header.stamp 提取微秒时间戳。

    公式:  sec × 1,000,000 + nsec ÷ 1,000
    用途:  所有回调中作为文件名和 CSV 第一列 timestamp_us。
    """
    return msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000


# ═══════════════════════════════════════════════════════════════════════════
# 2.2  AsyncWriter — 异步文件写入器
# ═══════════════════════════════════════════════════════════════════════════

class AsyncWriter:
    """
    独立线程 + 队列, 将磁盘 I/O 从 ROS2 主回调剥离。

    两种入队方式:
      enqueue(path, bytes, mode) —— 简单字节写入 (ADC / Image / ego)
      enqueue_task(callable)      —— 可调用对象, 在写线程执行
                                    (构建 PCD/CSV 字符串 + 写入)

    生命周期:  __init__() 启动 daemon 线程 → stop() 发送终止信号。
    """

    def __init__(self):
        self._queue = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ── 入队 API ──

    def enqueue(self, file_path: str, data: bytes, mode: str = 'wb'):
        """入队: 原始字节写入。"""
        self._queue.put(('write', (file_path, data, mode)))

    def enqueue_task(self, task):
        """入队: 可调用对象 (用于卸载字符串构建等重操作)。"""
        self._queue.put(('task', task))

    # ── 生命周期 ──

    def stop(self):
        """终止写线程: 先排空队列再发送终止信号，避免数据丢失。"""
        self._stop_event.set()
        # 先排空队列中剩余的任务
        while True:
            try:
                item = self._queue.get(timeout=0.1)
                if item is None:
                    break
                tag, payload = item
                if tag == 'write':
                    fpath, data, mode = payload
                    os.makedirs(os.path.dirname(fpath), exist_ok=True)
                    with open(fpath, mode) as f:
                        f.write(data)
                elif tag == 'task':
                    payload()
            except queue.Empty:
                break
        # 发送终止信号
        self._queue.put(None)
        self._thread.join(timeout=5)

    # ── 工作循环 ──

    def _run(self):
        while not self._stop_event.is_set():
            try:
                item = self._queue.get(timeout=1)
                if item is None:              # 终止信号
                    break
                tag, payload = item
                if tag == 'write':
                    fpath, data, mode = payload
                    os.makedirs(os.path.dirname(fpath), exist_ok=True)
                    with open(fpath, mode) as f:
                        f.write(data)
                elif tag == 'task':
                    payload()
            except queue.Empty:
                continue
            except Exception as e:
                print(f'[AsyncWriter] 任务失败: {e}')


# ═══════════════════════════════════════════════════════════════════════════
# 2.3  LoggingNode — 数据日志记录节点
# ═══════════════════════════════════════════════════════════════════════════

class LoggingNode(Node):

    # ── 2.3.1  初始化 ────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('logging_node')

        # 参数声明  (优先级: 模块常量 < YAML < Launch 命令)
        self.declare_parameter('output_dir',         OUTPUT_DIR)
        self.declare_parameter('status_interval',    STATUS_INTERVAL)
        self.declare_parameter('max_frames.adc',      FRAME_LIMIT_ADC)
        self.declare_parameter('max_frames.image',    FRAME_LIMIT_OTHER)
        self.declare_parameter('max_frames.det_list', FRAME_LIMIT_OTHER)
        self.declare_parameter('max_frames.ego_motion', FRAME_LIMIT_OTHER)
        self.declare_parameter('max_frames.obj_list', FRAME_LIMIT_OTHER)
        self.declare_parameter('enable_adc',          ENABLE_ADC)
        self.declare_parameter('enable_image',        ENABLE_IMAGE)
        self.declare_parameter('enable_det_list',     ENABLE_DET_LIST)
        self.declare_parameter('enable_ego_motion',   ENABLE_EGO_MOTION)
        self.declare_parameter('enable_obj_list',     ENABLE_OBJ_LIST)
        self.declare_parameter('calibration_file',    CALIBRATION_FILE)

        # 路径: 绝对路径不变, 相对路径用 CWD 解析
        raw_dir = self.get_parameter('output_dir').value
        self._root = raw_dir if os.path.isabs(raw_dir) \
                     else os.path.abspath(raw_dir)
        self._status_interval  = float(self.get_parameter('status_interval').value)
        self._calibration_file = self.get_parameter('calibration_file').value

        # 帧数上限 (det_list_cuda 与 det_list 共享同一上限值)
        self._max_frames = {
            'adc':           int(self.get_parameter('max_frames.adc').value),
            'image':         int(self.get_parameter('max_frames.image').value),
            'det_list':      int(self.get_parameter('max_frames.det_list').value),
            'det_list_cuda': int(self.get_parameter('max_frames.det_list').value),
            'ego_motion':    int(self.get_parameter('max_frames.ego_motion').value),
            'obj_list':      int(self.get_parameter('max_frames.obj_list').value),
        }

        # 子目录映射  (key → <root>/<subdir>)
        self._dirs = {
            'adc':          os.path.join(self._root, 'adc_data'),
            'image':        os.path.join(self._root, 'camera_front_center'),
            'det_list_pcd': os.path.join(self._root, 'pc_pcd_radar_front_center'),
            'det_list_csv': os.path.join(self._root, 'pc_csv_radar_front_center'),
            'ego_motion':   os.path.join(self._root),     # ego_motion.csv 在根
            'obj_list':     os.path.join(self._root, 'obj_csv_radar'),
            'calib':        os.path.join(self._root, 'calibration'),
        }
        for _, d in self._dirs.items():
            try:
                os.makedirs(d, exist_ok=True)
            except OSError as e:
                self.get_logger().error(f'无法创建目录 {d}: {e}')
                raise

        # 运行时状态
        self._writer = AsyncWriter()
        self._frame_counts        = {k: 0     for k in self._max_frames}
        self._frame_reached_limit = {k: False for k in self._max_frames}

        # cv_bridge: Image → OpenCV → JPEG
        self._bridge = CvBridge() if CvBridge is not None else None
        if self._bridge is None:
            self.get_logger().warning('cv_bridge 未安装, Image 录制不可用')

        # ego_motion 表头
        self._init_ego_csv()

        # 订阅 (6 路) — Best Effort 匹配 C++ 发布者
        _qos = rclpy.qos.QoSProfile(depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub_adc = self.create_subscription(
            AdcRawData, '/adc/raw_data',  self._on_adc,  _qos)
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw',    self._on_image, _qos)
        self.sub_det = self.create_subscription(
            DetList, '/processing/radar/det_list',
            self._on_det_list, _qos)
        self.sub_det_cu = self.create_subscription(
            DetList, '/processing/radar/det_list_cuda',
            self._on_det_list_cuda, _qos)
        self.sub_ego = self.create_subscription(
            EgoMotion, '/vehicle/ego_motion', self._on_ego, _qos)
        self.sub_obj = self.create_subscription(
            ObjList, '/perception/objects',   self._on_obj, _qos)

        # 标定文件 (如果指定了路径且存在)
        if self._calibration_file and os.path.exists(self._calibration_file):
            import shutil
            dst = os.path.join(self._dirs['calib'], 'radar_front_center_ft.yaml')
            shutil.copy2(self._calibration_file, dst)
            self.get_logger().info(f'标定文件已复制 → {dst}')

        # 状态定时器
        self.create_timer(self._status_interval, self._on_status)
        self._start_time = time.time()

        self.get_logger().info(
            f'Logging 启动 → {self._root} | '
            f'ADC 上限={self._max_frames["adc"]}, '
            f'其他上限={self._max_frames["image"]}')

    # ── 2.3.2  ADC 回调 ──────────────────────────────────────────────────

    def _on_adc(self, msg: AdcRawData):
        """
        ADC 原始数据 → adc_data/<ts>.bin

        自定义二进制格式:
          [ 0- 7] timestamp_us           uint64 LE
          [ 8-11] num_rows               uint32 LE
          [12-15] num_chirps_per_row     uint32 LE
          [16-19] num_samples_per_chirp  uint32 LE
          [20-  ] int16[] payload        (rows × chirps × samples)
        """
        if not self._get_switch('enable_adc', 'adc'):
            return
        ts = get_timestamp_us(msg)

        hdr  = ts.to_bytes(8, 'little')
        hdr += msg.num_rows.to_bytes(4, 'little')
        hdr += msg.num_chirps_per_row.to_bytes(4, 'little')
        hdr += msg.num_samples_per_chirp.to_bytes(4, 'little')

        payload = bytes(msg.data)
        fpath = os.path.join(self._dirs['adc'], f'{ts}.bin')
        self._writer.enqueue(fpath, hdr + payload, 'wb')
        self._frame_counts['adc'] += 1

    # ── 2.3.3  Image 回调 ────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        """
        相机图像 → camera_front_center/<ts>.jpg

        流程:  ROS Image → cv_bridge → BGR8 → cv2.imencode('.jpg') → 异步写盘
        依赖:  cv_bridge + python3-opencv
        """
        if not self._get_switch('enable_image', 'image') or self._bridge is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ts = get_timestamp_us(msg)
            success, jpg = cv2.imencode('.jpg', cv_img)
            if success:
                fpath = os.path.join(self._dirs['image'], f'{ts}.jpg')
                self._writer.enqueue(fpath, jpg.tobytes(), 'wb')
                self._frame_counts['image'] += 1
        except Exception as e:
            self.get_logger().error(f'Image 写入失败: {e}')

    # ── 2.3.4  DetList 回调 (点云) ───────────────────────────────────────

    def _on_det_list(self, msg: DetList, source: str = ''):
        """
        DetList → PCD + CSV 双格式输出 (异步构建)。

        文件:  pc_pcd_radar_front_center/<ts>{source}.pcd
               pc_csv_radar_front_center/<ts>{source}.csv

        source: ''      = 主话题 (单路 / both 中的 Python)
                '_cuda' = CUDA 专属话题 (both / both_compare)
        """
        channel = 'det_list_cuda' if source == '_cuda' else 'det_list'
        if not self._get_switch('enable_det_list', channel):
            return
        ts = get_timestamp_us(msg)
        if len(msg.points) == 0:
            return

        self._writer.enqueue_task(self._build_det_list_task(ts, msg, source))
        self._frame_counts[channel] += 1

    def _on_det_list_cuda(self, msg: DetList):
        """CUDA 话题回调 → 文件名加 '_cuda' 后缀。"""
        self._on_det_list(msg, source='_cuda')

    def _build_det_list_task(self, ts: int, msg: DetList, source: str = ''):
        """
        构建 PCD + CSV 写入闭包, 返回零参数 callable。

        在 AsyncWriter 线程中执行, 将字符串构建和磁盘写入
        从 ROS2 主回调卸载。
        """
        n = len(msg.points)
        suffix = source                       # '' 或 '_cuda'

        def _task():
            # ── PCD v0.7 ASCII ──
            # 字段顺序对齐 spec §5.3:
            #   x y z  range azimuth elevation  RCS SNR ambgt
            #   exist_prob multi_tgt_prob ambgt_prob  raw_doppler idx
            pcd = [
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
            # float32 → .6f (防科学计数法), uint8 → 默认整数
            for p in msg.points:
                pcd.append(
                    f'{p.x:.6f} {p.y:.6f} {p.z:.6f} '
                    f'{p.range:.6f} {p.azimuth:.6f} {p.elevation:.6f} '
                    f'{p.rcs:.6f} {p.snr:.6f} {p.ambgt:.6f} '
                    f'{p.exist_prob} {p.multi_tgt_prob} {p.ambgt_prob} '
                    f'{p.raw_doppler:.6f} {p.idx}')
            pcd_path = os.path.join(
                self._dirs['det_list_pcd'], f'{ts}{suffix}.pcd')
            os.makedirs(os.path.dirname(pcd_path), exist_ok=True)
            with open(pcd_path, 'wb') as f:
                f.write('\n'.join(pcd).encode())

            # ── CSV ──
            csv = [
                'x,y,z,range,azimuth,elevation,RCS,SNR,ambgt,'
                'exist_prob,multi_tgt_prob,ambgt_prob,raw_doppler,idx'
            ]
            for p in msg.points:
                csv.append(
                    f'{p.x:.6f},{p.y:.6f},{p.z:.6f},'
                    f'{p.range:.6f},{p.azimuth:.6f},{p.elevation:.6f},'
                    f'{p.rcs:.6f},{p.snr:.6f},{p.ambgt:.6f},'
                    f'{p.exist_prob},{p.multi_tgt_prob},{p.ambgt_prob},'
                    f'{p.raw_doppler:.6f},{p.idx}')
            csv_path = os.path.join(
                self._dirs['det_list_csv'], f'{ts}{suffix}.csv')
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            with open(csv_path, 'wb') as f:
                f.write('\n'.join(csv).encode())

        return _task

    # ── 2.3.5  EgoMotion 回调 ────────────────────────────────────────────

    def _on_ego(self, msg: EgoMotion):
        """
        EgoMotion → ego_motion.csv (单文件, 追加)

        CSV 列 (对齐 spec §3.2):
          timestamp_us (uint64)  vx yaw_rate steering_angle  ax ay gear
        """
        if not self._get_switch('enable_ego_motion', 'ego_motion'):
            return
        ts = get_timestamp_us(msg)
        line = (f'{ts},'
                f'{msg.vx:.6f},{msg.yaw_rate:.6f},{msg.steering_angle:.6f},'
                f'{msg.ax:.6f},{msg.ay:.6f},{msg.gear}\n')
        fpath = os.path.join(self._dirs['ego_motion'], 'ego_motion.csv')
        self._writer.enqueue(fpath, line.encode(), 'ab')
        self._frame_counts['ego_motion'] += 1

    # ── 2.3.6  ObjList 回调 (3D 目标) ────────────────────────────────────

    def _on_obj(self, msg: ObjList):
        """
        ObjList → obj_csv_radar/<ts>.csv (异步构建)

        CSV 列 (对齐 spec §7.3):
          object_id, tracked_times, score,
          x, y, z, l, w, h, yaw,
          vx_absolute, vy_absolute, vz_absolute,
          moving_state
        """
        if not self._get_switch('enable_obj_list', 'obj_list'):
            return
        ts = get_timestamp_us(msg)
        if len(msg.objects) == 0:
            return

        self._writer.enqueue_task(self._build_obj_csv_task(ts, msg))
        self._frame_counts['obj_list'] += 1

    def _build_obj_csv_task(self, ts: int, msg: ObjList):
        """构建 ObjList CSV 写入闭包 (AsyncWriter 线程执行)。"""

        def _task():
            csv = [
                'object_id,tracked_times,score,x,y,z,l,w,h,yaw,'
                'vx_absolute,vy_absolute,vz_absolute,moving_state'
            ]
            for obj in msg.objects:
                csv.append(
                    f'{obj.object_id},{obj.tracked_times},'
                    f'{obj.score:.6f},'
                    f'{obj.x:.6f},{obj.y:.6f},{obj.z:.6f},'
                    f'{obj.l:.6f},{obj.w:.6f},{obj.h:.6f},{obj.yaw:.6f},'
                    f'{obj.vx_absolute:.6f},{obj.vy_absolute:.6f},'
                    f'{obj.vz_absolute:.6f},'
                    f'{obj.moving_state}')
            fpath = os.path.join(self._dirs['obj_list'], f'{ts}.csv')
            os.makedirs(os.path.dirname(fpath), exist_ok=True)
            with open(fpath, 'wb') as f:
                f.write('\n'.join(csv).encode())

        return _task

    # ── 2.3.7  双重门控 ──────────────────────────────────────────────────

    def _get_switch(self, param_name: str, channel: str) -> bool:
        """
        检查通道是否允许录制。

        门控 1 — 开关:  读取 ROS2 参数, str→bool 转换
                        'true'/'1'/'yes' → True, 其余 → False
        门控 2 — 帧数:  已达上限 → 自动停止并告警 (仅一次)

        返回:  True = 允许录制, False = 跳过
        """
        # 门 1: 通道开关
        val = self.get_parameter(param_name).value
        enabled = val.lower() in ('true', '1', 'yes') \
                  if isinstance(val, str) else bool(val)
        if not enabled:
            return False

        # 门 2: 帧数上限
        if self._frame_reached_limit[channel]:
            return False
        if self._frame_counts[channel] >= self._max_frames[channel]:
            self._frame_reached_limit[channel] = True
            self.get_logger().warning(
                f'{channel} 已达上限 ({self._max_frames[channel]} 帧), 停止录制')
            return False
        return True

    # ── 2.3.8  ego_motion 初始化 ─────────────────────────────────────────

    def _init_ego_csv(self):
        """创建 ego_motion.csv 并写入表头 (启动时执行一次)。"""
        try:
            fpath = os.path.join(self._dirs['ego_motion'], 'ego_motion.csv')
            os.makedirs(os.path.dirname(fpath), exist_ok=True)
            if os.path.exists(fpath):
                self.get_logger().warning(
                    f'ego_motion.csv 已存在, 将追加写入: {fpath}')
            with open(fpath, 'a') as f:
                # 仅新建文件时写入表头
                if f.tell() == 0:
                    f.write('timestamp_us,vx,yaw_rate,steering_angle,ax,ay,gear\n')
            self.get_logger().info(f'ego_motion.csv → {fpath}')
        except OSError as e:
            self.get_logger().error(f'ego_motion.csv 初始化失败: {e}')

    # ── 2.3.9  状态输出 ──────────────────────────────────────────────────

    def _on_status(self):
        """定时输出各通道录制进度。"""
        elapsed = time.time() - self._start_time
        parts = [f'{k}={v}/{self._max_frames[k]}'
                 for k, v in self._frame_counts.items()]
        self.get_logger().info(
            f'[状态] {elapsed:.0f}s | ' + ' | '.join(parts))

    # ── 2.3.10  销毁 ─────────────────────────────────────────────────────

    def destroy_node(self):
        """输出最终状态 → 停止写线程 → 父类清理。"""
        self._on_status()
        self._writer.stop()
        self.get_logger().info('Logging 已停止')
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════
# 3. 入口
# ═══════════════════════════════════════════════════════════════════════════

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
