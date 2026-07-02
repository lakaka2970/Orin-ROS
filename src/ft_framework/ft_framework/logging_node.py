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
输出结构 (对齐 FT FVR60_XD Radar Dataset Requirement)
────────────────────────────────────────────────────────────────────────────────
  <dataset_root>/                           # 默认: output/ft_dataset
  ├── ego_motion.csv                        # §3  自车运动
  ├── calibration/
  │   └── radar_front_center_ft.yaml        # §4  标定
  ├── pc_pcd_radar_front_center/           # §5  PCD v0.7, 19 字段
  ├── pc_csv_radar_front_center/           # §6  CSV, 19 字段
  ├── rdCell_csv_radar_front_center/       # §7  RD Cell List CSV
  ├── rxNci_bin_radar_front_center/        # §8  RX NCI BIN
  ├── obj_csv_radar/                       # §9  3D 目标 CSV
  ├── camera_front_center/                 # §10 图像
  └── adc_data/                            # 内部: ADC 原始数据

通道      开关参数            话题                          最大帧  输出
──────── ────────────────── ────────────────────────────  ────── ─────────────
ADC      enable_adc          /adc/raw_data                 100   <ts>.bin
Image    enable_image        /camera/image_raw            1000   <ts>.jpg
DetList  enable_det_list     /processing/radar/det_list   1000   <ts>.pcd+csv
DetCUDA  enable_det_list     /processing/radar/det_list_cuda 1000 <ts>_cuda.*
RnNci    enable_rn_nci       /processing/radar/rn_nci_data  1000  <ts>.csv+.bin
RnNci_CU enable_rn_nci      /processing/radar/rn_nci_data_cuda 1000 <ts>_cuda.*
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

# ── 帧数上限 (达到后循环覆盖, 删除旧文件保留最新 N 帧) ──
FRAME_LIMIT_ADC   = 100                  # ADC: 32 MiB/帧 × 100 = ~3.2 GB
FRAME_LIMIT_OTHER = 1000                 # 其他 5 个通道统一上限

# ── 通道开关 (True = 录制, False = 静默) ──
#     运行时切换:  ros2 param set /logging_node enable_adc false
ENABLE_ADC        = True
ENABLE_IMAGE      = True
ENABLE_DET_LIST   = True
ENABLE_EGO_MOTION = True
ENABLE_OBJ_LIST   = True
ENABLE_RN_NCI     = True

# ── 标定文件源路径 (空字符串 = 不复制) ──
CALIBRATION_FILE = ""


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  2. 程序实现 (一般无需修改)                                              ║
# ╚═══════════════════════════════════════════════════════════════════════════╝

import os
import struct
import time
import threading
import queue
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ft_radar_msgs.msg import AdcRawData, DetList, EgoMotion, ObjList, RnNciData

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
    独立线程 + 有界队列, 将磁盘 I/O 从 ROS2 主回调剥离。

    入队方式:
      enqueue(path, bytes, mode)   —— 简单字节写入 (ego / rn_nci)
      enqueue_task(callable)       —— 可调用对象, 在写线程执行
                                      (构建 PCD/CSV 字符串 + 写入)
      enqueue_adc(...)             —— ADC 零拷贝写入 (header + data_array)
      enqueue_image(...)           —— Image JPEG 编码 + 写入 (卸载到写线程)
      enqueue_delete(path)         —— 延迟文件删除

    生命周期:  __init__() 启动 daemon 线程 → stop() 发送终止信号。

    优化 (2026.7.2):
      - 移除所有 os.makedirs() 重复调用 (目录已在 LoggingNode.__init__ 中创建)
      - Queue 设 maxsize=500 防止内存无限增长
      - ADC 写入用 struct.pack header + f.write(array.array) 零拷贝
      - Image 编码卸载到写线程执行
    """

    def __init__(self, maxsize: int = 500):
        self._queue = queue.Queue(maxsize=maxsize)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ── 入队 API ──

    def enqueue(self, file_path: str, data: bytes, mode: str = 'wb'):
        """入队: 原始字节写入。"""
        self._put(('write', (file_path, data, mode)))

    def enqueue_task(self, task):
        """入队: 可调用对象 (用于卸载字符串构建等重操作)。"""
        self._put(('task', task))

    def enqueue_adc(self, file_path: str, ts: int,
                    num_rows: int, num_chirps_per_row: int,
                    num_samples_per_chirp: int, data_array):
        """
        入队: ADC 零拷贝写入。
        data_array 为 array.array('B'), 支持 buffer protocol → f.write() 直接读取.
        msg 的引用由调用方持有 (保持在 _on_adc 的局部变量中, 闭包捕获).
        """
        hdr = struct.pack('<QIII', ts, num_rows, num_chirps_per_row, num_samples_per_chirp)
        self._put(('adc', (file_path, hdr, data_array)))

    def enqueue_image(self, file_path: str, msg, bridge):
        """入队: Image → JPEG 编码 + 写入 (卸载到写线程)。"""
        self._put(('image', (file_path, msg, bridge)))

    def enqueue_rn_nci_bin(self, file_path: str, data_array):
        """
        入队: RX NCI BIN 零拷贝写入.
        data_array 为 array.array('B'), 支持 buffer protocol → f.write() 直接读取.
        无需 bytes() 转换, 避免 5MB 额外内存拷贝.
        """
        self._put(('rn_nci_bin', (file_path, data_array)))

    def enqueue_delete(self, file_path: str):
        """入队: 延迟文件删除 (忽略文件不存在)。"""
        self._put(('delete', file_path))

    # ── 内部 ──

    def _put(self, item):
        """入队: 满时丢弃最旧任务并打印 warning。"""
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            try:
                self._queue.get_nowait()  # 丢弃最旧
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                pass  # 极端情况, 放弃

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
                self._process_item(item)
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
                self._process_item(item)
            except queue.Empty:
                continue
            except Exception as e:
                print(f'[AsyncWriter] 任务失败: {e}')

    def _process_item(self, item):
        """统一处理队列项 (worker 线程和 stop drain 共用)。"""
        tag, payload = item
        if tag == 'write':
            fpath, data, mode = payload
            with open(fpath, mode) as f:
                f.write(data)
        elif tag == 'task':
            payload()
        elif tag == 'adc':
            fpath, hdr, data_array = payload
            with open(fpath, 'wb') as f:
                f.write(hdr)              # 20 字节 header
                f.write(data_array)       # 零拷贝: array.array buffer protocol
        elif tag == 'image':
            fpath, msg, bridge = payload
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, jpg = cv2.imencode('.jpg', cv_img)
            if success:
                with open(fpath, 'wb') as f:
                    f.write(jpg.tobytes())
        elif tag == 'delete':
            try:
                os.remove(payload)
            except OSError:
                pass
        elif tag == 'rn_nci_bin':
            fpath, data_array = payload
            with open(fpath, 'wb') as f:
                f.write(data_array)       # 零拷贝: array.array buffer protocol


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
        self.declare_parameter('max_frames.rn_nci',  FRAME_LIMIT_OTHER)
        self.declare_parameter('enable_adc',          ENABLE_ADC)
        self.declare_parameter('enable_image',        ENABLE_IMAGE)
        self.declare_parameter('enable_det_list',     ENABLE_DET_LIST)
        self.declare_parameter('enable_ego_motion',   ENABLE_EGO_MOTION)
        self.declare_parameter('enable_obj_list',     ENABLE_OBJ_LIST)
        self.declare_parameter('enable_rn_nci',       ENABLE_RN_NCI)
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
            'rn_nci':        int(self.get_parameter('max_frames.rn_nci').value),
            'rn_nci_cuda':   int(self.get_parameter('max_frames.rn_nci').value),
            'rdcell':        int(self.get_parameter('max_frames.rn_nci').value),
            'rdcell_cuda':   int(self.get_parameter('max_frames.rn_nci').value),
        }

        # 子目录映射  (key → <root>/<subdir>)
        self._dirs = {
            'adc':          os.path.join(self._root, 'adc_data'),
            'image':        os.path.join(self._root, 'camera_front_center'),
            'det_list_pcd': os.path.join(self._root, 'pc_pcd_radar_front_center'),
            'det_list_csv': os.path.join(self._root, 'pc_csv_radar_front_center'),
            'ego_motion':   os.path.join(self._root),     # ego_motion.csv 在根
            'obj_list':     os.path.join(self._root, 'obj_csv_radar'),
            'rn_nci':       os.path.join(self._root, 'rxNci_bin_radar_front_center'),
            'rn_nci_cuda':  os.path.join(self._root, 'rxNci_bin_radar_front_center'),
            'rdcell':       os.path.join(self._root, 'rdCell_csv_radar_front_center'),
            'rdcell_cuda':  os.path.join(self._root, 'rdCell_csv_radar_front_center'),
            'calib':        os.path.join(self._root, 'calibration'),
        }

        # 先确保根目录可写, 给出明确错误提示
        try:
            os.makedirs(self._root, exist_ok=True)
        except OSError as e:
            self.get_logger().fatal(
                f'无法创建输出根目录 {self._root}: {e}\n'
                f'  请检查目录权限: ls -la {self._root}\n'
                f'  若属主为 root, 执行: sudo chown -R $USER:$USER {self._root}\n'
                f'  或修改 output_dir 参数指向可写路径.')
            raise

        # 验证根目录可写
        test_file = os.path.join(self._root, '.ft_write_test')
        try:
            with open(test_file, 'w') as f:
                f.write('')
            os.remove(test_file)
        except OSError as e:
            self.get_logger().fatal(
                f'输出根目录不可写 {self._root}: {e}\n'
                f'  请检查目录权限: ls -la {self._root}\n'
                f'  若属主为 root, 执行: sudo chown -R $USER:$USER {self._root}')
            raise

        for _, d in self._dirs.items():
            try:
                os.makedirs(d, exist_ok=True)
            except OSError as e:
                self.get_logger().error(f'无法创建子目录 {d}: {e}')
                raise

        # 运行时状态 — 循环覆盖
        self._writer = AsyncWriter()
        self._frame_counts = {k: 0 for k in self._max_frames}
        # 每个通道维护一个文件路径 deque, 达到上限后删除最旧文件
        self._frame_files = {k: deque(maxlen=self._max_frames[k])
                             for k in self._max_frames}
        # ego_motion 行缓冲 (单文件追加, 满 N 行后重写)
        self._ego_lines: deque = deque(maxlen=self._max_frames['ego_motion'])
        self._ego_fpath = os.path.join(self._dirs['ego_motion'], 'ego_motion.csv')
        self._ego_dirty = False          # 批量写入: 有未刷新行时为 True
        self._ego_flush_every = 50       # 每 N 条消息触发一次文件重写
        self._ego_first_write = True     # 首条消息立即写盘, 不等待 timer

        # cv_bridge: Image → OpenCV → JPEG
        self._bridge = CvBridge() if CvBridge is not None else None
        if self._bridge is None:
            self.get_logger().warning('cv_bridge 未安装, Image 录制不可用')

        # ego_motion 表头
        self._init_ego_csv()

        # 订阅 — 全部使用 Best Effort (匹配 C++ 发布者).
        # 注意: 不能混用 Reliable, C++ rx 节点 (adc_rx, vehicle_data_rx,
        # camera_rx) 全部使用 Best Effort, DDS 要求 publisher/subscriber
        # reliability 一致, 否则无法匹配.
        _qos_be = rclpy.qos.QoSProfile(
            depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub_adc = self.create_subscription(
            AdcRawData, '/adc/raw_data',  self._on_adc,  _qos_be)
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw',    self._on_image, _qos_be)
        self.sub_det = self.create_subscription(
            DetList, '/processing/radar/det_list',
            self._on_det_list, _qos_be)
        self.sub_det_cu = self.create_subscription(
            DetList, '/processing/radar/det_list_cuda',
            self._on_det_list_cuda, _qos_be)
        self.sub_ego = self.create_subscription(
            EgoMotion, '/vehicle/ego_motion', self._on_ego, _qos_be)
        self.sub_obj = self.create_subscription(
            ObjList, '/perception/objects',   self._on_obj, _qos_be)
        self.sub_rn_nci = self.create_subscription(
            RnNciData, '/processing/radar/rn_nci_data',
            self._on_rn_nci, _qos_be)
        self.sub_rn_nci_cuda = self.create_subscription(
            RnNciData, '/processing/radar/rn_nci_data_cuda',
            lambda msg: self._on_rn_nci(msg, source='_cuda'), _qos_be)

        # 标定文件 (如果指定了路径且存在)
        if self._calibration_file and os.path.exists(self._calibration_file):
            import shutil
            dst = os.path.join(self._dirs['calib'], 'radar_front_center_ft.yaml')
            shutil.copy2(self._calibration_file, dst)
            self.get_logger().info(f'标定文件已复制 → {dst}')

        # 状态定时器
        self.create_timer(self._status_interval, self._on_status)
        # ego_motion 定期刷新定时器 (0.5 秒, 首条已立即写盘, 定期兜底)
        self._ego_flush_timer = self.create_timer(0.5, self._on_ego_flush)
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

        优化: 零拷贝写入 — msg.data (array.array) 直接传给 AsyncWriter,
              header 用 struct.pack 一次构建, 不拼接不拷贝.
        """
        if not self._get_switch('enable_adc', 'adc'):
            return
        ts = get_timestamp_us(msg)
        fpath = os.path.join(self._dirs['adc'], f'{ts}.bin')
        self._writer.enqueue_adc(fpath, ts,
            msg.num_rows, msg.num_chirps_per_row,
            msg.num_samples_per_chirp, msg.data)
        self._register_frame_file('adc', fpath)
        self._frame_counts['adc'] += 1

    # ── 2.3.3  Image 回调 ────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        """
        相机图像 → camera_front_center/<ts>.jpg

        流程:  入队 Image msg 引用 → AsyncWriter 线程做 cv_bridge + imencode → 写盘
        依赖:  cv_bridge + python3-opencv

        优化: cv_bridge 和 JPEG 编码已卸载到 writer 线程, 回调本身仅入队.
        """
        if not self._get_switch('enable_image', 'image') or self._bridge is None:
            return
        ts = get_timestamp_us(msg)
        fpath = os.path.join(self._dirs['image'], f'{ts}.jpg')
        self._writer.enqueue_image(fpath, msg, self._bridge)
        self._register_frame_file('image', fpath)
        self._frame_counts['image'] += 1

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

        # 注册 PCD + CSV 两个文件路径 (循环覆盖用)
        pcd_path = os.path.join(self._dirs['det_list_pcd'], f'{ts}{source}.pcd')
        csv_path = os.path.join(self._dirs['det_list_csv'], f'{ts}{source}.csv')
        self._register_frame_file(channel, pcd_path)
        self._register_frame_file(channel, csv_path)

        self._writer.enqueue_task(self._build_det_list_task(ts, msg, source))
        self._frame_counts[channel] += 1

    def _on_det_list_cuda(self, msg: DetList):
        """CUDA 话题回调 → 文件名加 '_cuda' 后缀。"""
        self._on_det_list(msg, source='_cuda')

    def _build_det_list_task(self, ts: int, msg: DetList, source: str = ''):
        """构建 PCD + CSV 写入闭包 (对齐 spec §5.3 + §6.3, 19 字段)."""
        n = len(msg.points)
        suffix = source                       # '' 或 '_cuda'
        frame_id = msg.frame_id

        def _task():
            # ── PCD v0.7 ASCII (§5.3) ──
            pcd = [
                '# .PCD v0.7 - Point Cloud Data file format',
                'VERSION 0.7',
                'FIELDS x y z range speed azimuth_ang ele_ang '
                'snr_db rcs_db power_db obj_same_rv rd_cell_idx '
                'range_idx doppler_idx azimuth_idx elevation_idx '
                'peak_val sin_azim_snr_lin sin_elev_snr_lin',
                'SIZE 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4',
                'TYPE F F F F F F F F F F F F F F F F F F F',
                'COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1',
                f'WIDTH {n}',
                'HEIGHT 1',
                f'VIEWPOINT 0 0 0 1 0 0 0 {ts} {frame_id} {n}',
                f'POINTS {n}',
                'DATA ascii',
            ]
            for p in msg.points:
                pcd.append(
                    f'{p.x:.6f} {p.y:.6f} {p.z:.6f} '
                    f'{p.range:.6f} {p.speed:.6f} '
                    f'{p.azimuth_ang:.6f} {p.ele_ang:.6f} '
                    f'{p.snr_db:.6f} {p.rcs_db:.6f} {p.power_db:.6f} '
                    f'{p.obj_same_rv} {p.rd_cell_idx} '
                    f'{p.range_idx} {p.doppler_idx} '
                    f'{p.azimuth_idx} {p.elevation_idx} '
                    f'{p.peak_val} {p.sin_azim_snr_lin} {p.sin_elev_snr_lin}')
            pcd_path = os.path.join(
                self._dirs['det_list_pcd'], f'{ts}{suffix}.pcd')
            with open(pcd_path, 'wb') as f:
                f.write('\n'.join(pcd).encode())

            # ── CSV (§6.3) ──
            csv_hdr = (
                'u32TimeStamp,u16FrameID,u16DetObjNum,'
                'f32XPos,f32YPos,f32ZPos,'
                'f32Range,f32Speed,'
                'f32AzimuthAng,f32EleAng,'
                'f32SNRdB,f32RcsdB,f32PowerdB,'
                'u32ObjSameRV,'
                'u16RdCellIdx,u16RangeIdx,u16DopplerIdx,'
                'u8AzimuthIdx,u8ElevationIdx,'
                'u16PeakVal,u16SinAzimSNRLin,u16SinElevSNRLin'
            )
            csv = [csv_hdr]
            for p in msg.points:
                csv.append(
                    f'{ts},{frame_id},{n},'
                    f'{p.x:.6f},{p.y:.6f},{p.z:.6f},'
                    f'{p.range:.6f},{p.speed:.6f},'
                    f'{p.azimuth_ang:.6f},{p.ele_ang:.6f},'
                    f'{p.snr_db:.6f},{p.rcs_db:.6f},{p.power_db:.6f},'
                    f'{p.obj_same_rv},'
                    f'{p.rd_cell_idx},{p.range_idx},{p.doppler_idx},'
                    f'{p.azimuth_idx},{p.elevation_idx},'
                    f'{p.peak_val},{p.sin_azim_snr_lin},{p.sin_elev_snr_lin}')
            csv_path = os.path.join(
                self._dirs['det_list_csv'], f'{ts}{suffix}.csv')
            with open(csv_path, 'wb') as f:
                f.write('\n'.join(csv).encode())

        return _task

    # ── 2.3.5  EgoMotion 回调 ────────────────────────────────────────────

    def _on_ego(self, msg: EgoMotion):
        """
        EgoMotion → ego_motion.csv (单文件, 追加)

        CSV 列 (对齐 spec §3.2):
          timestamp_us (uint64)  vx yaw_rate steering_angle  ax ay gear

        优化: 首条消息立即写盘, 后续批量写入 (每 N 条或每 0.5s),
              兼顾短时运行的可靠性和长时运行的 I/O 效率.
        """
        if not self._get_switch('enable_ego_motion', 'ego_motion'):
            return
        ts = get_timestamp_us(msg)
        line = (f'{ts},'
                f'{msg.vx:.6f},{msg.yaw_rate:.6f},{msg.steering_angle:.6f},'
                f'{msg.ax:.6f},{msg.ay:.6f},{msg.gear}\n')

        self._ego_lines.append(line)
        self._frame_counts['ego_motion'] += 1
        self._ego_dirty = True

        # 首条消息立即写盘 (不等待 timer), 确保短时运行也能持久化
        if self._ego_first_write:
            self._ego_first_write = False
            self._writer.enqueue_task(self._build_ego_rewrite_task())
            self._ego_dirty = False
            return

        # 后续: 每 N 条消息触发一次文件重写
        if len(self._ego_lines) % self._ego_flush_every == 0:
            self._writer.enqueue_task(self._build_ego_rewrite_task())
            self._ego_dirty = False

    def _on_ego_flush(self):
        """定时刷新: 若缓冲中有未写入的行, 触发文件重写。"""
        if self._ego_dirty and len(self._ego_lines) > 0:
            self._writer.enqueue_task(self._build_ego_rewrite_task())
            self._ego_dirty = False

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

        fpath = os.path.join(self._dirs['obj_list'], f'{ts}.csv')
        self._register_frame_file('obj_list', fpath)

        self._writer.enqueue_task(self._build_obj_csv_task(ts, msg))
        self._frame_counts['obj_list'] += 1

    # ── 2.3.6b  RnNciData 回调 (RD Cell List + RX NCI) ───────────────────

    def _on_rn_nci(self, msg: RnNciData, source: str = ''):
        """
        RnNciData → RD Cell List CSV (§7.3) + RX NCI BIN (§8.3).

        RD Cell CSV 列 (按 spec §7.3, 数组/复数展开):
          u32FrameTimeStamp, u16FrameId, u16NofRdCell, u8Index_Idletime,
          u16Rb, u16Db,
          f32PowRbNci_Q7dB-0, f32PowRbNci_Q7dB-1, f32PowRbNci_Q7dB-2,
          f32PowDbNci_Q7dB-0, f32PowDbNci_Q7dB-1, f32PowDbNci_Q7dB-2,
          f32PeakPowVchNci_Q7dB, f32NoiseNci_Q7dB,
          u8RdValidFlag, u8RdPeakFlag,
          sVch-0_r, sVch-0_im, sVch-1_r, sVch-1_im, ..., sVch-255_r, sVch-255_im

        RX NCI BIN: 原始 float32 二维数组 (rx_nci_rows × rx_nci_cols)
        """
        channel = 'rn_nci_cuda' if source == '_cuda' else 'rn_nci'
        if not self._get_switch('enable_rn_nci', channel):
            return

        ts = get_timestamp_us(msg)
        suffix = source                       # '' 或 '_cuda'
        nc = msg.num_cells

        # ---- RD Cell List CSV (§7.3) ----
        if nc > 0:
            rdcell_channel = 'rdcell_cuda' if source == '_cuda' else 'rdcell'
            if self._get_switch('enable_rn_nci', rdcell_channel):
                rdcell_csv = os.path.join(self._dirs[rdcell_channel],
                                          f'{ts}{suffix}.csv')
                self._register_frame_file(rdcell_channel, rdcell_csv)
                self._writer.enqueue_task(
                    self._build_rdcell_csv_task(ts, msg, suffix, rdcell_channel))
                self._frame_counts[rdcell_channel] += 1

        # ---- RX NCI BIN (§8.3) —— 零拷贝写入 ----
        fpath_bin = os.path.join(self._dirs[channel], f'{ts}{suffix}.bin')
        self._writer.enqueue_rn_nci_bin(fpath_bin, msg.rx_nci_data)
        self._register_frame_file(channel, fpath_bin)
        self._frame_counts[channel] += 1

    def _build_rdcell_csv_task(self, ts: int, msg: RnNciData,
                                 suffix: str, channel: str):
        """构建 RD Cell List CSV 写入闭包 (spec §7.3, 数组展开 + 复数分离)."""

        nc = msg.num_cells

        # 构建 CSV header
        hdr_cols = [
            'u32FrameTimeStamp', 'u16FrameId', 'u16NofRdCell', 'u8Index_Idletime',
            'u16Rb', 'u16Db',
        ]
        # f32PowRbNci_Q7dB[3]
        for ch_idx in range(3):
            hdr_cols.append(f'f32PowRbNci_Q7dB-{ch_idx}')
        # f32PowDbNci_Q7dB[3]
        for ch_idx in range(3):
            hdr_cols.append(f'f32PowDbNci_Q7dB-{ch_idx}')
        hdr_cols += [
            'f32PeakPowVchNci_Q7dB', 'f32NoiseNci_Q7dB',
            'u8RdValidFlag', 'u8RdPeakFlag',
        ]
        # sVch[256] — complex int32: r=real, im=imag
        for ch_idx in range(256):
            hdr_cols.append(f'sVch-{ch_idx}_r')
            hdr_cols.append(f'sVch-{ch_idx}_im')

        def _task():
            rows = [','.join(hdr_cols)]
            # 解析 channel data bytes → int32 pairs per cell
            ch_bytes = bytes(msg.channel_data_bytes)
            cell_ch_stride = 256 * 2 * 4  # 256 complex × 2 int32 × 4 bytes
            for i in range(nc):
                row = [
                    str(msg.frame_timestamp_us),
                    str(msg.frame_id),
                    str(nc),
                    str(msg.idle_time_idx),
                    str(msg.rb_list[i]),
                    str(msg.db_list[i]),
                    f'{msg.pow_rb_nci_0[i]:.3f}',
                    f'{msg.pow_rb_nci_1[i]:.3f}',
                    f'{msg.pow_rb_nci_2[i]:.3f}',
                    f'{msg.pow_db_nci_0[i]:.3f}',
                    f'{msg.pow_db_nci_1[i]:.3f}',
                    f'{msg.pow_db_nci_2[i]:.3f}',
                    f'{msg.peak_power_list[i]:.3f}',
                    f'{msg.noise_power_list[i]:.3f}',
                    str(msg.valid_flag_list[i]),
                    str(msg.peak_flag_list[i]),
                ]
                # 解析 sVch[256]
                import struct
                offset = i * cell_ch_stride
                for j in range(256):
                    r_val = struct.unpack_from('<i', ch_bytes, offset + j * 8)[0]
                    im_val = struct.unpack_from('<i', ch_bytes, offset + j * 8 + 4)[0]
                    row.append(str(r_val))
                    row.append(str(im_val))
                rows.append(','.join(row))

            csv_path = os.path.join(self._dirs[channel], f'{ts}{suffix}.csv')
            with open(csv_path, 'wb') as f:
                f.write('\n'.join(rows).encode())

        return _task

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
            with open(fpath, 'wb') as f:
                f.write('\n'.join(csv).encode())

        return _task

    # ── 2.3.7  双重门控 ──────────────────────────────────────────────────

    def _get_switch(self, param_name: str, channel: str) -> bool:
        """
        检查通道是否允许录制。

        门控 1 — 开关:  读取 ROS2 参数, str→bool 转换
                        'true'/'1'/'yes' → True, 其余 → False
        门控 2 — 帧数:  已达上限 → 循环覆盖 (删除最旧文件, 保留最新 N 帧)

        返回:  True = 允许录制, False = 跳过
        """
        # 门 1: 通道开关
        val = self.get_parameter(param_name).value
        enabled = val.lower() in ('true', '1', 'yes') \
                  if isinstance(val, str) else bool(val)
        if not enabled:
            return False

        # 门 2: 帧数上限 → 循环覆盖
        limit = self._max_frames[channel]
        if self._frame_counts[channel] >= limit:
            self._frame_counts[channel] = 0  # 重置计数器
            self.get_logger().warning(
                f'{channel} 已达上限 ({limit} 帧), '
                f'循环覆盖 — 旧文件将被删除')
        return True

    def _register_frame_file(self, channel: str, filepath: str):
        """记录帧文件路径; deque 满时异步删除最旧文件。"""
        files = self._frame_files[channel]
        if len(files) >= files.maxlen and files:
            oldest = files[0]
            self._writer.enqueue_delete(oldest)
        files.append(filepath)

    def _build_ego_rewrite_task(self):
        """构建 ego_motion.csv 重写闭包 (AsyncWriter 线程执行)。

        将行缓冲完整重写到文件, 保证文件只保留最新 N 行。
        优化: 直接传 deque 引用 (writer 线程串行执行, 无竞争).
        """
        lines = self._ego_lines  # deque 引用, 无需 list() 快照拷贝

        def _task():
            header = 'timestamp_us,vx,yaw_rate,steering_angle,ax,ay,gear\n'
            with open(self._ego_fpath, 'w') as f:
                f.write(header)
                f.writelines(lines)

        return _task

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
        """输出最终状态 → 刷新 ego 缓冲 → 停止写线程 → 父类清理。"""
        self._on_status()
        # 最终刷新 ego_motion 未写入的行
        if self._ego_dirty and len(self._ego_lines) > 0:
            self._ego_dirty = False
            self._writer.enqueue_task(self._build_ego_rewrite_task())
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
