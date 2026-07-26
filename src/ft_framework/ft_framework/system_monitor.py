#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
system_monitor.py — 系统监控节点 (V2 架构)
================================================================================
以 1 Hz 频率发布 SystemMonitor 消息, 涵盖:
  - 磁盘 I/O (读写速率、利用率)
  - 内存占用 (总量、已用、可用、百分比)
  - 进程状态 (CPU%、内存%)
  - 时钟漂移 (ROS Time vs System Time)
  - 帧周期统计 (ADC/Camera 帧间隔均值、标准差、丢帧数)

话题:
  发布: /system/monitor (SystemMonitor, 1 Hz)
  订阅: /adc/file_path (AdcFilePath) — 用于帧周期统计
        /camera/file_path (CameraFilePath) — 用于帧周期统计

作者: zhengyuan.liu
日期: 2026-07-26
================================================================================
"""

import os
import time
from collections import deque

import rclpy
from rclpy.node import Node

from ft_radar_msgs.msg import AdcFilePath, CameraFilePath, SystemMonitor


class SystemMonitorNode(Node):

    def __init__(self):
        super().__init__('system_monitor')

        self.declare_parameter('monitor_hz', 1.0)
        self.declare_parameter('frame_history_size', 100)
        self.declare_parameter('adc_expected_period_ms', 66.0)
        self.declare_parameter('camera_expected_period_ms', 66.0)
        self.declare_parameter('drop_threshold_factor', 1.5)

        self.monitor_hz = float(self.get_parameter('monitor_hz').value)
        history_size = int(self.get_parameter('frame_history_size').value)
        self.adc_expected_ms = float(self.get_parameter('adc_expected_period_ms').value)
        self.cam_expected_ms = float(self.get_parameter('camera_expected_period_ms').value)
        self.drop_factor = float(self.get_parameter('drop_threshold_factor').value)

        # 帧时间戳历史
        self._adc_timestamps = deque(maxlen=history_size)
        self._cam_timestamps = deque(maxlen=history_size)
        self._adc_drop_count = 0
        self._cam_drop_count = 0

        # 磁盘 I/O 统计
        self._last_disk_io = None
        self._last_disk_time = None

        # 订阅
        qos = rclpy.qos.QoSProfile(
            depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(AdcFilePath, '/adc/file_path', self._on_adc, qos)
        self.create_subscription(CameraFilePath, '/camera/file_path', self._on_cam, qos)

        # 发布
        self.pub = self.create_publisher(SystemMonitor, '/system/monitor', 10)

        # 定时器
        self.create_timer(1.0 / self.monitor_hz, self._on_timer)

        self.get_logger().info(f'System Monitor 启动: {self.monitor_hz} Hz')

    def _on_adc(self, msg: AdcFilePath):
        ts_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        self._adc_timestamps.append(ts_us)
        self._check_drops(self._adc_timestamps, self.adc_expected_ms, 'adc')

    def _on_cam(self, msg: CameraFilePath):
        ts_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        self._cam_timestamps.append(ts_us)
        self._check_drops(self._cam_timestamps, self.cam_expected_ms, 'cam')

    def _check_drops(self, timestamps, expected_ms, label):
        if len(timestamps) < 2:
            return
        interval_us = timestamps[-1] - timestamps[-2]
        threshold_us = expected_ms * 1000 * self.drop_factor
        if interval_us > threshold_us:
            if label == 'adc':
                self._adc_drop_count += 1
            else:
                self._cam_drop_count += 1

    def _on_timer(self):
        msg = SystemMonitor()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'system'

        # 磁盘 I/O
        self._fill_disk_io(msg)

        # 内存
        self._fill_memory(msg)

        # 进程
        self._fill_processes(msg)

        # 时钟漂移
        self._fill_clock_drift(msg)

        # 帧周期统计
        self._fill_frame_stats(msg)

        self.pub.publish(msg)

    def _fill_disk_io(self, msg):
        try:
            with open('/proc/diskstats', 'r') as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 14 and parts[2] in ('nvme0n1', 'mmcblk0'):
                        read_sectors = int(parts[5])
                        write_sectors = int(parts[9])
                        io_ticks = int(parts[12])
                        now = time.monotonic()

                        if self._last_disk_io is not None:
                            dt = now - self._last_disk_time
                            if dt > 0:
                                dr = (read_sectors - self._last_disk_io[0]) * 512 / dt / 1e6
                                dw = (write_sectors - self._last_disk_io[1]) * 512 / dt / 1e6
                                msg.disk_read_rate = dr
                                msg.disk_write_rate = dw
                                msg.disk_utilization = min(100.0,
                                    (io_ticks - self._last_disk_io[2]) / dt / 10.0)

                        self._last_disk_io = (read_sectors, write_sectors, io_ticks)
                        self._last_disk_time = now
                        break
        except (IOError, IndexError, ValueError):
            pass

    def _fill_memory(self, msg):
        try:
            with open('/proc/meminfo', 'r') as f:
                info = {}
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        info[parts[0].rstrip(':')] = int(parts[1])

                total_kb = info.get('MemTotal', 0)
                avail_kb = info.get('MemAvailable', 0)
                used_kb = total_kb - avail_kb

                msg.memory_total = total_kb // 1024
                msg.memory_used = used_kb // 1024
                msg.memory_available = avail_kb // 1024
                msg.memory_percent = (used_kb / total_kb * 100.0) if total_kb > 0 else 0.0
        except (IOError, ValueError):
            pass

    def _fill_processes(self, msg):
        try:
            import psutil
            procs = []
            for p in psutil.process_iter(['name', 'cpu_percent', 'memory_percent']):
                name = p.info.get('name', '')
                if any(k in name for k in ('adc_rx', 'camera_rx', 'vehicle', 'rsp', 'rviz')):
                    procs.append(p)

            msg.process_count = len(procs)
            msg.process_names = [p.info['name'] for p in procs]
            msg.process_cpu_percents = [p.info.get('cpu_percent', 0.0) for p in procs]
            msg.process_memory_percents = [p.info.get('memory_percent', 0.0) for p in procs]
        except ImportError:
            msg.process_count = 0
        except Exception:
            msg.process_count = 0

    def _fill_clock_drift(self, msg):
        ros_now = self.get_clock().now()
        sys_now = time.time()
        ros_sec = ros_now.nanoseconds / 1e9
        msg.clock_drift_us = (ros_sec - sys_now) * 1e6

    def _fill_frame_stats(self, msg):
        msg.adc_frame_period_mean, msg.adc_frame_period_std = \
            self._compute_stats(self._adc_timestamps)
        msg.adc_frame_drop_count = self._adc_drop_count

        msg.camera_frame_period_mean, msg.camera_frame_period_std = \
            self._compute_stats(self._cam_timestamps)
        msg.camera_frame_drop_count = self._cam_drop_count

    @staticmethod
    def _compute_stats(timestamps):
        if len(timestamps) < 2:
            return 0.0, 0.0
        intervals = []
        ts_list = list(timestamps)
        for i in range(1, len(ts_list)):
            intervals.append((ts_list[i] - ts_list[i-1]) / 1000.0)  # us → ms
        if not intervals:
            return 0.0, 0.0
        mean = sum(intervals) / len(intervals)
        variance = sum((x - mean) ** 2 for x in intervals) / len(intervals)
        return mean, variance ** 0.5


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
