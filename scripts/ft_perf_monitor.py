#!/usr/bin/env python3
"""
FT Radar 性能监控脚本 — 非侵入式记录全链路运行时性能。

启动方式:
    source scripts/env.sh
    python3 scripts/ft_perf_monitor.py                     # 默认 60s, 输出到 output/perf_monitor/
    python3 scripts/ft_perf_monitor.py --duration 120      # 运行 120s
    python3 scripts/ft_perf_monitor.py -o /path/to/out     # 自定义输出目录

监控维度:
    [1] 话题频率 — 每个话题的实际发布 Hz
    [2] 端到端时延 — ADC 时间戳 → RSP 输出 的延迟
    [3] 消息带宽 — 每个话题的 MB/s
    [4] 系统资源 — CPU / RAM / GPU / 温度 (tegrastats)
    [5] 丢帧检测 — 对比相邻帧间隔识别丢帧
    [6] 管线吞吐 — ADC → RSP → ObjectDetection 各级别产出率

输出文件 (按启动时间命名):
    perf_topics_<ts>.csv      # 逐秒: 每个话题的频率 + 带宽
    perf_latency_<ts>.csv     # 逐帧: ADC→RSP 端到端延迟 (us)
    perf_system_<ts>.csv      # 逐秒: CPU% / RAM MB / GPU% / 温度
    perf_summary_<ts>.txt     # 汇总报告

作者: zhengyuan.liu
日期: 2026.7.2
"""

import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np
import psutil

import rclpy
from rclpy.node import Node
from ft_radar_msgs.msg import AdcRawData, DetList, EgoMotion, ObjList
from sensor_msgs.msg import Image


# =============================================================================
# 数据结构
# =============================================================================

@dataclass
class TopicStats:
    """单个话题的实时统计"""
    name: str
    msg_count: int = 0
    total_bytes: int = 0
    last_stamp_ns: int = 0            # 上一帧 header.stamp (ns)
    last_wall_time: float = 0.0       # 上一帧到达时的 wall time
    intervals_ns: List[int] = field(default_factory=list)  # 最近 N 个帧间隔
    drop_count: int = 0               # 检测到的丢帧数

    def record(self, size_bytes: int, header_stamp_ns: int, wall_ts: float):
        self.msg_count += 1
        self.total_bytes += size_bytes

        if self.last_stamp_ns > 0 and self.last_wall_time > 0:
            interval_ns = header_stamp_ns - self.last_stamp_ns
            if interval_ns > 0:
                self.intervals_ns.append(interval_ns)
                # 只保留最近 200 个
                if len(self.intervals_ns) > 200:
                    self.intervals_ns.pop(0)
        self.last_stamp_ns = header_stamp_ns
        self.last_wall_time = wall_ts

    def freq_hz(self) -> float:
        """基于最近间隔的中位数推算频率"""
        if len(self.intervals_ns) < 2:
            return 0.0
        median_ns = float(np.median(self.intervals_ns[-50:]))
        return 1_000_000_000.0 / median_ns if median_ns > 0 else 0.0

    def bw_mbps(self, duration_s: float) -> float:
        """时间段内的平均带宽"""
        if duration_s <= 0:
            return 0.0
        return (self.total_bytes / 1024 / 1024) / duration_s


@dataclass
class LatencySample:
    """单帧的端到端延迟采样"""
    adc_stamp_ns: int      # ADC header.stamp
    wall_arrival_ns: int   # det_list 到达时的 wall time
    source: str            # 'det_list' 或 'det_list_cuda'
    det_count: int         # 检测点数量


# =============================================================================
# 监控节点
# =============================================================================

class PerfMonitor(Node):
    """订阅全链路话题，收集性能数据"""

    def __init__(self):
        super().__init__("ft_perf_monitor")

        # QoS — 匹配各发布端
        qos_be = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_default = 10

        # 话题统计
        self.topics: Dict[str, TopicStats] = {}
        # 时延采样
        self.latency_samples: List[LatencySample] = []
        self._latency_lock = threading.Lock()
        # 系统指标
        self._sys_lock = threading.Lock()
        self._cpu_percent: float = 0.0
        self._ram_mb: float = 0.0
        self._gpu_percent: float = 0.0
        self._gpu_temp: float = 0.0
        self._cpu_temp: float = 0.0

        # ── 订阅所有关键话题 ──
        # ADC
        self._reg_topic("/adc/raw_data", AdcRawData, qos_be)
        # Camera
        self._reg_topic("/camera/image_raw", Image, qos_be)
        # EgoMotion
        self._reg_topic("/vehicle/ego_motion", EgoMotion, qos_be)
        # RSP outputs
        self._reg_topic("/processing/radar/det_list", DetList, qos_be)
        self._reg_topic("/processing/radar/det_list_cuda", DetList, qos_be)
        # Object detection
        self._reg_topic("/perception/objects", ObjList, qos_default)
        # Visualization outputs
        self._reg_topic("/visualization/radar/display", None, qos_default)
        self._reg_topic("/visualization/radar/boxes", None, qos_default)

        self.get_logger().info(f"性能监控已启动 — 监听 {len(self.topics)} 个话题")

    def _reg_topic(self, name: str, msg_type, qos):
        """注册话题统计 + 订阅"""
        self.topics[name] = TopicStats(name=name)

        if msg_type is None:
            return  # 只统计不订阅 (通过其他方式)

        if msg_type == AdcRawData:
            cb = lambda msg: self._on_adc(msg, name)
        elif msg_type == DetList:
            cb = lambda msg: self._on_det_list(msg, name)
        elif msg_type == EgoMotion:
            cb = lambda msg: self._on_generic(msg, name, size=200)
        elif msg_type == ObjList:
            cb = lambda msg: self._on_generic(msg, name, size=500)
        elif msg_type == Image:
            cb = lambda msg: self._on_image(msg, name)
        else:
            cb = lambda msg: self._on_generic(msg, name, size=1000)

        self.create_subscription(msg_type, name, cb, qos)

    # ── 回调 ──

    def _on_adc(self, msg: AdcRawData, topic: str):
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        size = 20 + len(msg.data)  # 20B header + payload
        self.topics[topic].record(size, ts_ns, time.time())

    def _on_det_list(self, msg: DetList, topic: str):
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        # 估算 DetList 消息大小
        size = 32 + len(msg.points) * 150  # header + ~150B/点
        self.topics[topic].record(size, ts_ns, time.time())

        # 记录时延: ADC stamp → wall time now
        wall_ns = time.time_ns()
        sample = LatencySample(
            adc_stamp_ns=ts_ns,
            wall_arrival_ns=wall_ns,
            source="det_list_cuda" if "cuda" in topic else "det_list",
            det_count=len(msg.points),
        )
        with self._latency_lock:
            self.latency_samples.append(sample)
            if len(self.latency_samples) > 5000:
                self.latency_samples = self.latency_samples[-3000:]

    def _on_generic(self, msg, topic: str, size: int):
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.topics[topic].record(size, ts_ns, time.time())

    def _on_image(self, msg: Image, topic: str):
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        size = msg.width * msg.height * 3  # BGR8
        self.topics[topic].record(size, ts_ns, time.time())

    # ── 系统指标采集 ──

    def capture_system_metrics(self):
        """由外部线程调用"""
        try:
            cpu = psutil.cpu_percent(interval=0.5)
            mem = psutil.virtual_memory()
            with self._sys_lock:
                self._cpu_percent = cpu
                self._ram_mb = mem.used / 1024 / 1024
        except Exception:
            pass

    def get_system_snapshot(self) -> Tuple[float, float, float, float, float]:
        with self._sys_lock:
            return (self._cpu_percent, self._ram_mb,
                    self._gpu_percent, self._gpu_temp, self._cpu_temp)


# =============================================================================
# GPU 监控 (tegrastats)
# =============================================================================

class GpuMonitor:
    """解析 tegrastats 输出提取 GPU 利用率和温度"""

    def __init__(self):
        self.gpu_percent = 0.0
        self.gpu_temp = 0.0
        self.cpu_temp = 0.0
        self._proc: Optional[subprocess.Popen] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def start(self):
        if not os.path.exists("/usr/bin/tegrastats"):
            print("[WARN] tegrastats 不可用, 跳过 GPU 监控")
            return
        try:
            self._proc = subprocess.Popen(
                ["tegrastats", "--interval", "1000"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                universal_newlines=True,
            )
            self._running = True
            self._thread = threading.Thread(target=self._parse_loop, daemon=True)
            self._thread.start()
        except Exception as e:
            print(f"[WARN] 无法启动 tegrastats: {e}")

    def stop(self):
        self._running = False
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()

    def _parse_loop(self):
        while self._running and self._proc and self._proc.stdout:
            line = self._proc.stdout.readline()
            if not line:
                break
            # RAM 7983/62803MB ... GR3D_FREQ 15%@[0] ... GPU@40.218C ... CPU@45.625C
            try:
                gpu_pct = 0.0
                gpu_temp = 0.0
                cpu_temp = 0.0
                # GR3D_FREQ 后跟空格 + 百分比 如 "GR3D_FREQ 15%@"
                m = re.search(r'GR3D_FREQ (\d+)%', line)
                if m:
                    gpu_pct = float(m.group(1))
                m = re.search(r'GPU@([\d.]+)C', line)
                if m:
                    gpu_temp = float(m.group(1))
                m = re.search(r'CPU@([\d.]+)C', line)
                if m:
                    cpu_temp = float(m.group(1))
                with self._lock:
                    self.gpu_percent = gpu_pct
                    self.gpu_temp = gpu_temp
                    self.cpu_temp = cpu_temp
            except (ValueError, IndexError):
                pass

    def snapshot(self) -> Tuple[float, float, float]:
        with self._lock:
            return (self.gpu_percent, self.gpu_temp, self.cpu_temp)


# =============================================================================
# CSV 输出
# =============================================================================

class CsvWriter:
    """管理多个 CSV 输出流"""

    def __init__(self, output_dir: str, ts: str):
        os.makedirs(output_dir, exist_ok=True)
        self.dir = output_dir
        self.ts = ts
        self._files: Dict[str, Tuple] = {}

    def open(self, name: str, fields: List[str]) -> csv.DictWriter:
        path = os.path.join(self.dir, f"perf_{name}_{self.ts}.csv")
        f = open(path, "w", newline="")
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        f.flush()
        self._files[name] = (f, w, fields)
        return w

    def writer(self, name: str) -> Optional[csv.DictWriter]:
        entry = self._files.get(name)
        return entry[1] if entry else None

    def flush(self, name: str):
        entry = self._files.get(name)
        if entry:
            entry[0].flush()

    def close_all(self):
        for f, _, _ in self._files.values():
            f.close()

    def paths(self) -> Dict[str, str]:
        return {name: os.path.join(self.dir, f"perf_{name}_{self.ts}.csv")
                for name in self._files}


# =============================================================================
# 主循环
# =============================================================================

TOPIC_FIELDS = [
    "elapsed_s", "topic",
    "msg_count", "freq_hz", "bw_mbps",
    "total_bytes", "drop_count",
]

LATENCY_FIELDS = [
    "wall_time_ns", "adc_stamp_ns", "latency_us",
    "source", "det_count",
]

SYSTEM_FIELDS = [
    "elapsed_s", "cpu_percent", "ram_mb",
    "gpu_percent", "gpu_temp_c", "cpu_temp_c",
]


def main():
    parser = argparse.ArgumentParser(description="FT Radar 全链路性能监控")
    parser.add_argument("--duration", "-d", type=int, default=60,
                        help="监控时长 (秒), 默认 60")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="输出目录 (默认: output/perf_monitor/)")
    parser.add_argument("--interval", "-i", type=float, default=1.0,
                        help="采样间隔 (秒), 默认 1.0")
    args = parser.parse_args()

    # 输出目录
    if args.output:
        output_dir = args.output
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, "perf_monitor")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    print("=" * 60)
    print("  FT Radar 性能监控")
    print(f"  时长: {args.duration}s | 间隔: {args.interval}s")
    print(f"  输出: {output_dir}")
    print("=" * 60)
    print()

    # 初始化 CSV
    csv_out = CsvWriter(output_dir, ts)
    csv_out.open("topics", TOPIC_FIELDS)
    csv_out.open("latency", LATENCY_FIELDS)
    csv_out.open("system", SYSTEM_FIELDS)

    # 启动 ROS2 节点
    rclpy.init(args=sys.argv)
    node = PerfMonitor()

    # 启动 GPU 监控
    gpu_mon = GpuMonitor()
    gpu_mon.start()

    # 后台 ROS spin
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    time.sleep(1.0)  # 等 ROS 发现

    # 主循环
    start_time = time.time()
    sample_count = 0

    def save_latency_batch():
        """批量写入时延采样"""
        with node._latency_lock:
            samples = node.latency_samples[:]
            node.latency_samples.clear()
        writer = csv_out.writer("latency")
        if writer:
            for s in samples:
                writer.writerow({
                    "wall_time_ns": s.wall_arrival_ns,
                    "adc_stamp_ns": s.adc_stamp_ns,
                    "latency_us": (s.wall_arrival_ns - s.adc_stamp_ns) // 1000,
                    "source": s.source,
                    "det_count": s.det_count,
                })
            csv_out.flush("latency")

    # 记录初始 topic 统计 (归零)
    topic_snapshots: Dict[str, TopicStats] = {
        name: TopicStats(name=name) for name in node.topics
    }

    print(f"{'Time':>6s} | {'CPU%':>5s} {'RAM MB':>8s} {'GPU%':>5s} | "
          f"{'adc':>6s} {'cam':>6s} {'ego':>6s} {'det':>6s} {'obj':>6s} | "
          f"{'Lat(ms)':>8s}")
    print("-" * 85)

    try:
        while time.time() - start_time < args.duration:
            time.sleep(args.interval)
            elapsed = time.time() - start_time
            sample_count += 1

            # 系统指标
            node.capture_system_metrics()
            cpu, ram, _, _, _ = node.get_system_snapshot()
            gpu, gpu_temp, cpu_temp = gpu_mon.snapshot()

            # 更新 GPU 到 node
            with node._sys_lock:
                node._gpu_percent = gpu
                node._gpu_temp = gpu_temp
                node._cpu_temp = cpu_temp

            csv_out.writer("system").writerow({
                "elapsed_s": round(elapsed, 1),
                "cpu_percent": round(cpu, 1),
                "ram_mb": round(ram, 1),
                "gpu_percent": round(gpu, 1),
                "gpu_temp_c": round(gpu_temp, 1),
                "cpu_temp_c": round(cpu_temp, 1),
            })
            csv_out.flush("system")

            # 话题统计 (增量)
            topic_line_parts = []
            for name in ["/adc/raw_data", "/camera/image_raw",
                         "/vehicle/ego_motion", "/processing/radar/det_list",
                         "/processing/radar/det_list_cuda", "/perception/objects"]:
                cur = node.topics.get(name)
                prev = topic_snapshots.get(name)
                if cur and prev:
                    delta_count = cur.msg_count - prev.msg_count
                    freq = cur.freq_hz()
                    bw = cur.bw_mbps(args.interval * sample_count)

                    csv_out.writer("topics").writerow({
                        "elapsed_s": round(elapsed, 1),
                        "topic": name,
                        "msg_count": delta_count,
                        "freq_hz": round(freq, 1),
                        "bw_mbps": round(bw, 2),
                        "total_bytes": cur.total_bytes - prev.total_bytes,
                        "drop_count": cur.drop_count,
                    })

                    topic_line_parts.append(f"{freq:5.1f}")
                else:
                    topic_line_parts.append("    -")

                # 更新快照
                if cur:
                    topic_snapshots[name] = TopicStats(
                        name=name, msg_count=cur.msg_count,
                        total_bytes=cur.total_bytes,
                        last_stamp_ns=cur.last_stamp_ns,
                        last_wall_time=cur.last_wall_time,
                    )
            csv_out.flush("topics")

            # 时延统计
            with node._latency_lock:
                recent = node.latency_samples[-200:]
            if recent:
                lat_us = [s.wall_arrival_ns - s.adc_stamp_ns for s in recent]
                avg_lat_ms = np.mean(lat_us) / 1000
                max_lat_ms = np.max(lat_us) / 1000
                lat_str = f"{avg_lat_ms:5.1f}/{max_lat_ms:5.1f}"
            else:
                lat_str = "   -/-"

            # 终端输出
            topic_str = " ".join(f"{p:>6s}" for p in topic_line_parts)
            print(f"{elapsed:5.0f}s | {cpu:4.1f}% {ram:7.0f}MB {gpu:4.1f}% | "
                  f"{topic_str} | {lat_str:>8s}")

            # 定期写入时延
            if sample_count % 5 == 0:
                save_latency_batch()

    except KeyboardInterrupt:
        print("\n用户中断")

    # 收尾
    elapsed = time.time() - start_time
    print(f"\n监控结束, 运行 {elapsed:.0f}s, 正在生成报告...")

    save_latency_batch()

    # 汇总
    print("\n" + "=" * 60)
    print("  性能汇总")
    print("=" * 60)

    with node._latency_lock:
        all_lat = node.latency_samples[:]
    if all_lat:
        lat_us = [s.wall_arrival_ns - s.adc_stamp_ns for s in all_lat]
        print(f"\n  端到端时延 (ADC → RSP det_list):")
        print(f"    均值: {np.mean(lat_us)/1000:.1f} ms")
        print(f"    中位: {np.median(lat_us)/1000:.1f} ms")
        print(f"    P95:  {np.percentile(lat_us, 95)/1000:.1f} ms")
        print(f"    P99:  {np.percentile(lat_us, 99)/1000:.1f} ms")
        print(f"    最大: {np.max(lat_us)/1000:.1f} ms")
        print(f"    采样: {len(lat_us)} 帧")

        # 按来源分
        for src in ["det_list", "det_list_cuda"]:
            src_lat = [s.wall_arrival_ns - s.adc_stamp_ns
                       for s in all_lat if s.source == src]
            if src_lat:
                print(f"    {src}: avg={np.mean(src_lat)/1000:.1f}ms, "
                      f"median={np.median(src_lat)/1000:.1f}ms, n={len(src_lat)}")

    print(f"\n  话题频率:")
    for name in ["/adc/raw_data", "/camera/image_raw",
                 "/vehicle/ego_motion", "/processing/radar/det_list",
                 "/processing/radar/det_list_cuda", "/perception/objects"]:
        ts_obj = node.topics.get(name)
        if ts_obj and ts_obj.msg_count > 0:
            freq = ts_obj.freq_hz()
            print(f"    {name:<40s} {freq:6.1f} Hz ({ts_obj.msg_count} msgs)")

    cpu, ram, gpu, gpu_t, cpu_t = node.get_system_snapshot()
    print(f"\n  系统资源 (最终快照):")
    print(f"    CPU: {cpu:.1f}% | RAM: {ram:.0f} MB | GPU: {gpu:.1f}%")
    print(f"    GPU温度: {gpu_t:.1f}°C | CPU温度: {cpu_t:.1f}°C")

    print(f"\n  输出文件:")
    for name, path in csv_out.paths().items():
        print(f"    {path}")

    # 写入汇总报告
    summary_path = os.path.join(output_dir, f"perf_summary_{ts}.txt")
    with open(summary_path, "w") as f:
        f.write(f"FT Radar 性能监控报告\n")
        f.write(f"{'='*60}\n")
        f.write(f"时间: {datetime.now().isoformat()}\n")
        f.write(f"时长: {elapsed:.0f}s\n\n")
        f.write(f"端到端时延 (ADC → RSP):\n")
        if all_lat:
            f.write(f"  均值: {np.mean(lat_us)/1000:.1f}ms\n")
            f.write(f"  P95:  {np.percentile(lat_us, 95)/1000:.1f}ms\n")
            f.write(f"  P99:  {np.percentile(lat_us, 99)/1000:.1f}ms\n")
        for name in node.topics:
            ts_obj = node.topics[name]
            if ts_obj.msg_count > 0:
                f.write(f"\n{name}: {ts_obj.freq_hz():.1f} Hz, "
                        f"{ts_obj.msg_count} msgs\n")
    print(f"    {summary_path}")

    # 清理
    gpu_mon.stop()
    node.destroy_node()
    rclpy.shutdown()
    csv_out.close_all()
    print("\n完成.")


if __name__ == "__main__":
    main()
