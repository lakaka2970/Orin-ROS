#!/usr/bin/env python3
"""
RSP 输出监控脚本 — 非侵入式记录 rsp_mil_python / rsp_cuda 的检测输出。

独立运行，不修改任何现有代码。自动订阅 RSP 的 DetList 话题并记录帧级统计。

用法:
    python3 scripts/monitor_rsp.py                          # 默认输出到 output/rsp_monitor/
    python3 scripts/monitor_rsp.py --output /path/to/out    # 自定义输出目录
    python3 scripts/monitor_rsp.py --detailed               # 额外记录逐点详情 CSV
    python3 scripts/monitor_rsp.py --no-summary             # 仅记录逐点详情

订阅:
    /processing/radar/det_list       (rsp_mil_python 或 rsp_cuda 单路模式)
    /processing/radar/det_list_cuda  (rsp_cuda 双路对比模式)

输出文件 (按启动时间命名):
    rsp_summary_20260702_143000.csv   # 帧级统计: 时间戳/来源/帧号/检测数/SNR/距离/能量
    rsp_detailed_20260702_143000.csv  # 逐点详情 (--detailed 时)

作者: zhengyuan.liu
日期: 2026.7.2
"""

import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from ft_radar_msgs.msg import DetList


# =============================================================================
# 统计计算
# =============================================================================

def compute_frame_stats(msg: DetList, source: str) -> dict:
    """从 DetList 消息提取帧级统计"""
    n = len(msg.points)

    if n == 0:
        return {
            "timestamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
            "source": source,
            "frame_id": msg.frame_id,
            "det_count": 0,
            # ---- SNR ----
            "snr_avg_db": 0.0,
            "snr_max_db": 0.0,
            "snr_min_db": 0.0,
            # ---- 距离 ----
            "range_avg_m": 0.0,
            "range_max_m": 0.0,
            "range_min_m": 0.0,
            # ---- 速度 ----
            "speed_avg_mps": 0.0,
            "speed_max_mps": 0.0,
            "speed_min_mps": 0.0,
            "rad_vel_avg_mps": 0.0,
            "rad_vel_max_mps": 0.0,
            "rad_vel_min_mps": 0.0,
            # ---- 角度 ----
            "azimuth_avg_deg": 0.0,
            "ele_avg_deg": 0.0,
            "ele_max_deg": 0.0,
            "ele_min_deg": 0.0,
            # ---- 功率/RCS ----
            "power_avg_db": 0.0,
            "rcs_avg_dbsm": 0.0,
            # ---- 位置 ----
            "x_avg_m": 0.0,
            "y_avg_m": 0.0,
            "det_conf_avg": 0.0,
            "moving_count": 0,
            "stationary_count": 0,
        }

    snrs = np.array([p.snr_db for p in msg.points], dtype=np.float32)
    ranges = np.array([p.range for p in msg.points], dtype=np.float32)
    powers = np.array([p.power_db for p in msg.points], dtype=np.float32)
    rcss = np.array([p.rcs_db for p in msg.points], dtype=np.float32)
    azimuths = np.array([p.azimuth_ang for p in msg.points], dtype=np.float32)
    elevations = np.array([p.ele_ang for p in msg.points], dtype=np.float32)
    speeds = np.array([p.speed for p in msg.points], dtype=np.float32)
    rad_vels = np.array([p.rad_vel_abs for p in msg.points], dtype=np.float32)
    xs = np.array([p.x for p in msg.points], dtype=np.float32)
    ys = np.array([p.y for p in msg.points], dtype=np.float32)
    confs = np.array([p.det_conf for p in msg.points], dtype=np.float32)
    motions = np.array([p.det_motion_pat for p in msg.points], dtype=np.uint8)

    moving = int(np.sum(motions == 1))
    stationary = int(np.sum(motions == 0))

    return {
        "timestamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
        "source": source,
        "frame_id": msg.frame_id,
        "det_count": n,
        # ---- SNR ----
        "snr_avg_db": round(float(np.mean(snrs)), 2),
        "snr_max_db": round(float(np.max(snrs)), 2),
        "snr_min_db": round(float(np.min(snrs)), 2),
        # ---- 距离 ----
        "range_avg_m": round(float(np.mean(ranges)), 3),
        "range_max_m": round(float(np.max(ranges)), 3),
        "range_min_m": round(float(np.min(ranges)), 3),
        # ---- 速度 ----
        "speed_avg_mps": round(float(np.mean(speeds)), 3),
        "speed_max_mps": round(float(np.max(speeds)), 3),
        "speed_min_mps": round(float(np.min(speeds)), 3),
        "rad_vel_avg_mps": round(float(np.mean(rad_vels)), 3),
        "rad_vel_max_mps": round(float(np.max(rad_vels)), 3),
        "rad_vel_min_mps": round(float(np.min(rad_vels)), 3),
        # ---- 角度 ----
        "azimuth_avg_deg": round(float(np.degrees(np.mean(azimuths))), 2),
        "ele_avg_deg": round(float(np.degrees(np.mean(elevations))), 2),
        "ele_max_deg": round(float(np.degrees(np.max(elevations))), 2),
        "ele_min_deg": round(float(np.degrees(np.min(elevations))), 2),
        # ---- 功率/RCS ----
        "power_avg_db": round(float(np.mean(powers)), 2),
        "rcs_avg_dbsm": round(float(np.mean(rcss)), 2),
        # ---- 位置 ----
        "x_avg_m": round(float(np.mean(xs)), 3),
        "y_avg_m": round(float(np.mean(ys)), 3),
        "det_conf_avg": round(float(np.mean(confs)), 1),
        "moving_count": moving,
        "stationary_count": stationary,
    }


def point_to_dict(msg: DetList, pt_idx: int, source: str) -> dict:
    """提取单个检测点的全部字段"""
    p = msg.points[pt_idx]
    return {
        "timestamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
        "source": source,
        "frame_id": msg.frame_id,
        "point_idx": pt_idx,
        "x_m": p.x,
        "y_m": p.y,
        "z_m": p.z,
        "range_m": p.range,
        "speed_mps": p.speed,
        "rad_vel_abs_mps": p.rad_vel_abs,
        "azimuth_deg": round(float(np.degrees(p.azimuth_ang)), 3),
        "ele_deg": round(float(np.degrees(p.ele_ang)), 3),
        "snr_db": p.snr_db,
        "rcs_dbsm": p.rcs_db,
        "power_db": p.power_db,
        "det_conf": p.det_conf,
        "det_motion_pat": p.det_motion_pat,
        "det_ambig_state": p.det_ambig_state,
        "range_idx": p.range_idx,
        "doppler_idx": p.doppler_idx,
        "azimuth_idx": p.azimuth_idx,
        "peak_val": p.peak_val,
        "doa_method": p.doa_method,
        "obj_quality": p.obj_quality,
    }


# =============================================================================
# ROS2 监控节点
# =============================================================================

class RspMonitor(Node):
    """订阅 RSP 检测输出并记录到 CSV"""

    def __init__(self, output_dir: str, do_summary: bool, do_detailed: bool):
        super().__init__("rsp_monitor")

        self.output_dir = output_dir
        self.do_summary = do_summary
        self.do_detailed = do_detailed
        os.makedirs(output_dir, exist_ok=True)

        # 生成带时间戳的文件名
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")

        # 帧级统计 CSV
        self.summary_file = None
        self.summary_writer = None
        if do_summary:
            self.summary_path = os.path.join(output_dir, f"rsp_summary_{ts}.csv")
            self.summary_file = open(self.summary_path, "w", newline="")
            self.summary_writer = csv.DictWriter(
                self.summary_file,
                fieldnames=SUMMARY_FIELDS,
            )
            self.summary_writer.writeheader()
            self.summary_file.flush()
            self.get_logger().info(f"帧级统计: {self.summary_path}")

        # 逐点详情 CSV
        self.detailed_file = None
        self.detailed_writer = None
        if do_detailed:
            self.detailed_path = os.path.join(output_dir, f"rsp_detailed_{ts}.csv")
            self.detailed_file = open(self.detailed_path, "w", newline="")
            self.detailed_writer = csv.DictWriter(
                self.detailed_file,
                fieldnames=DETAILED_FIELDS,
            )
            self.detailed_writer.writeheader()
            self.detailed_file.flush()
            self.get_logger().info(f"逐点详情: {self.detailed_path}")

        # 帧计数
        self.frame_count_python = 0
        self.frame_count_cuda = 0

        # QoS: BEST_EFFORT 匹配 RSP 发布端
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )

        # 订阅两个话题
        self.sub_python = self.create_subscription(
            DetList,
            "/processing/radar/det_list",
            lambda msg: self._on_det_list(msg, "det_list"),
            qos,
        )
        self.sub_cuda = self.create_subscription(
            DetList,
            "/processing/radar/det_list_cuda",
            lambda msg: self._on_det_list(msg, "det_list_cuda"),
            qos,
        )

        self.get_logger().info("RSP 监控已启动 — 等待检测数据...")
        self.get_logger().info(f"  输出目录: {output_dir}")
        topics = []
        if do_summary:
            topics.append("帧级统计")
        if do_detailed:
            topics.append("逐点详情")
        self.get_logger().info(f"  记录内容: {', '.join(topics)}")

    def _on_det_list(self, msg: DetList, source: str):
        """收到 DetList 消息时的回调"""
        now = self.get_clock().now()

        # 更新计数
        if source == "det_list":
            self.frame_count_python += 1
        else:
            self.frame_count_cuda += 1

        # 帧级统计
        if self.do_summary and self.summary_writer is not None:
            stats = compute_frame_stats(msg, source)
            self.summary_writer.writerow(stats)
            # 每 10 帧刷新一次
            total = self.frame_count_python + self.frame_count_cuda
            if total % 10 == 0:
                self.summary_file.flush()
                tc = f"det_list={self.frame_count_python}, det_list_cuda={self.frame_count_cuda}"
                self.get_logger().info(
                    f"[{total} 帧] {tc} | "
                    f"最新: frame#{msg.frame_id} {msg.det_obj_num}点 SNR_avg={stats['snr_avg_db']}dB"
                )

        # 逐点详情
        if self.do_detailed and self.detailed_writer is not None:
            for i in range(len(msg.points)):
                row = point_to_dict(msg, i, source)
                self.detailed_writer.writerow(row)
            # 每个帧写完刷新
            self.detailed_file.flush()

    def close(self):
        """关闭文件"""
        if self.summary_file:
            self.summary_file.close()
        if self.detailed_file:
            self.detailed_file.close()

    def get_stats(self) -> dict:
        """返回当前统计"""
        return {
            "frames_det_list": self.frame_count_python,
            "frames_det_list_cuda": self.frame_count_cuda,
            "output_dir": self.output_dir,
        }


# =============================================================================
# CSV 字段定义
# =============================================================================

SUMMARY_FIELDS = [
    "timestamp_ns", "source", "frame_id", "det_count",
    "snr_avg_db", "snr_max_db", "snr_min_db",
    "range_avg_m", "range_max_m", "range_min_m",
    "speed_avg_mps", "speed_max_mps", "speed_min_mps",
    "rad_vel_avg_mps", "rad_vel_max_mps", "rad_vel_min_mps",
    "azimuth_avg_deg", "ele_avg_deg", "ele_max_deg", "ele_min_deg",
    "power_avg_db", "rcs_avg_dbsm",
    "x_avg_m", "y_avg_m", "det_conf_avg",
    "moving_count", "stationary_count",
]

DETAILED_FIELDS = [
    "timestamp_ns", "source", "frame_id", "point_idx",
    "x_m", "y_m", "z_m", "range_m", "speed_mps", "rad_vel_abs_mps",
    "azimuth_deg", "ele_deg",
    "snr_db", "rcs_dbsm", "power_db",
    "det_conf", "det_motion_pat", "det_ambig_state",
    "range_idx", "doppler_idx", "azimuth_idx",
    "peak_val", "doa_method", "obj_quality",
]


# =============================================================================
# 入口
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="RSP 输出监控 — 非侵入式记录 rsp_mil_python / rsp_cuda 检测输出"
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default=None,
        help="输出目录 (默认: output/rsp_monitor/)",
    )
    parser.add_argument(
        "--detailed", "-d",
        action="store_true",
        help="同时记录逐点详情 CSV (数据量较大, 每帧数百行)",
    )
    parser.add_argument(
        "--no-summary",
        action="store_true",
        help="不记录帧级统计, 仅记录逐点详情 (需配合 --detailed)",
    )
    args = parser.parse_args()

    # 解析输出目录
    if args.output:
        output_dir = args.output
    else:
        # 默认: 项目根目录下的 output/rsp_monitor/
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        output_dir = os.path.join(project_root, "output", "rsp_monitor")

    do_summary = not args.no_summary
    do_detailed = args.detailed

    if not do_summary and not do_detailed:
        print("错误: --no-summary 需要配合 --detailed 使用")
        print("提示: 默认记录帧级统计, 加 --detailed 额外记录逐点详情")
        sys.exit(1)

    print(f"RSP 监控启动")
    print(f"  输出目录: {output_dir}")
    print(f"  帧级统计: {'是' if do_summary else '否'}")
    print(f"  逐点详情: {'是' if do_detailed else '否'}")
    print(f"  按 Ctrl+C 停止")
    print()

    rclpy.init(args=sys.argv)

    node = RspMonitor(output_dir, do_summary, do_detailed)

    # 优雅退出
    def shutdown(signum=None, frame=None):
        stats = node.get_stats()
        print()
        print("=" * 50)
        print("  RSP 监控停止")
        print(f"  接收帧数: det_list={stats['frames_det_list']}, "
              f"det_list_cuda={stats['frames_det_list_cuda']}")
        print(f"  输出目录: {stats['output_dir']}")
        if do_summary:
            print(f"  帧级统计: {node.summary_path}")
        if do_detailed:
            print(f"  逐点详情: {node.detailed_path}")
        print("=" * 50)
        node.close()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == "__main__":
    main()
