#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 车辆数据接收节点 (Vehicle Data Rx)
================================================================================
模拟通过 CAN/ETH 接口接收车辆动态数据，发布为自定义 EgoMotion 消息。

规格:
  - 帧率: 50 Hz
  - 数据字段: 与 FT_radar_dataset_requirement 第 3 节完全对齐 (7 字段)
  - 默认值机制: 超过 3 个周期未收到有效数据，自动切换为默认值
  - 时间戳: 全局统一，微秒精度

话题:
  发布: /vehicle/ego_motion    ft_radar_msgs/EgoMotion

连接关系:
  → R SP MIL Python (sub)
  → R SP Cuda (sub)
  → Logging (sub)

作者: zhengyuan.liu
日期: 2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 采集参数 ----------
VEHICLE_FPS         = 50        # 车辆数据更新率 (Hz)
TIMEOUT_CYCLES      = 1         # 超时周期数 (1周期 = 20ms @ 50Hz)

# ---------- 默认值（车辆总线未连接时使用的预设值） ----------
DEFAULT_VX           = 0.0      # 车速 (m/s)
DEFAULT_YAW_RATE     = 0.0      # 偏航角速度 (rad/s)
DEFAULT_STEERING     = 0.0      # 转向角度 (rad)
DEFAULT_AX           = 0.0      # 纵向加速度 (m/s²)
DEFAULT_AY           = 0.0      # 横向加速度 (m/s²)
DEFAULT_GEAR         = 1        # 挡位: D 挡

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'base_link'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import rclpy
from rclpy.node import Node
import threading

from ft_radar_msgs.msg import EgoMotion


# ============================================================================
# ROS2 节点
# ============================================================================

class VehicleDataRxNode(Node):
    """
    车辆数据接收节点

    发布话题:
      /vehicle/ego_motion    ft_radar_msgs/EgoMotion

    功能说明:
      - 模拟 CAN/ETH 总线接收车辆动态数据
      - 7 字段与 FT_radar_dataset_requirement Egomotion CSV 完全对齐
      - 超时检测: 超过 timeout_cycles 个周期未收到数据 → 切换为默认值
      - is_default 标志供下游节点识别数据有效性
      - 注入全局统一时间戳（微秒精度）
    """

    def __init__(self):
        super().__init__('vehicle_data_rx')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('fps', VEHICLE_FPS)
        self.declare_parameter('timeout_cycles', TIMEOUT_CYCLES)
        self.declare_parameter('defaults.vx', DEFAULT_VX)
        self.declare_parameter('defaults.yaw_rate', DEFAULT_YAW_RATE)
        self.declare_parameter('defaults.steering_angle', DEFAULT_STEERING)
        self.declare_parameter('defaults.ax', DEFAULT_AX)
        self.declare_parameter('defaults.ay', DEFAULT_AY)
        self.declare_parameter('defaults.gear', DEFAULT_GEAR)
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.fps            = float(self.get_parameter('fps').value)
        self.timeout_cycles = int(self.get_parameter('timeout_cycles').value)
        self.fixed_frame    = self.get_parameter('fixed_frame').value

        # ---------- 默认值字典 ----------
        self._defaults = {
            'vx':             float(self.get_parameter('defaults.vx').value),
            'yaw_rate':       float(self.get_parameter('defaults.yaw_rate').value),
            'steering_angle': float(self.get_parameter('defaults.steering_angle').value),
            'ax':             float(self.get_parameter('defaults.ax').value),
            'ay':             float(self.get_parameter('defaults.ay').value),
            'gear':           int(self.get_parameter('defaults.gear').value),
        }

        # ---------- 超时检测状态 (接入 CAN/ETH 总线后启用) ----------
        self._timeout_ns = int(self.timeout_cycles * (1.0 / self.fps) * 1e9)

        # ---------- 发布者 ----------
        self.pub_ego = self.create_publisher(EgoMotion, '/vehicle/ego_motion', 10)

        # ---------- CAN buffer (线程安全, 读取线程更新, 定时器发布) ----------
        self._buffer_lock = threading.Lock()
        self._latest_ego = {
            'vx': self._defaults['vx'],
            'yaw_rate': self._defaults['yaw_rate'],
            'steering_angle': self._defaults['steering_angle'],
            'ax': self._defaults['ax'],
            'ay': self._defaults['ay'],
            'gear': self._defaults['gear'],
        }
        self._buffer_valid = False
        self._last_can_update_ns = 0  # 上次 CAN 更新时间 (用于超时检测)

        # ---------- 定时器 (按 fps 频率发布, 不控制硬件读取) ----------
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self._on_timer)
        self.frame_count = 0

        # ---------- CAN 读取线程 (持续轮询, 更新 buffer) ----------
        self._can_stop_event = threading.Event()
        self._can_thread = threading.Thread(target=self._can_read_loop, daemon=True)
        self._can_thread.start()

        self.get_logger().info(
            f'Vehicle Data Rx 启动: 发布 {self.fps:.0f} Hz, '
            f'超时检测: {self.timeout_cycles} 周期 '
            f'({self._timeout_ns / 1e9:.1f}s), '
            f'CAN read-thread → buffer → timer publish')

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        """
        定时器回调: 从 buffer 取最新 CAN 数据并发布 (按 fps 频率).

        CAN 读取线程持续更新 buffer, 定时器定时发布快照.
        """
        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()

        msg = EgoMotion()
        msg.header.stamp = stamp
        msg.header.frame_id = self.fixed_frame

        with self._buffer_lock:
            if self._buffer_valid:
                # 超时检测: 超过 timeout_cycles 周期未收到 CAN 数据 → 切换默认值
                now_ns = self.get_clock().now().nanoseconds
                elapsed = now_ns - self._last_can_update_ns
                if elapsed > self._timeout_ns:
                    self.get_logger().warn(
                        f'CAN 数据超时 ({elapsed / 1e9:.1f}s), 切换为默认值')
                    self._buffer_valid = False
                    msg.vx = self._defaults['vx']
                    msg.yaw_rate = self._defaults['yaw_rate']
                    msg.steering_angle = self._defaults['steering_angle']
                    msg.ax = self._defaults['ax']
                    msg.ay = self._defaults['ay']
                    msg.gear = self._defaults['gear']
                    msg.is_default = True
                else:
                    msg.vx = self._latest_ego['vx']
                    msg.yaw_rate = self._latest_ego['yaw_rate']
                    msg.steering_angle = self._latest_ego['steering_angle']
                    msg.ax = self._latest_ego['ax']
                    msg.ay = self._latest_ego['ay']
                    msg.gear = self._latest_ego['gear']
                    msg.is_default = False
            else:
                # CAN 未接入 → 发布默认值
                msg.vx = self._defaults['vx']
                msg.yaw_rate = self._defaults['yaw_rate']
                msg.steering_angle = self._defaults['steering_angle']
                msg.ax = self._defaults['ax']
                msg.ay = self._defaults['ay']
                msg.gear = self._defaults['gear']
                msg.is_default = True

        self.pub_ego.publish(msg)

        if self.frame_count == 1:
            self.get_logger().info(
                'Vehicle Data Rx: CAN/ETH 总线未接入, 发布默认值 (is_default=True)')

    # ------------------------------------------------------------------
    # CAN 读取线程 (持续轮询, 更新 buffer)
    # ------------------------------------------------------------------

    def _can_read_loop(self):
        """持续轮询 CAN 总线, 将最新数据更新到线程安全 buffer.

        TODO: 接入真实 CAN/ETH 总线后替换为实际读取逻辑.
        当前: CAN 未接入, 短暂休眠避免忙等.
        """
        while rclpy.ok() and not self._can_stop_event.is_set():
            # TODO: 接入真实 CAN 后替换为:
            #   can_data = can_socket.recv()
            #   with self._buffer_lock:
            #       self._latest_ego = parse_can(can_data)
            #       self._buffer_valid = True
            #       self._last_can_update_ns = self.get_clock().now().nanoseconds
            self._can_stop_event.wait(timeout=0.001)  # 1ms

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._can_stop_event.set()
        if hasattr(self, '_can_thread') and self._can_thread.is_alive():
            self._can_thread.join(timeout=2.0)
        self.get_logger().info(f'Vehicle Data Rx 已停止（共处理 {self.frame_count} 帧）')
        super().destroy_node()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = VehicleDataRxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Vehicle Data Rx 收到中断信号，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
