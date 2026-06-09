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
TIMEOUT_CYCLES      = 3         # 超时周期数

# ---------- 默认值（车辆总线数据丢失时使用的预设值） ----------
DEFAULT_VX           = 0.0      # 车速 (m/s)
DEFAULT_YAW_RATE     = 0.0      # 偏航角速度 (rad/s)
DEFAULT_STEERING     = 0.0      # 转向角度 (rad)
DEFAULT_AX           = 0.0      # 纵向加速度 (m/s²)
DEFAULT_AY           = 0.0      # 横向加速度 (m/s²)
DEFAULT_GEAR         = 1        # 挡位: D 挡

# ---------- 模拟参数 ----------
SIM_SPEED_MEAN  = 15.0          # 模拟平均车速 (m/s)
SIM_SPEED_STD   = 2.0           # 模拟车速噪声标准差 (m/s)
SIM_YAW_RATE    = 0.05          # 模拟航向角变化率 (rad/s)

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'base_link'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import math
import numpy as np

import rclpy
from rclpy.node import Node

from ft_radar_msgs.msg import EgoMotion
from ft_framework.common import monotonic_us_stamp


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
        self.declare_parameter('sim_speed_mean', SIM_SPEED_MEAN)
        self.declare_parameter('sim_speed_std', SIM_SPEED_STD)
        self.declare_parameter('sim_yaw_rate', SIM_YAW_RATE)
        self.declare_parameter('fixed_frame', FIXED_FRAME)

        self.fps            = float(self.get_parameter('fps').value)
        self.timeout_cycles = int(self.get_parameter('timeout_cycles').value)
        self.sim_speed_mean = float(self.get_parameter('sim_speed_mean').value)
        self.sim_speed_std  = float(self.get_parameter('sim_speed_std').value)
        self.sim_yaw_rate   = float(self.get_parameter('sim_yaw_rate').value)
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

        # ---------- 超时检测状态 ----------
        self._last_valid_time = self.get_clock().now()
        # timeout 以纳秒为单位，与 Duration.nanoseconds 类型一致
        self._timeout_ns = int(self.timeout_cycles * (1.0 / self.fps) * 1e9)

        # ---------- 模拟状态 ----------
        self._heading = 0.0

        # ---------- 发布者 ----------
        self.pub_ego = self.create_publisher(EgoMotion, '/vehicle/ego_motion', 10)

        # ---------- 定时器 ----------
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self._on_timer)
        self.frame_count = 0

        self.get_logger().info(
            f'Vehicle Data Rx 启动: {self.fps:.0f} Hz, '
            f'超时检测: {self.timeout_cycles} 周期 '
            f'({self._timeout_ns / 1e9:.1f}s)')

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        """
        模拟车辆数据采集，含默认值机制。

        数据流:
          1. 检测是否超时
          2. 超时 → 使用默认值
          3. 未超时 → 模拟总线数据
          4. 注入时间戳并发布
        """
        self.frame_count += 1
        sec, nsec = monotonic_us_stamp()

        msg = EgoMotion()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = self.fixed_frame

        # ---- 超时检测 ----
        current_time = self.get_clock().now()
        elapsed_ns = (current_time - self._last_valid_time).nanoseconds

        if elapsed_ns > self._timeout_ns:
            # 数据超时 → 使用默认值
            msg.vx = self._defaults['vx']
            msg.yaw_rate = self._defaults['yaw_rate']
            msg.steering_angle = self._defaults['steering_angle']
            msg.ax = self._defaults['ax']
            msg.ay = self._defaults['ay']
            msg.gear = self._defaults['gear']
            msg.is_default = True

            self.get_logger().warn(
                f'车辆数据超时 ({elapsed_ns / 1e9:.2f}s)，使用默认值')
        else:
            # 模拟 CAN 总线有效数据
            # 实际部署时替换为 CAN/ETH 总线读取
            speed = np.random.normal(self.sim_speed_mean, self.sim_speed_std)
            speed = max(0.0, speed)
            self._heading += self.sim_yaw_rate / self.fps

            msg.vx = speed
            msg.yaw_rate = self.sim_yaw_rate + np.random.normal(0, 0.01)
            msg.steering_angle = math.atan2(self.sim_yaw_rate, speed + 1e-6)
            msg.ax = np.random.normal(0, 0.5)                 # 模拟纵向加速度波动
            msg.ay = speed * self.sim_yaw_rate                 # 向心加速度
            msg.gear = 1                                       # D 挡
            msg.is_default = False

            self._last_valid_time = current_time

        self.pub_ego.publish(msg)
        self.get_logger().debug(
            f'Vehicle Data Rx 帧 #{self.frame_count}: '
            f'vx={msg.vx:.2f}, yaw_rate={msg.yaw_rate:.3f}, '
            f'default={msg.is_default}')

    # ------------------------------------------------------------------
    # 外部接口：供其他节点触发默认值测试
    # ------------------------------------------------------------------

    def simulate_timeout(self):
        """(测试用) 强制触发超时，模拟总线数据丢失"""
        self._last_valid_time = self.get_clock().now() - \
            rclpy.duration.Duration(seconds=2 * self._timeout_ns / 1e9)
        self.get_logger().warn('模拟车辆数据丢失...')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
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
