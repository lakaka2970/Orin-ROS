#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 车辆数据接收节点 (Vehicle Data Rx)
================================================================================
模拟通过 CAN/ETH 接口接收车辆动态数据（车速、航向角等），发布为 TwistStamped。

发布话题：
  /ft/vehicle_data   TwistStamped   车辆速度与航向

连接关系：
  → R SP MIL Python (sub)
  → R SP Cuda (sub)
  → Logging (sub)

作者：zhengyuan.liu
日期：2026.6.8
================================================================================
"""

# ============================================================================
# ★ 用户配置区 —— 所有常用参数集中在此，修改后重启节点即可生效
# ============================================================================

# ---------- 车辆数据参数 ----------
VEHICLE_FPS     = 20.0       # 车辆数据更新率 (Hz)
SIM_SPEED_MEAN  = 15.0       # 模拟平均车速 (m/s)
SIM_SPEED_STD   = 2.0        # 模拟车速噪声标准差 (m/s)
SIM_YAW_RATE    = 0.05       # 模拟航向角变化率 (rad/s)

# ---------- RViz 坐标系 ----------
FIXED_FRAME = 'base_link'

# ============================================================================
# 以下为程序实现，一般无需修改
# ============================================================================

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from std_msgs.msg import Header


# ============================================================================
# ROS2 节点
# ============================================================================

class VehicleDataRxNode(Node):
    """
    车辆数据接收节点 —— 模拟 CAN/ETH 采集，发布 TwistStamped

    发布话题：
      /ft/vehicle_data   TwistStamped   车辆速度与航向角速度
    """

    def __init__(self):
        super().__init__('vehicle_data_rx')

        # ---------- ROS2 参数声明 ----------
        self.declare_parameter('vehicle_fps',    VEHICLE_FPS)
        self.declare_parameter('sim_speed_mean', SIM_SPEED_MEAN)
        self.declare_parameter('sim_speed_std',  SIM_SPEED_STD)
        self.declare_parameter('sim_yaw_rate',   SIM_YAW_RATE)
        self.declare_parameter('fixed_frame',    FIXED_FRAME)

        self.vehicle_fps    = float(self.get_parameter('vehicle_fps').value)
        self.sim_speed_mean = float(self.get_parameter('sim_speed_mean').value)
        self.sim_speed_std  = float(self.get_parameter('sim_speed_std').value)
        self.sim_yaw_rate   = float(self.get_parameter('sim_yaw_rate').value)
        self.fixed_frame    = self.get_parameter('fixed_frame').value

        # ---------- 发布者 ----------
        self.pub_vehicle = self.create_publisher(TwistStamped, '/ft/vehicle_data', 10)

        # ---------- 定时器 ----------
        period = 1.0 / self.vehicle_fps
        self.timer = self.create_timer(period, self._on_timer)
        self.frame_count = 0
        self.heading = 0.0    # 累积航向角

        self.get_logger().info(
            f'Vehicle Data Rx 启动: {self.vehicle_fps:.0f} Hz, '
            f'模拟车速 ~{self.sim_speed_mean} m/s')

    # ------------------------------------------------------------------
    # 定时器回调
    # ------------------------------------------------------------------

    def _on_timer(self):
        """生成模拟车辆动态数据并发布"""
        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()

        # 模拟车速（带噪声）
        speed = np.random.normal(self.sim_speed_mean, self.sim_speed_std)
        speed = max(0.0, speed)

        # 累积航向角（模拟车辆缓慢转弯）
        self.heading += self.sim_yaw_rate / self.vehicle_fps

        # 分解为 x (前向) 和 y (横向) 分量
        vx = speed * math.cos(self.heading)
        vy = speed * math.sin(self.heading)

        # 构造 TwistStamped 消息
        msg = TwistStamped()
        msg.header = Header()
        msg.header.frame_id = self.fixed_frame
        msg.header.stamp = stamp
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.sim_yaw_rate + np.random.normal(0, 0.01)

        self.pub_vehicle.publish(msg)
        self.get_logger().debug(
            f'Vehicle Data Rx 帧 #{self.frame_count}: '
            f'speed={speed:.2f} m/s, heading={math.degrees(self.heading):.1f}°')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Vehicle Data Rx 已停止')
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
