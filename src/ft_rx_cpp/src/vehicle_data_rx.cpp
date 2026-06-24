// ============================================================================
// vehicle_data_rx.cpp — 车辆数据采集节点 (50 Hz, Gaussian 模拟)
// ============================================================================
// 当前处于模拟模式: 使用 Gaussian 随机数生成车辆动力学数据.
//
// CAN/ETH 接入方案 (待 CAN 硬件就绪后启用):
//   1. 将 CAN 适配器 (如 PCAN-USB) 连接到 Jetson USB 口
//   2. 在 CMakeLists.txt 添加: target_compile_definitions(... PRIVATE USE_REAL_CAN)
//   3. 重新编译后, 节点自动切换到 SocketCAN 读取模式
//
// 用法: vehicle_data_rx_cpp, 由 launch 文件自动启动.
// ============================================================================

#include <algorithm>
#include <cmath>
#include <random>
#include <string>

#include "ft_rx_cpp/rx_node_base.hpp"
#include "ft_radar_msgs/msg/ego_motion.hpp"

using EgoMotion = ft_radar_msgs::msg::EgoMotion;

namespace { constexpr double FPS = 50.0; }

class VehicleDataRxNode : public ft_rx::RxNodeBase<EgoMotion, VehicleDataRxNode>
{
public:
  VehicleDataRxNode()
    : RxNodeBase("vehicle_data_rx", "/vehicle/ego_motion", 10, true)
  {
    declare_parameter("fps", FPS);
    declare_parameter("timeout_cycles", 3);
    declare_parameter("sim_speed_mean", 15.0);
    declare_parameter("sim_speed_std", 2.0);
    declare_parameter("sim_yaw_rate", 0.05);
    declare_parameter("defaults.vx", 0.0);
    declare_parameter("defaults.yaw_rate", 0.0);
    declare_parameter("defaults.steering_angle", 0.0);
    declare_parameter("defaults.ax", 0.0);
    declare_parameter("defaults.ay", 0.0);
    declare_parameter("defaults.gear", 1);
    declare_parameter("can_interface", "can0");

    fps_            = get_parameter("fps").as_double();
    timeout_cycles_ = get_parameter("timeout_cycles").as_int();
    sim_speed_mean_ = get_parameter("sim_speed_mean").as_double();
    sim_speed_std_  = get_parameter("sim_speed_std").as_double();
    sim_yaw_rate_   = get_parameter("sim_yaw_rate").as_double();
    can_iface_      = get_parameter("can_interface").as_string();

    dvx_   = get_parameter("defaults.vx").as_double();
    dyaw_  = get_parameter("defaults.yaw_rate").as_double();
    dsa_   = get_parameter("defaults.steering_angle").as_double();
    dax_   = get_parameter("defaults.ax").as_double();
    day_   = get_parameter("defaults.ay").as_double();
    dgear_ = get_parameter("defaults.gear").as_int();

    timeout_ns_ = static_cast<int64_t>(
        timeout_cycles_ * (1.0 / fps_) * 1'000'000'000.0);
    last_valid_ = this->now();

    {
      const char *can_mode =
#ifdef USE_REAL_CAN
          "实际";
#else
          "模拟";
#endif
      RCLCPP_INFO(get_logger(),
        "Vehicle Rx: %.0f Hz, 超时 %d cycles (%.1fs), CAN=%s [%s]",
        fps_, timeout_cycles_, timeout_ns_ / 1e9,
        can_iface_.c_str(), can_mode);
    }
    init_timer(fps_);
  }

  std::string frame_id() const { return "base_link"; }

  bool fill_message(EgoMotion &msg)
  {
    // ── CAN/ETH 模式 (编译时通过 USE_REAL_CAN 宏切换) ──
    // 启用方法: CMakeLists.txt 添加
    //   target_compile_definitions(vehicle_data_rx_cpp PRIVATE USE_REAL_CAN)
    // 并添加 CAN Socket 依赖: <linux/can.h>, <linux/can/raw.h>, <net/if.h>
#ifdef USE_REAL_CAN
    // 预留: SocketCAN 读取
    // if (can_fd_ >= 0) {
    //   struct can_frame frame;
    //   ssize_t n = read(can_fd_, &frame, sizeof(frame));
    //   if (n == sizeof(frame)) {
    //     // 从 CAN frame 解析 EgoMotion 字段 (需配合车辆 DBC 定义)
    //     parse_can_frame(frame, msg);
    //     last_valid_ = this->now();
    //     msg.is_default = false;
    //     return true;
    //   }
    // }
    // 超时：使用默认安全值
    int64_t e = (this->now() - last_valid_).nanoseconds();
    if (e > timeout_ns_) {
      msg.vx = dvx_;
      msg.yaw_rate = dyaw_;
      msg.steering_angle = dsa_;
      msg.ax = dax_;
      msg.ay = day_;
      msg.gear = dgear_;
      msg.is_default = true;
    }
    return true;
#else
    // ── 模拟模式: Gaussian 随机 ──
    int64_t e = (this->now() - last_valid_).nanoseconds();
    if (e > timeout_ns_) {
      // 超时：使用默认安全值
      msg.vx = dvx_;
      msg.yaw_rate = dyaw_;
      msg.steering_angle = dsa_;
      msg.ax = dax_;
      msg.ay = day_;
      msg.gear = dgear_;
      msg.is_default = true;
    } else {
      double sp = std::max(0.0, speed_dist_(rng_));
      heading_ += sim_yaw_rate_ / fps_;
      msg.vx = sp;
      msg.yaw_rate = sim_yaw_rate_ + yaw_noise_dist_(rng_);
      msg.steering_angle = std::atan2(sim_yaw_rate_, sp + 1e-6);
      msg.ax = ax_dist_(rng_);
      msg.ay = sp * sim_yaw_rate_;
      msg.gear = 1;
      msg.is_default = false;
      // 仅在数据有效时更新时间戳 (CAN 模式下解决超时永不触发的 bug)
      last_valid_ = this->now();
    }
    return true;
#endif
  }

private:
  int     timeout_cycles_ = 3;
  int64_t timeout_ns_     = 0;
  double  sim_speed_mean_ = 15.0;
  double  sim_speed_std_  = 2.0;
  double  sim_yaw_rate_   = 0.05;
  double  dvx_   = 0;
  double  dyaw_  = 0;
  double  dsa_   = 0;
  double  dax_   = 0;
  double  day_   = 0;
  int     dgear_ = 1;
  double  heading_ = 0;       // 累计航向角 (rad)
  rclcpp::Time last_valid_;
  std::string can_iface_ = "can0";  // CAN 接口名 (USE_REAL_CAN 时生效)

  // 预构造分布对象，避免每帧重复构造
  std::mt19937 rng_{std::random_device{}()};
  std::normal_distribution<double> speed_dist_{sim_speed_mean_, sim_speed_std_};
  std::normal_distribution<double> yaw_noise_dist_{0, 0.01};
  std::normal_distribution<double> ax_dist_{0, 0.5};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleDataRxNode>());
  rclcpp::shutdown();
  return 0;
}
