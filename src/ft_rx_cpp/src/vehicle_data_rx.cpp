// ============================================================================
// vehicle_data_rx.cpp — 车辆数据采集节点 (50 Hz, Gaussian 模拟)
// ============================================================================

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

    fps_            = get_parameter("fps").as_double();
    timeout_cycles_ = get_parameter("timeout_cycles").as_int();
    sim_speed_mean_ = get_parameter("sim_speed_mean").as_double();
    sim_speed_std_  = get_parameter("sim_speed_std").as_double();
    sim_yaw_rate_   = get_parameter("sim_yaw_rate").as_double();
    dvx_ = get_parameter("defaults.vx").as_double();
    dyaw_ = get_parameter("defaults.yaw_rate").as_double();
    dsa_ = get_parameter("defaults.steering_angle").as_double();
    dax_ = get_parameter("defaults.ax").as_double();
    day_ = get_parameter("defaults.ay").as_double();
    dgear_ = get_parameter("defaults.gear").as_int();

    timeout_ns_ = static_cast<int64_t>(
        timeout_cycles_ * (1.0 / fps_) * 1'000'000'000.0);
    last_valid_ = this->now();

    RCLCPP_INFO(get_logger(),
      "Vehicle Rx: %.0f Hz, 超时 %d cycles (%.1fs)",
      fps_, timeout_cycles_, timeout_ns_ / 1e9);
    init_timer(fps_);
  }

  std::string frame_id() const { return "base_link"; }

  bool fill_message(EgoMotion &msg)
  {
    int64_t e = (this->now() - last_valid_).nanoseconds();
    if (e > timeout_ns_) {
      msg.vx = dvx_; msg.yaw_rate = dyaw_; msg.steering_angle = dsa_;
      msg.ax = dax_; msg.ay = day_; msg.gear = dgear_;
      msg.is_default = true;
    } else {
      double sp = std::max(0.0,
        std::normal_distribution<double>(sim_speed_mean_, sim_speed_std_)(rng_));
      heading_ += sim_yaw_rate_ / fps_;
      msg.vx = sp;
      msg.yaw_rate = sim_yaw_rate_ + std::normal_distribution<double>(0, 0.01)(rng_);
      msg.steering_angle = std::atan2(sim_yaw_rate_, sp + 1e-6);
      msg.ax = std::normal_distribution<double>(0, 0.5)(rng_);
      msg.ay = sp * sim_yaw_rate_;
      msg.gear = 1;
      msg.is_default = false;
      last_valid_ = this->now();
    }
    return true;
  }

private:
  int     timeout_cycles_ = 3;
  int64_t timeout_ns_     = 0;
  double  sim_speed_mean_ = 15.0, sim_speed_std_ = 2.0, sim_yaw_rate_ = 0.05;
  double  dvx_ = 0, dyaw_ = 0, dsa_ = 0, dax_ = 0, day_ = 0;
  int     dgear_ = 1;
  double  heading_ = 0;
  rclcpp::Time last_valid_;
  std::mt19937 rng_{std::random_device{}()};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleDataRxNode>());
  rclcpp::shutdown();
}
