// ============================================================================
// vehicle_data_rx.cpp — 车辆数据采集节点 (50 Hz)
// ============================================================================
// TODO: 接入真实 CAN/ETH 总线解析，参见 docs/详细化开发方案.md
//
// CAN 接入方案:
//   1. 将 CAN 适配器 (如 PCAN-USB) 连接到 Jetson USB 口
//   2. 在 CMakeLists.txt 添加: target_compile_definitions(... PRIVATE USE_REAL_CAN)
//   3. 重新编译后, 节点自动切换到 SocketCAN 读取模式
//
// 当前状态: 发布默认值 + is_default=True, 下游节点据此判断数据有效性。
//
// 用法: vehicle_data_rx_cpp, 由 launch 文件自动启动.
// ============================================================================

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
    declare_parameter("defaults.vx", 0.0);
    declare_parameter("defaults.yaw_rate", 0.0);
    declare_parameter("defaults.steering_angle", 0.0);
    declare_parameter("defaults.ax", 0.0);
    declare_parameter("defaults.ay", 0.0);
    declare_parameter("defaults.gear", 1);
    declare_parameter("can_interface", "can0");

    fps_            = get_parameter("fps").as_double();
    timeout_cycles_ = get_parameter("timeout_cycles").as_int();
    can_iface_      = get_parameter("can_interface").as_string();

    dvx_   = get_parameter("defaults.vx").as_double();
    dyaw_  = get_parameter("defaults.yaw_rate").as_double();
    dsa_   = get_parameter("defaults.steering_angle").as_double();
    dax_   = get_parameter("defaults.ax").as_double();
    day_   = get_parameter("defaults.ay").as_double();
    dgear_ = get_parameter("defaults.gear").as_int();

    timeout_ns_ = static_cast<int64_t>(
        timeout_cycles_ * (1.0 / fps_) * 1'000'000'000.0);

    RCLCPP_INFO(get_logger(),
      "Vehicle Rx: %.0f Hz, CAN=%s (总线未接入, 发布默认值 is_default=True)",
      fps_, can_iface_.c_str());
    init_timer(fps_);
  }

  std::string frame_id() const { return "base_link"; }

  bool fill_message(EgoMotion &msg)
  {
    // TODO: 接入真实 CAN/ETH 总线，参见 docs/详细化开发方案.md
    // 当前状态: 发布默认值，is_default=True
#ifdef USE_REAL_CAN
    // 预留: SocketCAN 读取 (需配合车辆 DBC 定义)
    // if (can_fd_ >= 0) {
    //   struct can_frame frame;
    //   ssize_t n = read(can_fd_, &frame, sizeof(frame));
    //   if (n == sizeof(frame)) {
    //     parse_can_frame(frame, msg);
    //     msg.is_default = false;
    //     return true;
    //   }
    // }
#endif
    // 总线未接入 → 发布默认安全值
    msg.vx              = dvx_;
    msg.yaw_rate        = dyaw_;
    msg.steering_angle  = dsa_;
    msg.ax              = dax_;
    msg.ay              = day_;
    msg.gear            = dgear_;
    msg.is_default      = true;
    return true;
  }

private:
  int     timeout_cycles_ = 3;
  int64_t timeout_ns_     = 0;
  double  dvx_   = 0;
  double  dyaw_  = 0;
  double  dsa_   = 0;
  double  dax_   = 0;
  double  day_   = 0;
  int     dgear_ = 1;
  std::string can_iface_ = "can0";  // CAN 接口名 (USE_REAL_CAN 时生效)
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleDataRxNode>());
  rclcpp::shutdown();
  return 0;
}
