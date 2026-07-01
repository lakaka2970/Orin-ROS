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

#include <mutex>
#include <string>
#include <thread>

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
    declare_parameter("timeout_cycles", 1);
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
      "Vehicle Rx: publish %.0f Hz | CAN=%s (read-thread -> buffer -> timer publish)",
      fps_, can_iface_.c_str());
    init_timer(fps_);  // 定时器按 fps 频率发布, 不是硬件读取频率

    // 启动 CAN 读取线程 (持续轮询, 更新 buffer)
    can_read_thread_ = std::thread(&VehicleDataRxNode::can_read_loop, this);
  }

  ~VehicleDataRxNode() override
  {
    stop_read_ = true;
    if (can_read_thread_.joinable())
      can_read_thread_.join();
  }

  std::string frame_id() const { return "base_link"; }

  // ── 由定时器回调调用: 从 buffer 取最新 CAN 数据并发布 ──
  bool fill_message(EgoMotion &msg)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (buffer_valid_) {
      // 从 buffer 拷贝最新 CAN 数据
      msg = latest_ego_;
      // 超时检测: 超过 timeout_cycles 周期未收到 CAN 数据 → 切换默认值
      int64_t now_ns = this->now().nanoseconds();
      int64_t elapsed = now_ns - last_can_update_ns_;
      if (elapsed > timeout_ns_) {
        fill_defaults(msg);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "CAN 数据超时 (%.1fs), 切换为默认值", elapsed / 1e9);
      }
    } else {
      // CAN 未接入 → 发布默认安全值
      fill_defaults(msg);
    }
    return true;
  }

  // ── 默认值填充 ──
  void fill_defaults(EgoMotion &msg)
  {
    msg.vx              = dvx_;
    msg.yaw_rate        = dyaw_;
    msg.steering_angle  = dsa_;
    msg.ax              = dax_;
    msg.ay              = day_;
    msg.gear            = dgear_;
    msg.is_default      = true;
  }

  // ── CAN 读取线程: 持续轮询 CAN 总线, 更新 buffer ──
  void can_read_loop()
  {
    while (rclcpp::ok() && !stop_read_) {
#ifdef USE_REAL_CAN
      // TODO: 接入真实 CAN 总线后, 使用 poll() + read() 阻塞读取 CAN 帧
      // struct pollfd pfd = {can_fd_, POLLIN, 0};
      // int ret = poll(&pfd, 1, 1);  // 1ms timeout for responsive shutdown
      // if (ret > 0) {
      //   struct can_frame frame;
      //   ssize_t n = read(can_fd_, &frame, sizeof(frame));
      //   if (n == sizeof(frame)) {
      //     std::lock_guard<std::mutex> lock(buffer_mutex_);
      //     parse_can_frame(frame, latest_ego_);
      //     buffer_valid_ = true;
      //     last_can_update_ns_ = this->now().nanoseconds();
      //   }
      // }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
#else
      // CAN 未接入: 短暂休眠避免忙等, 等待 CAN 接入后启用 USE_REAL_CAN
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
    }
  }

private:
  int     timeout_cycles_ = 1;
  int64_t timeout_ns_     = 0;
  double  dvx_   = 0;
  double  dyaw_  = 0;
  double  dsa_   = 0;
  double  dax_   = 0;
  double  day_   = 0;
  int     dgear_ = 1;
  std::string can_iface_ = "can0";

  // ── CAN read-thread + buffer (Hybrid 模式) ──
  std::thread can_read_thread_;
  std::atomic<bool> stop_read_{false};
  std::mutex buffer_mutex_;
  EgoMotion latest_ego_;           // 线程安全 buffer, 存储最新 CAN 数据
  bool      buffer_valid_ = false;
  int64_t   last_can_update_ns_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleDataRxNode>());
  rclcpp::shutdown();
  return 0;
}
