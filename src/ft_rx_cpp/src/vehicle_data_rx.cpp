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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <mutex>
#include <sstream>
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
    declare_parameter("ego_data_file", "data/ego_motion.csv");

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
      // 从 buffer 拷贝最新 CAN 数据 (仅拷贝数据字段, 保留 header 由 execute_frame 设置)
      msg.vx              = latest_ego_.vx;
      msg.yaw_rate        = latest_ego_.yaw_rate;
      msg.steering_angle  = latest_ego_.steering_angle;
      msg.ax              = latest_ego_.ax;
      msg.ay              = latest_ego_.ay;
      msg.gear            = latest_ego_.gear;
      msg.is_default      = false;
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

  // ── 数据读取线程: 从 CSV 文件读取 ego-motion 假数据, 持续循环 ──
  // 文件格式:  vx,yaw_rate,steering_angle,ax,ay,gear (CSV with header)
  // 按 fps 速率逐行读取 → 更新 buffer → timer 发布时附上当前时间戳.
  // EOF 时回到数据首行继续循环.  文件缺失时回退为静默等待.
  void can_read_loop()
  {
    std::string ego_file = get_parameter("ego_data_file").as_string();
    auto period = std::chrono::duration<double>(1.0 / fps_);

    while (rclcpp::ok() && !stop_read_) {
      std::ifstream file(ego_file);
      if (!file.is_open()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "ego-motion 数据文件未找到: %s, 1s 后重试...", ego_file.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      // 跳过表头
      std::string header;
      std::getline(file, header);

      RCLCPP_INFO(get_logger(),
        "ego-motion 数据文件已打开: %s", ego_file.c_str());

      std::string line;
      while (rclcpp::ok() && !stop_read_) {
        if (!std::getline(file, line)) {
          // EOF — 回到数据首行继续循环
          file.clear();
          file.seekg(0);
          std::getline(file, header);  // 跳过表头
          continue;
        }

        if (line.empty()) continue;

        // 解析: vx,yaw_rate,steering_angle,ax,ay,gear
        // 将逗号替换为空格以简化解析
        std::replace(line.begin(), line.end(), ',', ' ');
        double vx = 0, yaw = 0, steering = 0, ax = 0, ay = 0;
        int gear = 1;
        std::istringstream iss(line);
        if (!(iss >> vx >> yaw >> steering >> ax >> ay >> gear)) {
          continue;  // 格式错误, 跳过
        }

        {
          std::lock_guard<std::mutex> lock(buffer_mutex_);
          latest_ego_.vx             = vx;
          latest_ego_.yaw_rate       = yaw;
          latest_ego_.steering_angle = steering;
          latest_ego_.ax             = ax;
          latest_ego_.ay             = ay;
          latest_ego_.gear           = gear;
          latest_ego_.is_default     = false;
          buffer_valid_              = true;
          last_can_update_ns_        = this->now().nanoseconds();
        }

        // 等待下一个帧间隔
        std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period));
      }
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
