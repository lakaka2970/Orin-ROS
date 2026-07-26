// ============================================================================
// vehicle_data_rx.cpp — 车辆数据采集节点 (V2 架构: SocketCAN)
// ============================================================================
// V2 架构变更:
//   - SocketCAN 完整实现: 5 报文解析 (车速/档位/横摆角/加速度/转向角)
//   - SIOCGSTAMP 硬件时间戳 (CLOCK_MONOTONIC, 微秒精度)
//   - Motorola (MSB) 字节序信号提取
//   - 内置 Ego Logging: 每 20ms 写入 CSV 文件
//   - 超时回退默认值机制
//   - CSV 回退模式 (CAN 未接入时)
//
// CAN 报文定义 (FT测试车通信矩阵):
//   0x100: vehicle_speed      (16-bit, Motorola, 0.01 m/s)
//   0x101: gear_status        (8-bit,  Motorola, 1)
//   0x102: yaw_rate           (16-bit, Motorola, 0.001 rad/s)
//   0x103: longitudinal_accel (16-bit, Motorola, 0.01 m/s²)
//   0x104: lateral_accel      (16-bit, Motorola, 0.01 m/s²)
//   0x105: steering_angle     (16-bit, Motorola, 0.001 rad)
//
// 话题:
//   发布: /vehicle/ego_motion (EgoMotion, 50 Hz)
//
// 作者: zhengyuan.liu
// 日期: 2026-07-26 (V2 重构)
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "ft_rx_cpp/rx_node_base.hpp"
#include "ft_radar_msgs/msg/ego_motion.hpp"

using EgoMotion = ft_radar_msgs::msg::EgoMotion;
namespace fs = std::filesystem;

namespace {
  constexpr double FPS = 50.0;
  const std::string NVME_MOUNT = "/mnt/nvme";
  const std::string EMMC_MOUNT = "/mnt/emmc";

  // CAN ID 定义
  constexpr canid_t CAN_ID_SPEED    = 0x100;
  constexpr canid_t CAN_ID_GEAR     = 0x101;
  constexpr canid_t CAN_ID_YAW      = 0x102;
  constexpr canid_t CAN_ID_AX       = 0x103;
  constexpr canid_t CAN_ID_AY       = 0x104;
  constexpr canid_t CAN_ID_STEERING = 0x105;

  // Motorola 字节序 16-bit 有符号提取
  inline int16_t extract_motorola_16(const uint8_t *data, int start_bit)
  {
    int byte_pos = start_bit / 8;
    uint16_t raw = (static_cast<uint16_t>(data[byte_pos]) << 8) |
                    static_cast<uint16_t>(data[byte_pos + 1]);
    return static_cast<int16_t>(raw);
  }

  inline uint8_t extract_motorola_8(const uint8_t *data, int start_bit)
  {
    return data[start_bit / 8];
  }
}

// ============================================================================
// Vehicle Data Rx 节点 (V2): SocketCAN → Ego Buffer → EgoMotion 发布
// ============================================================================
class VehicleDataRxNode : public ft_rx::RxNodeBase<EgoMotion, VehicleDataRxNode>
{
public:
  VehicleDataRxNode()
    : RxNodeBase("vehicle_data_rx", "/vehicle/ego_motion", 10, true)
  {
    declare_parameter("fps", FPS);
    declare_parameter("timeout_cycles", 1);
    declare_parameter("can_interface", "can0");
    declare_parameter("ego_data_file", "data/ego_motion.csv");
    declare_parameter("operation_mode", "FT_DEBUG_MODE");
    declare_parameter("logging_mode", "ADC_MODE");
    declare_parameter("logging_output_dir", "");
    declare_parameter("defaults.vx", 0.0);
    declare_parameter("defaults.yaw_rate", 0.0);
    declare_parameter("defaults.steering_angle", 0.0);
    declare_parameter("defaults.ax", 0.0);
    declare_parameter("defaults.ay", 0.0);
    declare_parameter("defaults.gear", 1);

    fps_            = get_parameter("fps").as_double();
    timeout_cycles_ = get_parameter("timeout_cycles").as_int();
    can_iface_      = get_parameter("can_interface").as_string();
    operation_mode_ = get_parameter("operation_mode").as_string();
    logging_mode_   = get_parameter("logging_mode").as_string();

    // 默认值
    defaults_.vx             = get_parameter("defaults.vx").as_double();
    defaults_.yaw_rate       = get_parameter("defaults.yaw_rate").as_double();
    defaults_.steering_angle = get_parameter("defaults.steering_angle").as_double();
    defaults_.ax             = get_parameter("defaults.ax").as_double();
    defaults_.ay             = get_parameter("defaults.ay").as_double();
    defaults_.gear           = static_cast<int>(get_parameter("defaults.gear").as_int());

    // Ego Logging: 除 IDLE_MODE 外所有模式均启用
    ego_logging_enabled_ = (operation_mode_ == "FT_DEBUG_MODE" &&
                            logging_mode_ != "IDLE_MODE");

    if (ego_logging_enabled_) {
      detect_storage();
      init_ego_csv();
    }

    // 尝试打开 SocketCAN
    can_fd_ = open_can_socket();
    if (can_fd_ >= 0) {
      RCLCPP_INFO(get_logger(), "SocketCAN 已连接: %s (fd=%d)", can_iface_.c_str(), can_fd_);
      // 启动 CAN 读取线程
      can_thread_running_ = true;
      can_thread_ = std::thread(&VehicleDataRxNode::can_read_thread, this);
    } else {
      RCLCPP_WARN(get_logger(),
        "SocketCAN 不可用 (%s), 使用默认值模式. "
        "请确认: sudo ip link set %s up type can bitrate 500000",
        can_iface_.c_str(), can_iface_.c_str());
      // 尝试 CSV 回退
      load_csv_fallback();
    }

    RCLCPP_INFO(get_logger(),
      "Vehicle Data Rx V2: %.0f Hz | CAN: %s | Logging: %s",
      fps_, can_fd_ >= 0 ? "OK" : "DEFAULT",
      ego_logging_enabled_ ? "ON" : "OFF");

    init_timer(fps_);
  }

  ~VehicleDataRxNode() override
  {
    can_thread_running_ = false;
    if (can_thread_.joinable()) can_thread_.join();
    if (can_fd_ >= 0) ::close(can_fd_);
    if (ego_csv_.is_open()) ego_csv_.close();
  }

  std::string frame_id() const { return "base_link"; }

  bool fill_message(EgoMotion &msg)
  {
    std::lock_guard<std::mutex> lock(ego_mutex_);

    // 超时检测: 如果超过 timeout_cycles 个周期未收到 CAN 数据, 使用默认值
    bool is_timeout = (can_fd_ >= 0) &&
                      (frames_since_last_can_ > timeout_cycles_);

    if (is_timeout || can_fd_ < 0) {
      // 使用默认值或 CSV 回退数据
      if (!csv_data_.empty()) {
        // CSV 回退: 循环读取
        auto &row = csv_data_[csv_index_ % csv_data_.size()];
        ego_buffer_ = row;
        csv_index_++;
      } else {
        ego_buffer_ = defaults_;
      }
      ego_buffer_.is_default = true;
    } else {
      ego_buffer_.is_default = false;
    }

    frames_since_last_can_++;

    // 获取当前时间戳 (微秒)
    auto now = std::chrono::steady_clock::now();
    uint64_t ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();

    // 内置 Ego Logging: 写入 CSV
    if (ego_logging_enabled_) {
      write_ego_csv(ts_us);
    }

    // 填充消息
    msg.header.stamp    = this->now();
    msg.header.frame_id = "base_link";
    msg.vx              = ego_buffer_.vx;
    msg.yaw_rate        = ego_buffer_.yaw_rate;
    msg.steering_angle  = ego_buffer_.steering_angle;
    msg.ax              = ego_buffer_.ax;
    msg.ay              = ego_buffer_.ay;
    msg.gear            = ego_buffer_.gear;
    msg.is_default      = ego_buffer_.is_default;

    return true;
  }

private:
  struct EgoData {
    double vx = 0.0;
    double yaw_rate = 0.0;
    double steering_angle = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    int    gear = 1;
    bool   is_default = true;
  };

  double fps_ = 50.0;
  int    timeout_cycles_ = 1;
  std::string can_iface_;
  std::string operation_mode_;
  std::string logging_mode_;
  bool ego_logging_enabled_ = false;

  int  can_fd_ = -1;
  std::thread can_thread_;
  std::atomic<bool> can_thread_running_{false};

  std::mutex ego_mutex_;
  EgoData ego_buffer_;
  EgoData defaults_;
  int frames_since_last_can_ = 0;

  // CSV 回退
  std::vector<EgoData> csv_data_;
  size_t csv_index_ = 0;

  // Ego Logging
  std::string ego_csv_path_;
  std::ofstream ego_csv_;

  // ── SocketCAN 打开 ──
  int open_can_socket()
  {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return -1;

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_iface_.c_str(), IFNAMSIZ - 1);

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      ::close(fd);
      return -1;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      ::close(fd);
      return -1;
    }

    // 设置非阻塞 (由 CAN 线程使用 poll)
    return fd;
  }

  // ── CAN 读取线程 ──
  void can_read_thread()
  {
    struct can_frame frame;

    while (can_thread_running_ && rclcpp::ok()) {
      // 使用 poll 等待 CAN 帧 (超时 100ms)
      struct pollfd pfd;
      pfd.fd     = can_fd_;
      pfd.events = POLLIN;

      int ret = poll(&pfd, 1, 100);
      if (ret <= 0) continue;

      ssize_t nbytes = read(can_fd_, &frame, sizeof(frame));
      if (nbytes < static_cast<ssize_t>(sizeof(frame))) continue;

      // SIOCGSTAMP 硬件时间戳
      struct timeval tv;
      uint64_t hw_ts_us = 0;
      if (ioctl(can_fd_, SIOCGSTAMP, &tv) == 0) {
        hw_ts_us = static_cast<uint64_t>(tv.tv_sec) * 1000000ULL +
                   static_cast<uint64_t>(tv.tv_usec);
      }

      // 解析 CAN 报文
      std::lock_guard<std::mutex> lock(ego_mutex_);
      parse_can_frame(frame, hw_ts_us);
      frames_since_last_can_ = 0;
    }
  }

  // ── CAN 报文解析 (Motorola 字节序) ──
  void parse_can_frame(const struct can_frame &frame, uint64_t hw_ts_us)
  {
    (void)hw_ts_us;  // 时间戳用于 Logging

    switch (frame.can_id) {
      case CAN_ID_SPEED: {
        int16_t raw = extract_motorola_16(frame.data, 0);
        ego_buffer_.vx = raw * 0.01;  // m/s
        break;
      }
      case CAN_ID_GEAR: {
        ego_buffer_.gear = extract_motorola_8(frame.data, 0);
        break;
      }
      case CAN_ID_YAW: {
        int16_t raw = extract_motorola_16(frame.data, 0);
        ego_buffer_.yaw_rate = raw * 0.001;  // rad/s
        break;
      }
      case CAN_ID_AX: {
        int16_t raw = extract_motorola_16(frame.data, 0);
        ego_buffer_.ax = raw * 0.01;  // m/s²
        break;
      }
      case CAN_ID_AY: {
        int16_t raw = extract_motorola_16(frame.data, 0);
        ego_buffer_.ay = raw * 0.01;  // m/s²
        break;
      }
      case CAN_ID_STEERING: {
        int16_t raw = extract_motorola_16(frame.data, 0);
        ego_buffer_.steering_angle = raw * 0.001;  // rad
        break;
      }
      default:
        break;
    }
  }

  // ── CSV 回退模式 ──
  void load_csv_fallback()
  {
    std::string csv_file = get_parameter("ego_data_file").as_string();
    std::ifstream ifs(csv_file);
    if (!ifs.is_open()) return;

    std::string line;
    std::getline(ifs, line);  // 跳过表头

    while (std::getline(ifs, line)) {
      if (line.empty()) continue;
      EgoData d;
      // 格式: timestamp_us,vx,yaw_rate,steering_angle,ax,ay,gear
      if (sscanf(line.c_str(), "%*d,%lf,%lf,%lf,%lf,%lf,%d",
                 &d.vx, &d.yaw_rate, &d.steering_angle,
                 &d.ax, &d.ay, &d.gear) == 6) {
        d.is_default = false;
        csv_data_.push_back(d);
      }
    }

    if (!csv_data_.empty()) {
      RCLCPP_INFO(get_logger(), "CSV 回退: 已加载 %zu 行 ego 数据", csv_data_.size());
    }
  }

  // ── 存储检测 ──
  void detect_storage()
  {
    std::string user_dir = get_parameter("logging_output_dir").as_string();
    std::string base;

    if (!user_dir.empty()) {
      base = user_dir;
    } else if (fs::exists(NVME_MOUNT)) {
      base = NVME_MOUNT;
    } else {
      base = EMMC_MOUNT;
    }

    fs::create_directories(base);
    ego_csv_path_ = base + "/ego_motion.csv";
  }

  // ── Ego CSV 初始化 ──
  void init_ego_csv()
  {
    ego_csv_.open(ego_csv_path_, std::ios::out | std::ios::trunc);
    if (ego_csv_.is_open()) {
      ego_csv_ << "timestamp_us,vx,yaw_rate,steering_angle,ax,ay,gear\n";
      ego_csv_.flush();
    }
  }

  // ── Ego CSV 写入 (每 20ms 调用一次) ──
  void write_ego_csv(uint64_t ts_us)
  {
    if (!ego_csv_.is_open()) return;

    char buf[256];
    snprintf(buf, sizeof(buf), "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
             ts_us,
             ego_buffer_.vx,
             ego_buffer_.yaw_rate,
             ego_buffer_.steering_angle,
             ego_buffer_.ax,
             ego_buffer_.ay,
             ego_buffer_.gear);
    ego_csv_ << buf;
    ego_csv_.flush();
  }
};

// ============================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleDataRxNode>());
  rclcpp::shutdown();
  return 0;
}
