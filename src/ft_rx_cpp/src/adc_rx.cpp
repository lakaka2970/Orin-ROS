// ============================================================================
// adc_rx.cpp — ADC 数据采集节点 (预生成噪声池 → 15 Hz 零磁盘 I/O)
// ============================================================================
// 启动时预生成 int16 噪声池 (类似 Python 版噪声池优化), 每帧从池中随机切片.
// 可选: 若 data_dir 中存在 .bin 文件, 预加载至多 max_preload 帧用于真实数据循环.
//
// 设计目标: 15 Hz × 32 MB/帧 = 480 MB/s 吞吐, CycloneDDS SHM 传输.
// ============================================================================

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ft_rx_cpp/rx_node_base.hpp"
#include "ft_radar_msgs/msg/adc_raw_data.hpp"

using AdcRawData = ft_radar_msgs::msg::AdcRawData;
namespace fs = std::filesystem;

namespace {
  constexpr double FPS      = 15.0;
  constexpr int    NOISE_LEVEL = 100;      // ± 噪声幅度
  constexpr int    POOL_FACTOR = 4;        // 噪声池倍数 (预生成池 = 帧大小 × 倍数)
  constexpr size_t MAX_PRELOAD_FILES = 30; // 最多预加载文件数 (~960 MB)
}

class AdcRxNode : public ft_rx::RxNodeBase<AdcRawData, AdcRxNode>
{
public:
  AdcRxNode()
    : RxNodeBase("adc_rx", "/adc/raw_data", 10, true)
  {
    declare_parameter("fps", FPS);
    declare_parameter("num_rows", 512);
    declare_parameter("num_chirps_per_row", 16);
    declare_parameter("num_samples_per_chirp", 2048);
    declare_parameter("data_dir", "data");
    declare_parameter("fixed_frame", "radar");
    declare_parameter("max_preload_files", static_cast<int>(MAX_PRELOAD_FILES));
    declare_parameter("use_noise_pool", true);

    fps_            = get_parameter("fps").as_double();
    num_rows_       = static_cast<uint32_t>(get_parameter("num_rows").as_int());
    num_chirps_     = static_cast<uint32_t>(get_parameter("num_chirps_per_row").as_int());
    num_samples_    = static_cast<uint32_t>(get_parameter("num_samples_per_chirp").as_int());
    data_dir_       = get_parameter("data_dir").as_string();
    frame_id_       = get_parameter("fixed_frame").as_string();
    max_preload_    = static_cast<size_t>(get_parameter("max_preload_files").as_int());
    use_noise_pool_ = get_parameter("use_noise_pool").as_bool();

    total_elems_ = num_rows_ * num_chirps_ * num_samples_;  // int16 元素数
    frame_bytes_ = total_elems_ * sizeof(int16_t);           // 32 MB

    // ── 静态 TF (成员变量 — 生命周期必须覆盖整个节点) ──
    tf_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp    = builtin_interfaces::msg::Time();  // timestamp=0 → 永久有效
      tf.header.frame_id = "map";
      tf.child_frame_id  = frame_id_;
      tf.transform.translation.z = 0.5;
      tf.transform.rotation.w    = 1.0;
      tf_bc_->sendTransform(tf);
    }

    // ── 数据源初始化 ──
    scan_files();
    init_data_source();

    // ── 日志 ──
    if (!file_buffers_.empty()) {
      RCLCPP_INFO(get_logger(),
        "ADC Rx: %.0f Hz, %ux%ux%u, %.0f MB/帧, %zu 文件预加载 (%.0f MB RAM)",
        fps_, num_rows_, num_chirps_, num_samples_,
        frame_bytes_ / 1048576.0, file_buffers_.size(),
        (file_buffers_.size() * frame_bytes_) / 1048576.0);
    }
    if (use_noise_pool_ && noise_pool_.empty()) {
      RCLCPP_WARN(get_logger(), "噪声池为空, 将发布空帧");
    }
    RCLCPP_INFO(get_logger(),
      "ADC Rx: 数据源=%s, %.0f MB/帧",
      file_buffers_.empty() ? "噪声池" : "预加载文件",
      frame_bytes_ / 1048576.0);

    init_timer(fps_);
  }

  std::string frame_id() const { return frame_id_; }

  bool fill_message(AdcRawData &msg)
  {
    // ── 数据源 1: 预加载文件 (优先级最高) ──
    if (!file_buffers_.empty()) {
      msg.data = file_buffers_[file_index_];
      file_index_ = (file_index_ + 1) % file_buffers_.size();
      prof_.checkpoint("copy");       // ★ 测量 32 MB 拷贝耗时
    }
    // ── 数据源 2: 噪声池 ──
    else if (!noise_pool_.empty()) {
      size_t max_offset = noise_pool_.size() - frame_bytes_;
      size_t offset = uniform_dist_(rng_,
        std::uniform_int_distribution<size_t>::param_type(0, max_offset));
      msg.data.assign(
        noise_pool_.begin() + static_cast<std::ptrdiff_t>(offset),
        noise_pool_.begin() + static_cast<std::ptrdiff_t>(offset + frame_bytes_));
      prof_.checkpoint("copy");       // ★ 测量噪声切片拷贝耗时
    }
    // ── 无数据源: 发布空消息 ──
    else {
      msg.data.clear();
    }

    msg.num_rows              = num_rows_;
    msg.num_chirps_per_row    = num_chirps_;
    msg.num_samples_per_chirp = num_samples_;

    // ── 紧急 FPS 计数器 (不依赖 50 帧累积, 每 2s 输出一次) ──
    {
      static int cnt = 0;
      static auto last = std::chrono::steady_clock::now();
      cnt++;
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - last).count();
      if (dt >= 2.0) {
        RCLCPP_INFO(get_logger(),
          "[ADC-RATE] internal: %.1f Hz  (%d frames in %.1fs)",
          cnt / dt, cnt, dt);
        cnt = 0;
        last = now;
      }
    }

    return true;
  }

private:
  // ──────────────────────────────────────────────────────────────────────────
  // 文件扫描
  // ──────────────────────────────────────────────────────────────────────────
  void scan_files()
  {
    std::error_code ec;
    if (!fs::is_directory(data_dir_, ec)) {
      RCLCPP_WARN(get_logger(),
        "数据目录 '%s' 不存在, 将使用噪声池", data_dir_.c_str());
      return;
    }
    for (auto &e : fs::directory_iterator(data_dir_, ec))
      if (e.path().extension() == ".bin") paths_.push_back(e.path());
    std::sort(paths_.begin(), paths_.end());
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 数据源初始化 (文件预加载 + 噪声池)
  // ──────────────────────────────────────────────────────────────────────────
  void init_data_source()
  {
    // ── 预加载 .bin 文件 (限制数量, 避免内存溢出) ──
    if (!paths_.empty()) {
      size_t nload = std::min(paths_.size(), max_preload_);
      file_buffers_.reserve(nload);
      for (size_t i = 0; i < nload; ++i) {
        std::ifstream f(paths_[i], std::ios::binary | std::ios::ate);
        if (!f) {
          RCLCPP_WARN(get_logger(), "跳过: %s", paths_[i].string().c_str());
          continue;
        }
        size_t sz = std::min(static_cast<size_t>(f.tellg()), frame_bytes_);
        std::vector<uint8_t> buf(sz);
        f.seekg(0);
        f.read(reinterpret_cast<char *>(buf.data()), sz);
        if (sz < frame_bytes_) buf.resize(frame_bytes_, 0);  // 补齐
        file_buffers_.push_back(std::move(buf));
      }
      if (!file_buffers_.empty()) return;  // 文件数据优先, 不初始化噪声池
    }

    // ── 噪声池 (备选数据源) ──
    if (!use_noise_pool_) return;

    size_t pool_elems = total_elems_ * POOL_FACTOR;
    size_t pool_bytes = pool_elems * sizeof(int16_t);
    noise_pool_.resize(pool_bytes);

    // 分块填充: 避免 134 MB 的临时 vector
    auto *ptr = reinterpret_cast<int16_t *>(noise_pool_.data());
    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<int16_t> dist(
      static_cast<int16_t>(-NOISE_LEVEL),
      static_cast<int16_t>(NOISE_LEVEL));
    for (size_t i = 0; i < pool_elems; ++i)
      ptr[i] = dist(gen);

    RCLCPP_INFO(get_logger(),
      "噪声池已生成: %.0f MB (%zu 采样, ±%d)",
      pool_bytes / 1048576.0, pool_elems, NOISE_LEVEL);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 成员变量
  // ──────────────────────────────────────────────────────────────────────────
  uint32_t num_rows_    = 512;
  uint32_t num_chirps_  = 16;
  uint32_t num_samples_ = 2048;
  size_t   total_elems_ = 0;
  size_t   frame_bytes_ = 0;
  size_t   max_preload_ = MAX_PRELOAD_FILES;
  bool     use_noise_pool_ = true;
  std::string data_dir_;
  std::string frame_id_ = "radar";

  // ── 文件数据源 ──
  std::vector<fs::path> paths_;
  std::vector<std::vector<uint8_t>> file_buffers_;  // 预加载文件内容
  size_t file_index_ = 0;

  // ── 噪声池数据源 ──
  std::vector<uint8_t> noise_pool_;  // int16 噪声池 (字节形式)
  std::mt19937 rng_{std::random_device{}()};
  std::uniform_int_distribution<size_t> uniform_dist_;

  // ── TF 广播器 (成员变量 — 保持存活以支持 /tf_static latch) ──
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bc_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdcRxNode>());
  rclcpp::shutdown();
}
