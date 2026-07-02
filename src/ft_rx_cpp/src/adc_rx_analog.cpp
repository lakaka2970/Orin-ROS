// ============================================================================
// adc_rx_analog.cpp — ADC 模拟数据采集节点 (噪声池 / .bin 预加载)
// ============================================================================
// 数据源优先级 (自动降级):
//   1. 双半集文件: ctrx0(RX0-3) + ctrx1(RX4-7) 逐帧拼接 → 完整 16T16R 32 MiB/帧
//   2. 单文件目录扫描: data_dir 下 .bin 文件逐帧加载 (回退兼容)
//   3. 噪声池: int16 随机噪声 (±100), 4x 帧大小预生成
//
// 规格: 1024 chirps × 8 RX × 2048 samples × int16 = 32 MiB/帧
//   ctrx0: 512 chirps × 2 groups × 4 RX × 2048 samples = 8,388,608 int16 = 16 MiB (RX 0-3)
//   ctrx1: 512 chirps × 2 groups × 4 RX × 2048 samples = 8,388,608 int16 = 16 MiB (RX 4-7)
//   合并:  [ctrx0_half | ctrx1_half] = 16,777,216 int16 = 32 MiB
//
// 用法: 由 launch 文件通过 adc_source:=analog 参数自动选择本节点.
//
// 设计目标: 10 Hz × 32 MiB/帧 = 320 MiB/s 吞吐, FastDDS SHM 共享内存传输.
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
  constexpr double FPS      = 10.0;
  constexpr int    NOISE_LEVEL = 100;      // ± 噪声幅度
  constexpr int    POOL_FACTOR = 4;        // 噪声池倍数 (预生成池 = 帧大小 × 倍数)
  constexpr size_t MAX_PRELOAD_FILES = 30; // 最多预加载文件数 (~960 MB)
}

class AdcRxAnalogNode : public ft_rx::RxNodeBase<AdcRawData, AdcRxAnalogNode>
{
public:
  AdcRxAnalogNode()
    : RxNodeBase("adc_rx", "/adc/raw_data", 10, true)
  {
    declare_parameter("fps", FPS);
    declare_parameter("num_rows", 1024);                  // 总 chirp: 512 × 2 groups
    declare_parameter("num_chirps_per_row", 8);            // RX 天线数 (ctrx0:4 + ctrx1:4)
    declare_parameter("num_samples_per_chirp", 2048);
    declare_parameter("bin_file_ctrx0", "data/ctrx0_raw.bin");  // 8T8R 半集 0 (RX 0-3)
    declare_parameter("bin_file_ctrx1", "data/ctrx1_raw.bin");  // 8T8R 半集 1 (RX 4-7)
    declare_parameter("data_dir", "data");
    declare_parameter("fixed_frame", "radar");
    declare_parameter("max_preload_files", static_cast<int>(MAX_PRELOAD_FILES));
    declare_parameter("use_noise_pool", true);

    fps_             = get_parameter("fps").as_double();
    num_rows_        = static_cast<uint32_t>(get_parameter("num_rows").as_int());
    num_chirps_      = static_cast<uint32_t>(get_parameter("num_chirps_per_row").as_int());
    num_samples_     = static_cast<uint32_t>(get_parameter("num_samples_per_chirp").as_int());
    bin_file_ctrx0_  = get_parameter("bin_file_ctrx0").as_string();
    bin_file_ctrx1_  = get_parameter("bin_file_ctrx1").as_string();
    data_dir_        = get_parameter("data_dir").as_string();
    frame_id_        = get_parameter("fixed_frame").as_string();
    max_preload_     = static_cast<size_t>(get_parameter("max_preload_files").as_int());
    use_noise_pool_  = get_parameter("use_noise_pool").as_bool();

    total_elems_ = num_rows_ * num_chirps_ * num_samples_;  // int16 元素数 = 16,777,216
    frame_bytes_ = total_elems_ * sizeof(int16_t);           // 32 MiB

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

    // ── 数据源初始化 (三级降级: 双文件 → 单文件目录 → 噪声池) ──
    init_data_source();

    // ── 日志 ──
    const char *source_label = "噪声池";
    if (!file_buffers_.empty())
      source_label = use_dual_file_ ? "双半集文件 (ctrx0+ctrx1 合并)" : "单文件目录扫描";
    RCLCPP_INFO(get_logger(),
      "ADC Rx [analog]: %.0f Hz, %ux%ux%u, %.0f MB/帧, 数据源=%s, %zu 帧预加载 (%.0f MB RAM)",
      fps_, num_rows_, num_chirps_, num_samples_,
      frame_bytes_ / 1048576.0, source_label, file_buffers_.size(),
      (file_buffers_.size() * frame_bytes_) / 1048576.0);

    init_timer(fps_);
  }

  std::string frame_id() const { return frame_id_; }

  bool fill_message(AdcRawData &msg)
  {
    // ── 数据源 1: 预加载文件 (优先级最高) ──
    if (!file_buffers_.empty()) {
      msg.data = file_buffers_[file_index_];
      file_index_ = (file_index_ + 1) % file_buffers_.size();
      prof_.checkpoint("copy");
    }
    // ── 数据源 2: 噪声池 ──
    else if (!noise_pool_.empty()) {
      size_t max_offset = noise_pool_.size() - frame_bytes_;
      size_t offset = uniform_dist_(rng_,
        std::uniform_int_distribution<size_t>::param_type(0, max_offset));
      msg.data.assign(
        noise_pool_.begin() + static_cast<std::ptrdiff_t>(offset),
        noise_pool_.begin() + static_cast<std::ptrdiff_t>(offset + frame_bytes_));
      prof_.checkpoint("copy");
    }
    // ── 无数据源: 发布空消息 ──
    else {
      msg.data.clear();
    }

    msg.num_rows              = num_rows_;
    msg.num_chirps_per_row    = num_chirps_;
    msg.num_samples_per_chirp = num_samples_;

    // ── FPS 计数器 (每 2s 输出一次) ──
    {
      static int cnt = 0;
      static auto last = std::chrono::steady_clock::now();
      cnt++;
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - last).count();
      if (dt >= 2.0) {
        RCLCPP_INFO(get_logger(),
          "[ADC-RATE] analog: %.1f Hz  (%d frames in %.1fs)",
          cnt / dt, cnt, dt);
        cnt = 0;
        last = now;
      }
    }

    return true;
  }

private:
  // ──────────────────────────────────────────────────────────────────────────
  // 数据源初始化 (三级降级: 双半集合并 → 单文件扫描 → 噪声池)
  // ──────────────────────────────────────────────────────────────────────────
  void init_data_source()
  {
    // ── 优先级 1: 双半集文件 (ctrx0 + ctrx1 合并为完整 16T16R) ──
    if (try_load_dual_files()) return;

    // ── 优先级 2: 单文件目录扫描 (回退兼容) ──
    if (try_load_single_files()) return;

    // ── 优先级 3: 噪声池 ──
    if (use_noise_pool_) {
      size_t pool_elems = total_elems_ * POOL_FACTOR;
      size_t pool_bytes = pool_elems * sizeof(int16_t);
      noise_pool_.resize(pool_bytes);

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
    } else {
      RCLCPP_WARN(get_logger(), "所有数据源为空, 将发布空帧");
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 优先级 1: 双半集文件预加载 (ctrx0 + ctrx1 → 逐帧合并)
  // ──────────────────────────────────────────────────────────────────────────
  bool try_load_dual_files()
  {
    std::error_code ec;
    bool ctrx0_exists = fs::exists(bin_file_ctrx0_, ec);
    bool ctrx1_exists = fs::exists(bin_file_ctrx1_, ec);
    if (!ctrx0_exists || !ctrx1_exists) {
      if (ctrx0_exists || ctrx1_exists)
        RCLCPP_WARN(get_logger(),
          "仅找到一个半集文件 (%s), 跳过双文件模式, 回退到目录扫描",
          ctrx0_exists ? "ctrx0" : "ctrx1");
      return false;
    }

    size_t half_elems  = total_elems_ / 2;            // 8,388,608 int16/半集
    size_t half_bytes  = half_elems * sizeof(int16_t); // 16 MiB

    size_t sz0 = fs::file_size(bin_file_ctrx0_, ec);
    size_t sz1 = fs::file_size(bin_file_ctrx1_, ec);
    if (ec) return false;

    size_t n0 = sz0 / half_bytes;
    size_t n1 = sz1 / half_bytes;
    if (n0 < 1 || n1 < 1) {
      RCLCPP_WARN(get_logger(),
        "半集文件帧数不足: ctrx0=%zu ctrx1=%zu", n0, n1);
      return false;
    }

    size_t total_frames = std::min({n0, n1, max_preload_});
    if (n0 != n1)
      RCLCPP_WARN(get_logger(),
        "ctrx0 和 ctrx1 帧数不一致 (%zu vs %zu), 取较小值 %zu",
        n0, n1, total_frames);

    // ── 逐帧读取并拼接 ──
    std::ifstream f0(bin_file_ctrx0_, std::ios::binary);
    std::ifstream f1(bin_file_ctrx1_, std::ios::binary);
    if (!f0 || !f1) {
      RCLCPP_WARN(get_logger(), "无法打开半集文件");
      return false;
    }

    std::vector<uint8_t> half0(half_bytes);
    std::vector<uint8_t> half1(half_bytes);
    file_buffers_.reserve(total_frames);

    for (size_t i = 0; i < total_frames; ++i) {
      f0.read(reinterpret_cast<char *>(half0.data()),
              static_cast<std::streamsize>(half_bytes));
      f1.read(reinterpret_cast<char *>(half1.data()),
              static_cast<std::streamsize>(half_bytes));
      if (!f0 || !f1) {
        RCLCPP_WARN(get_logger(),
          "读取半集文件失败 (帧 %zu/%zu), 已加载 %zu 帧",
          i, total_frames, file_buffers_.size());
        break;
      }

      // 拼接: ctrx0(RX0-3) || ctrx1(RX4-7) → 完整 8 RX
      std::vector<uint8_t> merged;
      merged.reserve(frame_bytes_);
      merged.insert(merged.end(), half0.begin(), half0.end());
      merged.insert(merged.end(), half1.begin(), half1.end());
      file_buffers_.push_back(std::move(merged));
    }

    if (!file_buffers_.empty()) {
      use_dual_file_ = true;
      double total_mb = static_cast<double>(sz0 + sz1) / 1048576.0;
      RCLCPP_INFO(get_logger(),
        "bin 文件已预加载 (双半集): %zu 帧 (%.0f MB), "
        "来自 %s + %s",
        file_buffers_.size(), total_mb,
        bin_file_ctrx0_.c_str(), bin_file_ctrx1_.c_str());
      return true;
    }
    return false;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 优先级 2: 单文件目录扫描 (回退兼容 — 每个 .bin 作为独立帧)
  // ──────────────────────────────────────────────────────────────────────────
  bool try_load_single_files()
  {
    std::error_code ec;
    if (!fs::is_directory(data_dir_, ec)) return false;

    // 扫描 .bin 文件 (排除已知双半集文件, 避免重复加载)
    std::vector<fs::path> paths;
    for (auto &e : fs::directory_iterator(data_dir_, ec)) {
      if (e.path().extension() != ".bin") continue;
      // 跳过双半集文件 (已优先尝试过, 通过文件名匹配)
      std::string fname = e.path().filename().string();
      if (fname == fs::path(bin_file_ctrx0_).filename().string() ||
          fname == fs::path(bin_file_ctrx1_).filename().string()) continue;
      paths.push_back(e.path());
    }
    if (paths.empty()) return false;
    std::sort(paths.begin(), paths.end());

    size_t nload = std::min(paths.size(), max_preload_);
    file_buffers_.reserve(nload);
    for (size_t i = 0; i < nload; ++i) {
      std::ifstream f(paths[i], std::ios::binary | std::ios::ate);
      if (!f) {
        RCLCPP_WARN(get_logger(), "跳过: %s", paths[i].string().c_str());
        continue;
      }
      auto pos = f.tellg();
      if (pos <= 0) {
        RCLCPP_WARN(get_logger(), "空或无效文件, 跳过: %s", paths[i].string().c_str());
        continue;
      }
      size_t sz = std::min(static_cast<size_t>(pos), frame_bytes_);
      std::vector<uint8_t> buf(sz);
      f.seekg(0);
      f.read(reinterpret_cast<char *>(buf.data()), static_cast<std::streamsize>(sz));
      if (sz < frame_bytes_) buf.resize(frame_bytes_, 0);
      file_buffers_.push_back(std::move(buf));
    }

    if (!file_buffers_.empty()) {
      use_dual_file_ = false;
      RCLCPP_INFO(get_logger(),
        "单文件目录扫描: %zu 帧预加载 (%.0f MB RAM)",
        file_buffers_.size(),
        (file_buffers_.size() * frame_bytes_) / 1048576.0);
      return true;
    }
    return false;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 成员变量
  // ──────────────────────────────────────────────────────────────────────────
  uint32_t num_rows_    = 1024;
  uint32_t num_chirps_  = 8;
  uint32_t num_samples_ = 2048;
  size_t   total_elems_ = 0;
  size_t   frame_bytes_ = 0;
  size_t   max_preload_ = MAX_PRELOAD_FILES;
  bool     use_noise_pool_ = true;
  std::string data_dir_;
  std::string bin_file_ctrx0_;   // 8T8R 半集 0 (RX 0-3)
  std::string bin_file_ctrx1_;   // 8T8R 半集 1 (RX 4-7)
  std::string frame_id_ = "radar";

  // ── 文件数据源 ──
  bool     use_dual_file_ = false;   // 是否使用双半集合并模式
  std::vector<std::vector<uint8_t>> file_buffers_;
  size_t file_index_ = 0;

  // ── 噪声池数据源 ──
  std::vector<uint8_t> noise_pool_;  // int16 噪声池 (字节形式)
  std::mt19937 rng_{std::random_device{}()};
  std::uniform_int_distribution<size_t> uniform_dist_;

  // ── TF 广播器 ──
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bc_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdcRxAnalogNode>());
  rclcpp::shutdown();
  return 0;
}
