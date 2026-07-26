// ============================================================================
// adc_rx.cpp — ADC 数据采集节点 (V2 架构)
// ============================================================================
// 双 CTRX V4L2 mmap 采集 → 同步写入文件 → 发布文件路径消息
//
// V2 架构变更:
//   - 文件路径发布: 替代 32MB AdcRawData DDS 传输, 带宽降低 99.99%
//   - 内置 Logging: ADC 数据直接写入 NVMe SSD / eMMC, 无需独立 logging_node
//   - V4L2 硬件时间戳: v4l2_buffer.timestamp (CLOCK_MONOTONIC, 微秒精度)
//   - Warm-up 机制: 启动后 flush V4L2 预填充 buffer, 消除排空效应
//   - NVMe SSD 双存储: 自动检测 NVMe, 回退 eMMC + 最大帧数限制
//   - 运行模式: FT_DEBUG_MODE (含 Logging) / FT_RUNNING_MODE (仅实时处理)
//
// 数据格式: RG12 (12-bit padded to 16-bit), 8192×1024 per device
//   单设备:  8192 × 1024 × 2B = 16 MiB (4 RX)
//   双设备拼接: 16 MiB × 2 = 32 MiB (8 RX, 1024 chirps × 8 RX × 2048 samples)
//
// 话题:
//   发布: /adc/file_path (AdcFilePath, 15 Hz)
//   发布: /system/stop_all (std_msgs/Bool, 事件触发)
//
// 作者: zhengyuan.liu
// 日期: 2026-07-26 (V2 重构)
// ============================================================================

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/statvfs.h>
#include <unistd.h>
#include <linux/videodev2.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ft_rx_cpp/rx_node_base.hpp"
#include "ft_radar_msgs/msg/adc_file_path.hpp"

using AdcFilePath = ft_radar_msgs::msg::AdcFilePath;
namespace fs = std::filesystem;

namespace {
  constexpr uint32_t DEFAULT_WIDTH  = 8192;   // 4 RX × 2048 samples
  constexpr uint32_t DEFAULT_HEIGHT = 1024;   // 512 chirps × 2 groups
  constexpr uint32_t NUM_V4L2_BUFS  = 4;
  constexpr uint32_t BYPASS_MODE_CID = 0x009a2064;

  // 存储路径
  const std::string NVME_MOUNT = "/mnt/nvme";
  const std::string EMMC_MOUNT = "/mnt/emmc";
}

// ============================================================================
// V4L2 单设备管理器
// ============================================================================
struct V4L2Device {
  struct Buffer {
    void   *start  = nullptr;
    size_t  length = 0;
  };

  std::string path;
  std::string label;
  int         fd          = -1;
  bool        streaming   = false;
  uint32_t    width       = DEFAULT_WIDTH;
  uint32_t    height      = DEFAULT_HEIGHT;
  size_t      frame_bytes = 0;
  uint32_t    buf_index   = 0;
  std::vector<Buffer> buffers;

  // V4L2 硬件时间戳 (CLOCK_MONOTONIC)
  uint64_t    last_hw_timestamp_us = 0;

  bool init()
  {
    if (fd >= 0) close_noexcept();

    fd = ::open(path.c_str(), O_RDWR);
    if (fd < 0) {
      fprintf(stderr, "[adc_rx] %s: 无法打开 '%s': %s\n",
              label.c_str(), path.c_str(), strerror(errno));
      return false;
    }

    if (!query_cap() || !set_format() || !set_bypass() || !init_mmap() || !start_streaming()) {
      close_noexcept();
      return false;
    }

    fprintf(stderr, "[adc_rx] %s: 已连接 fd=%d (%ux%u RG12, %.1f KB/帧)\n",
            label.c_str(), fd, width, height, frame_bytes / 1024.0);
    return true;
  }

  // 阻塞出队, 返回 V4L2 硬件时间戳
  bool dequeue(uint8_t *&data, size_t &bytes_used, uint64_t &hw_timestamp_us)
  {
    struct v4l2_buffer buf;
    std::memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
      fprintf(stderr, "[adc_rx] %s: VIDIOC_DQBUF 失败: %s\n",
              label.c_str(), strerror(errno));
      return false;
    }

    buf_index  = buf.index;
    data       = static_cast<uint8_t *>(buffers[buf.index].start);
    bytes_used = buf.bytesused;

    // V4L2 硬件时间戳: struct timeval → 微秒
    hw_timestamp_us = static_cast<uint64_t>(buf.timestamp.tv_sec) * 1000000ULL
                    + static_cast<uint64_t>(buf.timestamp.tv_usec);
    last_hw_timestamp_us = hw_timestamp_us;
    return true;
  }

  void enqueue(uint32_t index)
  {
    struct v4l2_buffer buf;
    std::memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = index;
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
      fprintf(stderr, "[adc_rx] %s: VIDIOC_QBUF 失败: %s\n",
              label.c_str(), strerror(errno));
  }

  // Flush 所有已入队 buffer (warm-up 后调用, 消除排空效应)
  void flush_buffers()
  {
    if (fd < 0 || !streaming) return;

    struct v4l2_buffer buf;
    while (true) {
      std::memset(&buf, 0, sizeof(buf));
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0)
        break;  // 无更多 buffer
      // 立即归还
      enqueue(buf.index);
    }
  }

  void stop_and_close()
  {
    if (fd >= 0 && streaming) {
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ioctl(fd, VIDIOC_STREAMOFF, &type);
      streaming = false;
    }
    for (auto &b : buffers) {
      if (b.start && b.start != MAP_FAILED)
        munmap(b.start, b.length);
    }
    buffers.clear();
    if (fd >= 0) { ::close(fd); fd = -1; }
  }

private:
  void close_noexcept() { if (fd >= 0) { ::close(fd); fd = -1; } }

  bool query_cap()
  {
    struct v4l2_capability cap;
    std::memset(&cap, 0, sizeof(cap));
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) return false;
    return (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
           (cap.capabilities & V4L2_CAP_STREAMING);
  }

  bool set_format()
  {
    struct v4l2_format fmt;
    std::memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB12;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) return false;
    if (fmt.fmt.pix.width != width || fmt.fmt.pix.height != height) {
      width  = fmt.fmt.pix.width;
      height = fmt.fmt.pix.height;
    }
    frame_bytes = (fmt.fmt.pix.sizeimage > 0)
                    ? fmt.fmt.pix.sizeimage
                    : static_cast<size_t>(width) * height * 2;
    return true;
  }

  bool set_bypass()
  {
    struct v4l2_control ctrl;
    std::memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id    = BYPASS_MODE_CID;
    ctrl.value = 0;
    ioctl(fd, VIDIOC_S_CTRL, &ctrl);  // 非致命
    return true;
  }

  bool init_mmap()
  {
    struct v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.count  = NUM_V4L2_BUFS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) return false;

    buffers.resize(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
      struct v4l2_buffer buf;
      std::memset(&buf, 0, sizeof(buf));
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index  = i;
      if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) return false;

      buffers[i].length = buf.length;
      buffers[i].start  = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, buf.m.offset);
      if (buffers[i].start == MAP_FAILED) return false;
    }
    return true;
  }

  bool start_streaming()
  {
    for (uint32_t i = 0; i < buffers.size(); ++i) {
      struct v4l2_buffer buf;
      std::memset(&buf, 0, sizeof(buf));
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index  = i;
      if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) return false;
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) return false;
    streaming = true;
    return true;
  }
};

// ============================================================================
// ADC Rx 节点 (V2): 双设备 V4L2 → 文件写入 → AdcFilePath 发布
// ============================================================================
// V4L2 buffer 生命周期:
//   DQBUF → 数据落盘(Logging) + 发布文件路径 → 等待 RSP processing_complete → QBUF
//   buffer 在 RSP 处理完成前不释放, 确保 RSP 读取期间数据不被覆盖.
// ============================================================================
class AdcRxNode : public ft_rx::RxNodeBase<AdcFilePath, AdcRxNode>
{
public:
  AdcRxNode()
    : RxNodeBase("adc_rx", "/adc/file_path", 10, true)
  {
    // ── 参数声明 ──
    declare_parameter("fps", 15.0);
    declare_parameter("capture_width", static_cast<int>(DEFAULT_WIDTH));
    declare_parameter("capture_height", static_cast<int>(DEFAULT_HEIGHT));
    declare_parameter("device_path_ctrx0", "/dev/radar_ctrx0");
    declare_parameter("device_path_ctrx1", "/dev/radar_ctrx1");
    declare_parameter("fixed_frame", "radar");

    // V2: 运行模式与 Logging 配置
    declare_parameter("operation_mode", "FT_DEBUG_MODE");  // FT_DEBUG_MODE | FT_RUNNING_MODE
    declare_parameter("logging_mode", "ADC_MODE");         // ADC_MODE | RD_CELL_LIST_MODE | DET_LIST_MODE | IDLE_MODE
    declare_parameter("logging_output_dir", "");           // 空=自动检测 NVMe/eMMC
    declare_parameter("logging_max_frames", 100);          // eMMC 模式最大帧数
    declare_parameter("enable_warmup", true);
    declare_parameter("warmup_sec", 5.0);
    declare_parameter("rsp_timeout_ms", 100);              // 等待 RSP 处理完成的超时 (ms)

    // 读取参数
    uint32_t w = static_cast<uint32_t>(get_parameter("capture_width").as_int());
    uint32_t h = static_cast<uint32_t>(get_parameter("capture_height").as_int());
    frame_id_  = get_parameter("fixed_frame").as_string();
    operation_mode_ = get_parameter("operation_mode").as_string();
    logging_mode_   = get_parameter("logging_mode").as_string();
    max_frames_     = static_cast<int>(get_parameter("logging_max_frames").as_int());
    enable_warmup_  = get_parameter("enable_warmup").as_bool();
    warmup_sec_     = get_parameter("warmup_sec").as_double();

    // 判断是否启用 ADC Logging (仅 ADC_MODE + FT_DEBUG_MODE)
    adc_logging_enabled_ = (operation_mode_ == "FT_DEBUG_MODE" &&
                            logging_mode_ == "ADC_MODE");

    // ── 存储路径检测 ──
    detect_storage();

    // ── 初始化 V4L2 设备 ──
    dev0_.path = get_parameter("device_path_ctrx0").as_string();
    dev0_.label = "ctrx0";
    dev0_.width = w;  dev0_.height = h;

    dev1_.path = get_parameter("device_path_ctrx1").as_string();
    dev1_.label = "ctrx1";
    dev1_.width = w;  dev1_.height = h;

    bool ok0 = dev0_.init();
    bool ok1 = dev1_.init();

    if (!ok0) RCLCPP_ERROR(get_logger(), "ctrx0 (%s) 初始化失败", dev0_.path.c_str());
    if (!ok1) RCLCPP_ERROR(get_logger(), "ctrx1 (%s) 初始化失败", dev1_.path.c_str());

    // ── 静态 TF ──
    tf_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp    = builtin_interfaces::msg::Time();
      tf.header.frame_id = "map";
      tf.child_frame_id  = frame_id_;
      tf.transform.translation.z = 0.5;
      tf.transform.rotation.w    = 1.0;
      tf_bc_->sendTransform(tf);
    }

    // ── 自动停止发布者 ──
    stop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/system/stop_all", ft_rx::rx_qos(10, false));

    // ── RSP 处理完成信号订阅 (buffer 释放同步) ──
    rsp_timeout_ms_ = static_cast<int>(get_parameter("rsp_timeout_ms").as_int());
    rsp_complete_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/system/processing_complete", ft_rx::rx_qos(10, true),
        [this](std_msgs::msg::Bool::SharedPtr) {
          std::lock_guard<std::mutex> lock(rsp_mutex_);
          rsp_done_ = true;
          rsp_cv_.notify_one();
        });

    double fps_val = get_parameter("fps").as_double();

    RCLCPP_INFO(get_logger(),
      "ADC Rx V2: %.0f Hz | 模式: %s | Logging: %s | 存储: %s (%s) | warm-up: %s",
      fps_val,
      operation_mode_.c_str(),
      adc_logging_enabled_ ? "ADC_MODE" : "OFF",
      storage_path_.c_str(),
      storage_type_.c_str(),
      enable_warmup_ ? "ON" : "OFF");

    // ── Warm-up: flush V4L2 预填充 buffer ──
    if (enable_warmup_ && ok0 && ok1) {
      RCLCPP_INFO(get_logger(), "Warm-up: 等待 %.1f 秒后 flush V4L2 buffers...", warmup_sec_);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(warmup_sec_ * 1000)));
      dev0_.flush_buffers();
      dev1_.flush_buffers();
      RCLCPP_INFO(get_logger(), "Warm-up 完成, V4L2 buffers 已清空");
    }

    start_polling_loop(fps_val);
  }

  ~AdcRxNode() override
  {
    stop_polling_ = true;
    dev0_.stop_and_close();
    dev1_.stop_and_close();

    if (adc_logging_enabled_) {
      RCLCPP_INFO(get_logger(),
        "ADC Logging 统计: 已写入 %d 帧, 总大小 %.2f GB",
        frames_logged_, total_bytes_logged_ / 1073741824.0);
    }
  }

  std::string frame_id() const { return frame_id_; }

  bool fill_message(AdcFilePath &msg)
  {
    // ── 设备断线重连 ──
    if (dev0_.fd < 0 || dev1_.fd < 0) {
      if (++reconnect_counter_ >= RECONNECT_INTERVAL) {
        reconnect_counter_ = 0;
        if (dev0_.fd < 0) dev0_.init();
        if (dev1_.fd < 0) dev1_.init();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      return false;
    }

    // ── 检查是否达到最大帧数 ──
    if (adc_logging_enabled_ && max_frames_ > 0 && frames_logged_ >= max_frames_) {
      RCLCPP_INFO_ONCE(get_logger(), "达到最大帧数限制 (%d), 发布停止信号", max_frames_);
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = true;
      stop_pub_->publish(stop_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return false;
    }

    // ── 双设备阻塞出队 ──
    uint8_t *buf0 = nullptr, *buf1 = nullptr;
    size_t   bytes0 = 0, bytes1 = 0;
    uint64_t hw_ts0 = 0, hw_ts1 = 0;

    if (!dev0_.dequeue(buf0, bytes0, hw_ts0)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "ctrx0 DQBUF 失败");
      dev0_.stop_and_close();
      return false;
    }
    if (!dev1_.dequeue(buf1, bytes1, hw_ts1)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "ctrx1 DQBUF 失败");
      dev0_.enqueue(dev0_.buf_index);
      dev1_.stop_and_close();
      return false;
    }
    prof_.checkpoint("v4l2_dequeue");

    // ── 使用 ctrx0 的硬件时间戳作为帧时间戳 ──
    uint64_t frame_ts_us = hw_ts0;

    // ── 内置 Logging: 写入文件 (与 RSP 读取并行) ──
    std::string file_path;
    uint64_t file_size = 0;

    if (adc_logging_enabled_) {
      file_path = adc_data_dir_ + "/" + std::to_string(frame_ts_us) + ".bin";
      if (write_adc_file(file_path, buf0, bytes0, buf1, bytes1, frame_ts_us)) {
        file_size = bytes0 + bytes1;
        frames_logged_++;
        total_bytes_logged_ += file_size;
      }
    }
    prof_.checkpoint("file_write");

    // ── 填充 AdcFilePath 消息 (发布后 RSP 从文件读取) ──
    msg.header.stamp.sec     = static_cast<int32_t>(frame_ts_us / 1000000ULL);
    msg.header.stamp.nanosec = static_cast<uint32_t>((frame_ts_us % 1000000ULL) * 1000ULL);
    msg.header.frame_id      = frame_id_;

    msg.file_path             = file_path;
    msg.file_size             = file_size;
    msg.num_rows              = dev0_.height;           // 1024
    msg.num_chirps_per_row    = 8;                      // ctrx0:4 + ctrx1:4
    msg.num_samples_per_chirp = dev0_.width / 4;        // 2048
    msg.file_ready            = !file_path.empty();

    // ── 等待 RSP 处理完成后释放 V4L2 buffer ──
    // buffer 生命周期: DQBUF → 落盘+发布 → 等待RSP完成 → QBUF
    {
      std::unique_lock<std::mutex> lock(rsp_mutex_);
      rsp_done_ = false;
      // 消息将由基类发布, RSP 收到后处理并发布 processing_complete
      rsp_cv_.wait_for(lock, std::chrono::milliseconds(rsp_timeout_ms_),
                       [this]() { return rsp_done_; });
    }
    prof_.checkpoint("rsp_wait");

    // 释放 V4L2 buffer (RSP 已完成或超时)
    dev0_.enqueue(dev0_.buf_index);
    dev1_.enqueue(dev1_.buf_index);

    return true;
  }

private:
  V4L2Device dev0_;
  V4L2Device dev1_;

  std::string frame_id_ = "radar";
  std::string operation_mode_;
  std::string logging_mode_;
  bool   adc_logging_enabled_ = false;
  bool   enable_warmup_ = true;
  double warmup_sec_ = 5.0;

  // 存储
  std::string storage_path_;
  std::string storage_type_;   // "NVME_SSD" | "EMMC"
  std::string adc_data_dir_;
  int max_frames_ = 100;
  int frames_logged_ = 0;
  uint64_t total_bytes_logged_ = 0;

  // 重连
  static constexpr int RECONNECT_INTERVAL = 90;
  int reconnect_counter_ = 0;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bc_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;

  // ── RSP 处理完成同步 (buffer 释放控制) ──
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rsp_complete_sub_;
  std::mutex rsp_mutex_;
  std::condition_variable rsp_cv_;
  bool rsp_done_ = false;
  int  rsp_timeout_ms_ = 100;

  // ── 存储检测 ──
  void detect_storage()
  {
    std::string user_dir = get_parameter("logging_output_dir").as_string();

    if (!user_dir.empty()) {
      storage_path_ = user_dir;
      storage_type_ = "USER";
    } else if (fs::exists(NVME_MOUNT) && fs::is_directory(NVME_MOUNT)) {
      storage_path_ = NVME_MOUNT;
      storage_type_ = "NVME_SSD";
    } else {
      storage_path_ = EMMC_MOUNT;
      storage_type_ = "EMMC";
      // eMMC 模式: 根据磁盘空间计算最大帧数
      calculate_max_frames_emmc();
    }

    adc_data_dir_ = storage_path_ + "/adc_data";

    if (adc_logging_enabled_) {
      fs::create_directories(adc_data_dir_);
      RCLCPP_INFO(get_logger(), "ADC 存储: %s (%s), 最大帧数: %d",
                  adc_data_dir_.c_str(), storage_type_.c_str(), max_frames_);
    }
  }

  void calculate_max_frames_emmc()
  {
    struct statvfs stat;
    if (statvfs(storage_path_.c_str(), &stat) == 0) {
      uint64_t available_bytes = stat.f_bavail * stat.f_frsize;
      uint64_t safe_bytes = static_cast<uint64_t>(available_bytes * 0.9);  // 预留 10%
      uint64_t frame_size = 33554432;  // 32 MiB
      int computed_max = static_cast<int>(safe_bytes / frame_size);
      if (computed_max < max_frames_) {
        max_frames_ = computed_max;
        RCLCPP_WARN(get_logger(),
          "eMMC 空间有限: 可用 %.1f GB, 最大帧数限制为 %d",
          safe_bytes / 1073741824.0, max_frames_);
      }
    }
  }

  // ── ADC 文件写入 (20字节头 + 原始数据) ──
  bool write_adc_file(const std::string &path,
                      const uint8_t *buf0, size_t bytes0,
                      const uint8_t *buf1, size_t bytes1,
                      uint64_t timestamp_us)
  {
    FILE *fp = fopen(path.c_str(), "wb");
    if (!fp) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "无法写入 ADC 文件: %s", path.c_str());
      return false;
    }

    // 20 字节文件头: magic(4) + version(4) + timestamp_us(8) + data_size(4)
    uint32_t magic = 0x46544144;  // "FTAD"
    uint32_t version = 2;
    uint32_t data_size = static_cast<uint32_t>(bytes0 + bytes1);

    fwrite(&magic, 4, 1, fp);
    fwrite(&version, 4, 1, fp);
    fwrite(&timestamp_us, 8, 1, fp);
    fwrite(&data_size, 4, 1, fp);

    // 原始数据: ctrx0 || ctrx1
    fwrite(buf0, 1, bytes0, fp);
    fwrite(buf1, 1, bytes1, fp);

    fflush(fp);
    fclose(fp);
    return true;
  }
};

// ============================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdcRxNode>());
  rclcpp::shutdown();
  return 0;
}
