// ============================================================================
// adc_rx.cpp — ADC 真实数据采集节点 (V4L2 mmap streaming)
// ============================================================================
// 使用 V4L2 mmap streaming 从 Infineon CTRX 雷达 sensor 逐帧采集原始 ADC 数据.
// 替换旧的 open()/read() 字符设备模式.
//
// 数据格式: RG12 (12-bit padded to 16-bit), 分辨率可配置
// 设备: /dev/video0 (可通过 device_path 参数配置)
//
// 用法: adc_rx_cpp (默认), 由 launch 文件通过 adc_source:=real 选择.
//
// V4L2 流程:
//   open → QUERYCAP → S_FMT → S_CTRL(bypass_mode) → REQBUFS → QUERYBUF×N →
//   mmap×N → QBUF×N → STREAMON → (DQBUF→copy→QBUF)×… → STREAMOFF → munmap → close
//
// 设计目标: 30 Hz, CycloneDDS SHM 传输.
// ============================================================================

#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/videodev2.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ft_rx_cpp/rx_node_base.hpp"
#include "ft_radar_msgs/msg/adc_raw_data.hpp"

using AdcRawData = ft_radar_msgs::msg::AdcRawData;

namespace {
  constexpr double   FPS           = 30.0;
  constexpr uint32_t DEFAULT_WIDTH  = 2048;
  constexpr uint32_t DEFAULT_HEIGHT = 512;
  constexpr uint32_t NUM_V4L2_BUFS  = 4;
}

class AdcRxNode : public ft_rx::RxNodeBase<AdcRawData, AdcRxNode>
{
public:
  AdcRxNode()
    : RxNodeBase("adc_rx", "/adc/raw_data", 10, true)
  {
    declare_parameter("fps", FPS);
    declare_parameter("capture_width", static_cast<int>(DEFAULT_WIDTH));
    declare_parameter("capture_height", static_cast<int>(DEFAULT_HEIGHT));
    declare_parameter("device_path", "/dev/video0");
    declare_parameter("fixed_frame", "radar");

    fps_         = get_parameter("fps").as_double();
    width_       = static_cast<uint32_t>(get_parameter("capture_width").as_int());
    height_      = static_cast<uint32_t>(get_parameter("capture_height").as_int());
    device_path_ = get_parameter("device_path").as_string();
    frame_id_    = get_parameter("fixed_frame").as_string();

    // RG12: 每个像素 2 字节 (12-bit 数据 + 4-bit 零填充)
    frame_bytes_ = static_cast<size_t>(width_) * height_ * 2;

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

    // ── 打开 V4L2 数据源 ──
    open_device();

    RCLCPP_INFO(get_logger(),
      "ADC Rx [V4L2]: %.0f Hz, %ux%u RG12, %.1f KB/帧, 设备=%s (%s)",
      fps_, width_, height_, frame_bytes_ / 1024.0,
      device_path_.c_str(),
      v4l2_fd_ >= 0 ? "已连接" : "未连接 — 将发布空帧");

    init_timer(fps_);
  }

  ~AdcRxNode() override
  {
    stop_streaming();
    close_device();
  }

  std::string frame_id() const { return frame_id_; }

  bool fill_message(AdcRawData &msg)
  {
    if (v4l2_fd_ < 0) {
      // 设备未连接: 尝试重新打开
      open_device();
      msg.data.clear();
    } else {
      uint8_t *buf = nullptr;
      size_t   bytes_used = 0;

      if (dequeue_buffer(buf, bytes_used)) {
        msg.data.assign(buf, buf + bytes_used);
        enqueue_buffer(v4l2_buf_index_);
        prof_.checkpoint("v4l2_dequeue");
      } else {
        // 无可用帧 (非阻塞, 驱动尚未填充) — 发布空帧
        msg.data.clear();
      }
    }

    msg.num_rows              = height_;
    msg.num_chirps_per_row    = 1;
    msg.num_samples_per_chirp = width_;

    // ── FPS 计数器 (每 2s 输出一次) ──
    {
      static int   cnt  = 0;
      static auto last = std::chrono::steady_clock::now();
      cnt++;
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - last).count();
      if (dt >= 2.0) {
        RCLCPP_INFO(get_logger(),
          "[ADC-RATE] V4L2: %.1f Hz  (%d frames in %.1fs)",
          cnt / dt, cnt, dt);
        cnt  = 0;
        last = now;
      }
    }

    return true;
  }

private:
  // ──────────────────────────────────────────────────────────────────────────
  // V4L2 buffer 元数据
  // ──────────────────────────────────────────────────────────────────────────
  struct V4l2Buffer {
    void   *start  = nullptr;
    size_t  length = 0;
  };

  // ──────────────────────────────────────────────────────────────────────────
  // V4L2 设备管理
  // ──────────────────────────────────────────────────────────────────────────

  void open_device()
  {
    if (v4l2_fd_ >= 0) close_device();

    v4l2_fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK);
    if (v4l2_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "无法打开 V4L2 设备 '%s': %s", device_path_.c_str(), strerror(errno));
      return;
    }

    // ── QUERYCAP: 验证设备能力 ──
    struct v4l2_capability cap;
    std::memset(&cap, 0, sizeof(cap));
    if (ioctl(v4l2_fd_, VIDIOC_QUERYCAP, &cap) < 0) {
      RCLCPP_WARN(get_logger(), "VIDIOC_QUERYCAP 失败: %s", strerror(errno));
      close_device();
      return;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
      RCLCPP_WARN(get_logger(), "设备 '%s' 不支持 video capture", device_path_.c_str());
      close_device();
      return;
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
      RCLCPP_WARN(get_logger(), "设备 '%s' 不支持 streaming I/O", device_path_.c_str());
      close_device();
      return;
    }

    RCLCPP_DEBUG(get_logger(),
      "V4L2 设备: driver='%s' card='%s' bus='%s'",
      cap.driver, cap.card, cap.bus_info);

    // ── S_FMT: 设置像素格式 ──
    if (!init_format()) { close_device(); return; }

    // ── S_CTRL: bypass_mode=0 (绕过 NVIDIA ISP, 直出原始数据) ──
    {
      struct v4l2_control ctrl;
      std::memset(&ctrl, 0, sizeof(ctrl));
      ctrl.id    = 0x009a2064;   // bypass_mode (NVIDIA 自定义 V4L2 control)
      ctrl.value = 0;
      if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) < 0)
        RCLCPP_DEBUG(get_logger(), "设置 bypass_mode 失败 (非致命): %s", strerror(errno));
    }

    // ── REQBUFS + QUERYBUF + mmap ──
    if (!init_mmap()) { close_device(); return; }

    // ── QBUF × N + STREAMON ──
    if (!start_streaming()) { close_device(); return; }

    RCLCPP_INFO(get_logger(),
      "V4L2 设备已连接: %s fd=%d (%ux%u RG12, %.1f KB/帧)",
      device_path_.c_str(), v4l2_fd_,
      width_, height_, frame_bytes_ / 1024.0);
  }

  bool init_format()
  {
    struct v4l2_format fmt;
    std::memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width_;
    fmt.fmt.pix.height      = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB12;  // RG12
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (ioctl(v4l2_fd_, VIDIOC_S_FMT, &fmt) < 0) {
      RCLCPP_WARN(get_logger(), "VIDIOC_S_FMT 失败: %s", strerror(errno));
      return false;
    }

    // 驱动可能调整分辨率: 以实际值为准
    if (fmt.fmt.pix.width != width_ || fmt.fmt.pix.height != height_) {
      RCLCPP_INFO(get_logger(),
        "驱动调整了分辨率: 请求 %ux%u → 实际 %ux%u",
        width_, height_, fmt.fmt.pix.width, fmt.fmt.pix.height);
      width_  = fmt.fmt.pix.width;
      height_ = fmt.fmt.pix.height;
    }

    // 以驱动返回的 sizeimage 为准 (可能大于 width×height×2)
    if (fmt.fmt.pix.sizeimage > 0)
      frame_bytes_ = fmt.fmt.pix.sizeimage;
    else
      frame_bytes_ = static_cast<size_t>(width_) * height_ * 2;

    RCLCPP_INFO(get_logger(),
      "V4L2 格式: %ux%u %c%c%c%c stride=%u sizeimage=%u (%.1f KB/帧)",
      fmt.fmt.pix.width, fmt.fmt.pix.height,
      static_cast<char>((fmt.fmt.pix.pixelformat >> 0)  & 0xFF),
      static_cast<char>((fmt.fmt.pix.pixelformat >> 8)  & 0xFF),
      static_cast<char>((fmt.fmt.pix.pixelformat >> 16) & 0xFF),
      static_cast<char>((fmt.fmt.pix.pixelformat >> 24) & 0xFF),
      fmt.fmt.pix.bytesperline,
      fmt.fmt.pix.sizeimage,
      frame_bytes_ / 1024.0);

    return true;
  }

  bool init_mmap()
  {
    struct v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.count  = NUM_V4L2_BUFS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(v4l2_fd_, VIDIOC_REQBUFS, &req) < 0) {
      RCLCPP_WARN(get_logger(), "VIDIOC_REQBUFS 失败: %s", strerror(errno));
      return false;
    }
    if (req.count < 2) {
      RCLCPP_WARN(get_logger(), "驱动缓冲区不足: %u", req.count);
      return false;
    }

    v4l2_bufs_.resize(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
      struct v4l2_buffer buf;
      std::memset(&buf, 0, sizeof(buf));
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index  = i;

      if (ioctl(v4l2_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
        RCLCPP_WARN(get_logger(), "VIDIOC_QUERYBUF[%u] 失败: %s", i, strerror(errno));
        return false;
      }

      v4l2_bufs_[i].length = buf.length;
      v4l2_bufs_[i].start = mmap(
          nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
          v4l2_fd_, buf.m.offset);

      if (v4l2_bufs_[i].start == MAP_FAILED) {
        RCLCPP_WARN(get_logger(), "mmap[%u] 失败: %s", i, strerror(errno));
        return false;
      }
    }
    return true;
  }

  bool start_streaming()
  {
    // 将所有 buffer 入队
    for (uint32_t i = 0; i < v4l2_bufs_.size(); ++i) {
      struct v4l2_buffer buf;
      std::memset(&buf, 0, sizeof(buf));
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index  = i;
      if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buf) < 0) {
        RCLCPP_WARN(get_logger(), "VIDIOC_QBUF[%u] 失败: %s", i, strerror(errno));
        return false;
      }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(v4l2_fd_, VIDIOC_STREAMON, &type) < 0) {
      RCLCPP_WARN(get_logger(), "VIDIOC_STREAMON 失败: %s", strerror(errno));
      return false;
    }
    streaming_ = true;
    return true;
  }

  void stop_streaming()
  {
    if (v4l2_fd_ >= 0 && streaming_) {
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ioctl(v4l2_fd_, VIDIOC_STREAMOFF, &type);
      streaming_ = false;
    }
    for (auto &b : v4l2_bufs_) {
      if (b.start && b.start != MAP_FAILED)
        munmap(b.start, b.length);
    }
    v4l2_bufs_.clear();
  }

  // 非阻塞出队: 成功返回 true, 无可用帧返回 false (data=nullptr)
  bool dequeue_buffer(uint8_t *&data, size_t &bytes_used)
  {
    struct v4l2_buffer buf;
    std::memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(v4l2_fd_, VIDIOC_DQBUF, &buf) < 0) {
      if (errno == EAGAIN) return false;        // 无可用帧 (非阻塞)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "VIDIOC_DQBUF 失败: %s", strerror(errno));
      return false;
    }

    v4l2_buf_index_ = buf.index;
    data       = static_cast<uint8_t *>(v4l2_bufs_[buf.index].start);
    bytes_used = buf.bytesused;
    return true;
  }

  // 归还 buffer 到驱动队列
  void enqueue_buffer(uint32_t index)
  {
    struct v4l2_buffer buf;
    std::memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = index;
    if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buf) < 0)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "VIDIOC_QBUF 失败: %s", strerror(errno));
  }

  void close_device()
  {
    if (v4l2_fd_ >= 0) {
      close(v4l2_fd_);
      v4l2_fd_ = -1;
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 成员变量
  // ──────────────────────────────────────────────────────────────────────────
  std::string device_path_;
  std::string frame_id_ = "radar";

  uint32_t  width_       = DEFAULT_WIDTH;
  uint32_t  height_      = DEFAULT_HEIGHT;
  size_t    frame_bytes_ = 0;

  int       v4l2_fd_          = -1;
  bool      streaming_        = false;
  uint32_t  v4l2_buf_index_   = 0;
  std::vector<V4l2Buffer> v4l2_bufs_;

  // ── TF 广播器 ──
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bc_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdcRxNode>());
  rclcpp::shutdown();
  return 0;
}
