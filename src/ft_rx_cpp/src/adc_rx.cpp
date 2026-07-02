// ============================================================================
// adc_rx.cpp — ADC 真实数据采集节点 (双设备 V4L2 mmap streaming)
// ============================================================================
// 同时从 ctrx0 (/dev/radar_ctrx0) 和 ctrx1 (/dev/radar_ctrx1) 两个 V4L2 设备
// 捕获原始 ADC 数据, 逐帧拼接为完整 16T16R (8 RX) 32 MiB 消息。
//
// 修复历史:
//   2026-07-02: 增加 ctrx1 设备支持, ctrx0+ctrx1 拼接 → 32 MiB/帧, num_chirps_per_row=8
//               (修复: 单设备 16 MiB 导致 RSP reshape 失败, 点云/rdCell/rxNci 无输出)
//
// 数据格式: RG12 (12-bit padded to 16-bit), 8192×1024 per device
//   单设备:  8192 × 1024 × 2B = 16 MiB (4 RX, ctrx0 或 ctrx1)
//   双设备拼接: 16 MiB × 2 = 32 MiB (8 RX, ctrx0+ctrx1)
//
// 用法: adc_rx_cpp (默认), 由 launch 文件通过 adc_source:=real 选择.
//
// V4L2 流程 (per device):
//   open → QUERYCAP → S_FMT → S_CTRL(bypass_mode) → REQBUFS → QUERYBUF×N →
//   mmap×N → QBUF×N → STREAMON → (DQBUF→copy→QBUF)×… → STREAMOFF → munmap → close
//
// 设计目标: 15 Hz, FastDDS SHM 共享内存传输.
// ============================================================================

#include <chrono>
#include <cstring>
#include <string>
#include <thread>
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
  constexpr uint32_t DEFAULT_WIDTH  = 8192;   // 4 RX × 2048 samples (pixel-interleaved)
  constexpr uint32_t DEFAULT_HEIGHT = 1024;   // 512 chirps × 2 groups
  constexpr uint32_t NUM_V4L2_BUFS  = 4;
  // bypass_mode V4L2 control ID (NVIDIA 自定义)
  constexpr uint32_t BYPASS_MODE_CID = 0x009a2064;
}

// ============================================================================
// V4L2 单设备管理器 — 封装一个 CTRX 设备的完整生命周期
// ============================================================================
struct V4L2Device {
  struct Buffer {
    void   *start  = nullptr;
    size_t  length = 0;
  };

  std::string path;
  std::string label;       // 日志标签, 如 "ctrx0"
  int         fd          = -1;
  bool        streaming   = false;
  uint32_t    width       = DEFAULT_WIDTH;
  uint32_t    height      = DEFAULT_HEIGHT;
  size_t      frame_bytes = 0;
  uint32_t    buf_index   = 0;
  std::vector<Buffer> buffers;

  // ── 打开设备 → 配置格式 → 设置 bypass → mmap → streamon ──
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

  // ── 阻塞出队: 等待硬件帧就绪 ──
  bool dequeue(uint8_t *&data, size_t &bytes_used)
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
    return true;
  }

  // ── 归还 buffer ──
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

  // ── STREAMOFF + munmap + close ──
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
    if (fd >= 0) {
      ::close(fd);
      fd = -1;
    }
  }

private:
  void close_noexcept()
  {
    if (fd >= 0) { ::close(fd); fd = -1; }
  }

  bool query_cap()
  {
    struct v4l2_capability cap;
    std::memset(&cap, 0, sizeof(cap));
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
      fprintf(stderr, "[adc_rx] %s: VIDIOC_QUERYCAP 失败: %s\n",
              label.c_str(), strerror(errno));
      return false;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(cap.capabilities & V4L2_CAP_STREAMING)) {
      fprintf(stderr, "[adc_rx] %s: 不支持 video capture/streaming\n", label.c_str());
      return false;
    }
    return true;
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

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
      fprintf(stderr, "[adc_rx] %s: VIDIOC_S_FMT 失败: %s\n",
              label.c_str(), strerror(errno));
      return false;
    }

    if (fmt.fmt.pix.width != width || fmt.fmt.pix.height != height) {
      fprintf(stderr, "[adc_rx] %s: 驱动调整分辨率 %ux%u → %ux%u\n",
              label.c_str(), width, height, fmt.fmt.pix.width, fmt.fmt.pix.height);
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
    // 非致命: 某些驱动不支持 bypass_mode
    ioctl(fd, VIDIOC_S_CTRL, &ctrl);
    return true;
  }

  bool init_mmap()
  {
    struct v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.count  = NUM_V4L2_BUFS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
      fprintf(stderr, "[adc_rx] %s: VIDIOC_REQBUFS 失败: %s\n",
              label.c_str(), strerror(errno));
      return false;
    }
    if (req.count < 2) {
      fprintf(stderr, "[adc_rx] %s: 驱动缓冲区不足: %u\n", label.c_str(), req.count);
      return false;
    }

    buffers.resize(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
      struct v4l2_buffer buf;
      std::memset(&buf, 0, sizeof(buf));
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index  = i;

      if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
        fprintf(stderr, "[adc_rx] %s: VIDIOC_QUERYBUF[%u] 失败: %s\n",
                label.c_str(), i, strerror(errno));
        return false;
      }

      buffers[i].length = buf.length;
      buffers[i].start = mmap(
          nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
          fd, buf.m.offset);

      if (buffers[i].start == MAP_FAILED) {
        fprintf(stderr, "[adc_rx] %s: mmap[%u] 失败: %s\n",
                label.c_str(), i, strerror(errno));
        return false;
      }
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
      if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        fprintf(stderr, "[adc_rx] %s: VIDIOC_QBUF[%u] 失败: %s\n",
                label.c_str(), i, strerror(errno));
        return false;
      }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
      fprintf(stderr, "[adc_rx] %s: VIDIOC_STREAMON 失败: %s\n",
              label.c_str(), strerror(errno));
      return false;
    }
    streaming = true;
    return true;
  }
};

// ============================================================================
// ADC Rx 节点: 双设备 V4L2 → 32 MiB AdcRawData
// ============================================================================
class AdcRxNode : public ft_rx::RxNodeBase<AdcRawData, AdcRxNode>
{
public:
  AdcRxNode()
    : RxNodeBase("adc_rx", "/adc/raw_data", 10, true)
  {
    declare_parameter("fps", FPS);
    declare_parameter("capture_width", static_cast<int>(DEFAULT_WIDTH));
    declare_parameter("capture_height", static_cast<int>(DEFAULT_HEIGHT));
    declare_parameter("device_path_ctrx0", "/dev/radar_ctrx0");
    declare_parameter("device_path_ctrx1", "/dev/radar_ctrx1");
    declare_parameter("fixed_frame", "radar");

    uint32_t  w = static_cast<uint32_t>(get_parameter("capture_width").as_int());
    uint32_t  h = static_cast<uint32_t>(get_parameter("capture_height").as_int());
    frame_id_   = get_parameter("fixed_frame").as_string();

    // ── 初始化两个 V4L2 设备 ──
    dev0_.path  = get_parameter("device_path_ctrx0").as_string();
    dev0_.label = "ctrx0";
    dev0_.width = w;  dev0_.height = h;

    dev1_.path  = get_parameter("device_path_ctrx1").as_string();
    dev1_.label = "ctrx1";
    dev1_.width = w;  dev1_.height = h;

    bool ok0 = dev0_.init();
    bool ok1 = dev1_.init();

    // ── 设备初始化状态日志 (ROS 级别, 替代 fprintf) ──
    if (!ok0) RCLCPP_ERROR(get_logger(),
      "ctrx0 (%s) 初始化失败, 将每 %d 帧自动重连", dev0_.path.c_str(), RECONNECT_INTERVAL);
    if (!ok1) RCLCPP_ERROR(get_logger(),
      "ctrx1 (%s) 初始化失败, 将每 %d 帧自动重连", dev1_.path.c_str(), RECONNECT_INTERVAL);

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

    // 合并后的帧大小: ctrx0 + ctrx1
    merged_frame_bytes_ = dev0_.frame_bytes + dev1_.frame_bytes;

    // 读取 fps 参数 (YAML/launch 可覆盖默认值 30Hz)
    double fps_val = get_parameter("fps").as_double();

    RCLCPP_INFO(get_logger(),
      "ADC Rx [V4L2 dual]: %.0f Hz, ctrx0=%ux%u + ctrx1=%ux%u → %.1f MB/帧, "
      "设备: %s (%s), %s (%s)",
      fps_val,
      dev0_.width, dev0_.height, dev1_.width, dev1_.height,
      merged_frame_bytes_ / 1048576.0,
      dev0_.path.c_str(), ok0 ? "已连接" : "未连接",
      dev1_.path.c_str(), ok1 ? "已连接" : "未连接");

    start_polling_loop(fps_val);
  }

  ~AdcRxNode() override
  {
    stop_polling_ = true;
    dev0_.stop_and_close();
    dev1_.stop_and_close();
  }

  std::string frame_id() const { return frame_id_; }

  bool fill_message(AdcRawData &msg)
  {
    // ── 设备断线重连 ──
    if (dev0_.fd < 0 || dev1_.fd < 0) {
      if (++reconnect_counter_ >= RECONNECT_INTERVAL) {
        reconnect_counter_ = 0;
        if (dev0_.fd < 0) {
          RCLCPP_INFO(get_logger(), "ctrx0 重连尝试中...");
          dev0_.init();
        }
        if (dev1_.fd < 0) {
          RCLCPP_INFO(get_logger(), "ctrx1 重连尝试中...");
          dev1_.init();
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      return false;
    }

    // ── 双设备阻塞出队 ──
    uint8_t *buf0 = nullptr, *buf1 = nullptr;
    size_t   bytes0 = 0, bytes1 = 0;

    if (!dev0_.dequeue(buf0, bytes0)) {
      // ctrx0 失败 → 清理并等待重连
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "ctrx0: VIDIOC_DQBUF 失败, 设备已断开, 等待重连...");
      dev0_.stop_and_close();
      return false;
    }
    if (!dev1_.dequeue(buf1, bytes1)) {
      // ctrx1 失败 → 归还 ctrx0 buffer, 清理 ctrx1
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "ctrx1: VIDIOC_DQBUF 失败, 设备已断开, 等待重连...");
      dev0_.enqueue(dev0_.buf_index);
      dev1_.stop_and_close();
      return false;
    }

    // ── 拼接: ctrx0_data || ctrx1_data → 完整 32 MiB ──
    msg.data.clear();
    msg.data.reserve(bytes0 + bytes1);
    msg.data.insert(msg.data.end(), buf0, buf0 + bytes0);
    msg.data.insert(msg.data.end(), buf1, buf1 + bytes1);

    dev0_.enqueue(dev0_.buf_index);
    dev1_.enqueue(dev1_.buf_index);
    prof_.checkpoint("v4l2_dequeue");

    // AdcRawData 维度: ctrx0 (4 RX) + ctrx1 (4 RX) = 8 RX 合并
    msg.num_rows              = dev0_.height;              // 1024 chirps
    msg.num_chirps_per_row    = 8;                         // ctrx0:4 + ctrx1:4 = 8 RX
    msg.num_samples_per_chirp = dev0_.width / 4;           // 2048 samples/RX/chirp

    return true;
  }

private:
  V4L2Device dev0_;    // ctrx0
  V4L2Device dev1_;    // ctrx1

  std::string frame_id_ = "radar";
  size_t      merged_frame_bytes_ = 0;

  // ── 重连机制 ──
  static constexpr int RECONNECT_INTERVAL = 90;  // 每 N 帧尝试重连 (~3s @ 30fps)
  int reconnect_counter_ = 0;

  // ── TF 广播器 ──
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bc_;
};

// ============================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdcRxNode>());
  rclcpp::shutdown();
  return 0;
}
