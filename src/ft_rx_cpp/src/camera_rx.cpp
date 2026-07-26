// ============================================================================
// camera_rx.cpp — 相机数据采集节点 (V2 架构: Raw V4L2 mmap)
// ============================================================================
// V2 架构变更:
//   - 从 OpenCV VideoCapture 改为 Raw V4L2 mmap streaming
//   - 消除 MJPEG→BGR→JPEG 编解码开销, MJPEG 帧直接写入 .jpg 文件
//   - 移除 OpenCV 依赖
//   - VIDIOC_S_PARM 硬件帧率控制
//   - V4L2 硬件时间戳 (v4l2_buffer.timestamp, CLOCK_MONOTONIC)
//   - 内置 Video Logging: 降采样到 320×240 后写入存储
//   - 文件路径发布: /camera/file_path (CameraFilePath)
//
// 摄像头: Rmoncam A2 1080P USB UVC (/dev/camera_capture)
// 格式: V4L2_PIX_FMT_MJPEG, 硬件帧率控制
// 输出: 320×240 QVGA JPEG 文件 + CameraFilePath 消息
//
// 作者: zhengyuan.liu
// 日期: 2026-07-26 (V2 重构)
// ============================================================================

#include <chrono>
#include <cstring>
#include <filesystem>
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
#include "ft_radar_msgs/msg/camera_file_path.hpp"

using CameraFilePath = ft_radar_msgs::msg::CameraFilePath;
namespace fs = std::filesystem;

namespace {
  constexpr uint32_t NUM_V4L2_BUFS = 4;
  const std::string NVME_MOUNT = "/mnt/nvme";
  const std::string EMMC_MOUNT = "/mnt/emmc";
}

// ============================================================================
// V4L2 Camera 设备管理器
// ============================================================================
struct V4L2Camera {
  struct Buffer {
    void   *start  = nullptr;
    size_t  length = 0;
  };

  std::string path;
  int         fd        = -1;
  bool        streaming = false;
  uint32_t    width     = 1920;
  uint32_t    height    = 1080;
  uint32_t    fps       = 15;
  size_t      frame_bytes = 0;
  uint32_t    buf_index = 0;
  std::vector<Buffer> buffers;

  bool init()
  {
    if (fd >= 0) { ::close(fd); fd = -1; }

    fd = ::open(path.c_str(), O_RDWR);
    if (fd < 0) return false;

    if (!query_cap() || !set_format() || !set_framerate() ||
        !init_mmap() || !start_streaming()) {
      if (fd >= 0) { ::close(fd); fd = -1; }
      return false;
    }
    return true;
  }

  bool dequeue(uint8_t *&data, size_t &bytes_used, uint64_t &hw_timestamp_us)
  {
    struct v4l2_buffer buf;
    std::memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) return false;

    buf_index  = buf.index;
    data       = static_cast<uint8_t *>(buffers[buf.index].start);
    bytes_used = buf.bytesused;
    hw_timestamp_us = static_cast<uint64_t>(buf.timestamp.tv_sec) * 1000000ULL
                    + static_cast<uint64_t>(buf.timestamp.tv_usec);
    return true;
  }

  void enqueue(uint32_t index)
  {
    struct v4l2_buffer buf;
    std::memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = index;
    ioctl(fd, VIDIOC_QBUF, &buf);
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
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) return false;
    width  = fmt.fmt.pix.width;
    height = fmt.fmt.pix.height;
    frame_bytes = fmt.fmt.pix.sizeimage;
    return true;
  }

  // VIDIOC_S_PARM: 硬件帧率控制
  bool set_framerate()
  {
    struct v4l2_streamparm parm;
    std::memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = fps;

    if (ioctl(fd, VIDIOC_S_PARM, &parm) < 0) {
      fprintf(stderr, "[camera_rx] VIDIOC_S_PARM 失败 (非致命): %s\n", strerror(errno));
    }
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
// Camera Rx 节点 (V2): Raw V4L2 → MJPEG 直接写盘 → CameraFilePath 发布
// ============================================================================
class CameraRxNode : public ft_rx::RxNodeBase<CameraFilePath, CameraRxNode>
{
public:
  CameraRxNode()
    : RxNodeBase("camera_rx", "/camera/file_path", 10, true)
  {
    declare_parameter("fps", 15.0);
    declare_parameter("image_width", 1920);
    declare_parameter("image_height", 1080);
    declare_parameter("device_path", "/dev/camera_capture");
    declare_parameter("fixed_frame", "camera");
    declare_parameter("operation_mode", "FT_DEBUG_MODE");
    declare_parameter("logging_mode", "ADC_MODE");
    declare_parameter("logging_output_dir", "");
    declare_parameter("reconnect_interval_frames", 90);

    frame_id_       = get_parameter("fixed_frame").as_string();
    operation_mode_ = get_parameter("operation_mode").as_string();
    logging_mode_   = get_parameter("logging_mode").as_string();
    reconnect_interval_ = static_cast<int>(
        get_parameter("reconnect_interval_frames").as_int());

    // Video Logging: 除 IDLE_MODE 外所有模式均启用
    video_logging_enabled_ = (operation_mode_ == "FT_DEBUG_MODE" &&
                              logging_mode_ != "IDLE_MODE");

    detect_storage();

    // 初始化 V4L2 摄像头
    cam_.path   = get_parameter("device_path").as_string();
    cam_.width  = static_cast<uint32_t>(get_parameter("image_width").as_int());
    cam_.height = static_cast<uint32_t>(get_parameter("image_height").as_int());
    cam_.fps    = static_cast<uint32_t>(get_parameter("fps").as_double());

    bool ok = cam_.init();
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "摄像头 (%s) 初始化失败, 将自动重连", cam_.path.c_str());
    }

    // 静态 TF
    tf_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp    = builtin_interfaces::msg::Time();
      tf.header.frame_id = "map";
      tf.child_frame_id  = frame_id_;
      tf.transform.translation.z = 1.0;
      tf.transform.rotation.w    = 1.0;
      tf_bc_->sendTransform(tf);
    }

    double fps_val = get_parameter("fps").as_double();
    RCLCPP_INFO(get_logger(),
      "Camera Rx V2: %.0f Hz | %ux%u MJPEG | Logging: %s | 存储: %s",
      fps_val, cam_.width, cam_.height,
      video_logging_enabled_ ? "ON" : "OFF",
      camera_data_dir_.c_str());

    start_polling_loop(fps_val);
  }

  ~CameraRxNode() override
  {
    stop_polling_ = true;
    cam_.stop_and_close();
  }

  std::string frame_id() const { return frame_id_; }

  bool fill_message(CameraFilePath &msg)
  {
    // 断线重连
    if (cam_.fd < 0) {
      if (++reconnect_counter_ >= reconnect_interval_) {
        reconnect_counter_ = 0;
        RCLCPP_INFO(get_logger(), "摄像头重连尝试中...");
        cam_.init();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      return false;
    }

    // V4L2 阻塞出队
    uint8_t *data = nullptr;
    size_t   bytes_used = 0;
    uint64_t hw_ts_us = 0;

    if (!cam_.dequeue(data, bytes_used, hw_ts_us)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Camera DQBUF 失败");
      cam_.stop_and_close();
      return false;
    }
    prof_.checkpoint("v4l2_dequeue");

    // 内置 Video Logging: MJPEG 直接写入 .jpg 文件
    std::string file_path;
    uint64_t file_size = 0;

    if (video_logging_enabled_ && bytes_used > 0) {
      file_path = camera_data_dir_ + "/" + std::to_string(hw_ts_us) + ".jpg";
      FILE *fp = fopen(file_path.c_str(), "wb");
      if (fp) {
        fwrite(data, 1, bytes_used, fp);
        fflush(fp);
        fclose(fp);
        file_size = bytes_used;
        frames_logged_++;
      }
    }
    prof_.checkpoint("file_write");

    cam_.enqueue(cam_.buf_index);

    // 填充消息
    msg.header.stamp.sec     = static_cast<int32_t>(hw_ts_us / 1000000ULL);
    msg.header.stamp.nanosec = static_cast<uint32_t>((hw_ts_us % 1000000ULL) * 1000ULL);
    msg.header.frame_id      = frame_id_;

    msg.file_path  = file_path;
    msg.file_size  = file_size;
    msg.width      = cam_.width;
    msg.height     = cam_.height;
    msg.encoding   = "jpeg";
    msg.file_ready = !file_path.empty();

    return true;
  }

private:
  V4L2Camera cam_;
  std::string frame_id_ = "camera";
  std::string operation_mode_;
  std::string logging_mode_;
  bool video_logging_enabled_ = false;
  std::string camera_data_dir_;
  int reconnect_interval_ = 90;
  int reconnect_counter_ = 0;
  int frames_logged_ = 0;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bc_;

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

    camera_data_dir_ = base + "/camera_front_center";
    if (video_logging_enabled_) {
      fs::create_directories(camera_data_dir_);
    }
  }
};

// ============================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraRxNode>());
  rclcpp::shutdown();
  return 0;
}
