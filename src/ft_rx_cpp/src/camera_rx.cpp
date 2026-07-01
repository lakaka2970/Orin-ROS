// ============================================================================
// camera_rx.cpp — 相机数据采集节点 (V4L2 真实摄像头, 30 Hz)
// ============================================================================
// 通过 OpenCV V4L2 后端从 USB UVC 摄像头采集中集图像帧.
// 发布 1920×1080 MJPEG→BGR8 图像到 /camera/image_raw.
//
// 摄像头: Rmoncam A2 1080P (USB UVC, /dev/camera_capture)
// 格式:   MJPEG → OpenCV 自动解码为 BGR8
// 帧率:   30 fps (最高)
//
// 断线处理: 定时尝试重连, 重连期间发布空图像以维持节点拓扑.
// ============================================================================

#include <atomic>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ft_rx_cpp/rx_node_base.hpp"

using Image = sensor_msgs::msg::Image;

namespace {
  constexpr double   FPS            = 30.0;
  constexpr int      DEFAULT_WIDTH  = 1920;
  constexpr int      DEFAULT_HEIGHT = 1080;
  constexpr int      RECONNECT_INTERVAL = 90;  // 每 N 帧尝试重连 (90帧≈3s)
}

class CameraRxNode : public ft_rx::RxNodeBase<Image, CameraRxNode>
{
public:
  CameraRxNode()
    : RxNodeBase("camera_rx", "/camera/image_raw", 10, false)  // reliable QoS for image
  {
    // ── 参数声明 ──
    declare_parameter("fps", FPS);
    declare_parameter("image_width",  DEFAULT_WIDTH);
    declare_parameter("image_height", DEFAULT_HEIGHT);
    declare_parameter("device_path", "/dev/camera_capture");
    declare_parameter("pixel_format", "MJPG");
    declare_parameter("reconnect_interval_frames", RECONNECT_INTERVAL);
    declare_parameter("fixed_frame", "camera");
    // 预留参数 (forward-compatible)
    declare_parameter("line", 0);
    declare_parameter("nof_line", 0);

    fps_        = get_parameter("fps").as_double();
    width_      = get_parameter("image_width").as_int();
    height_     = get_parameter("image_height").as_int();
    device_path_ = get_parameter("device_path").as_string();
    pixel_fmt_   = get_parameter("pixel_format").as_string();
    reconnect_interval_ = get_parameter("reconnect_interval_frames").as_int();
    frame_id_    = get_parameter("fixed_frame").as_string();

    // ── 静态 TF radar → camera ──
    tf_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp    = builtin_interfaces::msg::Time();  // zero = permanent
      tf.header.frame_id = "radar";
      tf.child_frame_id  = frame_id_;
      tf.transform.translation.z = 1.2;
      tf.transform.rotation.w    = 1.0;
      tf_bc_->sendTransform(tf);
    }

    // ── 空图像降级 (相机未连接时使用) ──
    empty_img_ = cv::Mat::zeros(height_, width_, CV_8UC3);

    // ── 打开 V4L2 摄像头 ──
    camera_opened_ = open_camera();

    if (camera_opened_) {
      RCLCPP_INFO(get_logger(),
        "Camera Rx [V4L2]: %.0f Hz, %dx%d %s, 设备=%s",
        fps_, width_, height_, pixel_fmt_.c_str(), device_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
        "Camera Rx: V4L2 设备打开失败 '%s' — 将发布空图像占位",
        device_path_.c_str());
    }

    RCLCPP_INFO(get_logger(),
      "Camera Rx 启动: %.0f Hz, %dx%d, 设备=%s, 格式=%s",
      fps_, width_, height_, device_path_.c_str(), pixel_fmt_.c_str());

    start_polling_loop(fps_);
  }

  ~CameraRxNode() override
  {
    stop_polling_ = true;     // signal polling thread to exit
    close_camera();           // cap.release() unblocks pending cap.read()
  }

  std::string frame_id() const { return frame_id_; }

  // ------------------------------------------------------------------
  // fill_message — 从 V4L2 摄像头读取一帧
  //   返回 false: 子类已在内部完成 publish (用于设置 cv_bridge header)
  // ------------------------------------------------------------------
  bool fill_message(Image & /*msg*/)
  {
    cv::Mat frame;

    if (camera_opened_) {
      cap_ >> frame;   // OpenCV 自动 MJPEG→BGR 解码

      if (!frame.empty()) {
        last_valid_img_ = frame.clone();
      } else {
        // 读取失败 — 可能断线
        frame = last_valid_img_.empty() ? empty_img_ : last_valid_img_;
        if (++frame_since_fail_ >= reconnect_interval_) {
          frame_since_fail_ = 0;
          RCLCPP_WARN(get_logger(),
            "V4L2 读取失败 (已丢%d帧)，尝试重连...", reconnect_interval_);
          camera_opened_ = open_camera();
          if (camera_opened_)
            RCLCPP_INFO(get_logger(), "V4L2 重连成功");
        }
      }
    } else {
      // 初始未连接 — 定期重试
      frame = empty_img_;
      if (++frame_since_fail_ >= reconnect_interval_) {
        frame_since_fail_ = 0;
        camera_opened_ = open_camera();
        if (camera_opened_)
          RCLCPP_INFO(get_logger(), "V4L2 连接成功 (首次)");
      }
    }

    prof_.checkpoint("camera_read");

    // 发布 via cv_bridge
    auto m = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    m->header.stamp    = this->now();
    m->header.frame_id = frame_id_;
    pub_->publish(*m);
    prof_.checkpoint("publish");

    return false;  // 子类已自行发布
  }

private:
  // ──────────────────────────────────────────────────────────────────────────
  // V4L2 摄像头管理
  // ──────────────────────────────────────────────────────────────────────────

  bool open_camera()
  {
    close_camera();

    cap_.open(device_path_, cv::CAP_V4L2);
    if (!cap_.isOpened())
      return false;

    // 设置像素格式 (必须在分辨率之前)
    auto fourcc = make_fourcc(pixel_fmt_);
    cap_.set(cv::CAP_PROP_FOURCC, static_cast<double>(fourcc));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  static_cast<double>(width_));
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
    cap_.set(cv::CAP_PROP_FPS, fps_);

    // 验证实际参数
    int actual_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);

    RCLCPP_DEBUG(get_logger(),
      "V4L2 format: %dx%d @ %.0f fps, fourcc=%.4s",
      actual_w, actual_h, actual_fps,
      reinterpret_cast<const char*>(&fourcc));

    frame_since_fail_ = 0;
    return true;
  }

  void close_camera()
  {
    if (cap_.isOpened())
      cap_.release();
  }

  static int make_fourcc(const std::string &fmt)
  {
    if (fmt.size() >= 4)
      return cv::VideoWriter::fourcc(fmt[0], fmt[1], fmt[2], fmt[3]);
    return cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 成员变量
  // ──────────────────────────────────────────────────────────────────────────
  cv::VideoCapture cap_;
  cv::Mat   empty_img_;
  cv::Mat   last_valid_img_;

  std::string device_path_;
  std::string pixel_fmt_;
  std::string frame_id_ = "camera";

  int    width_              = DEFAULT_WIDTH;
  int    height_             = DEFAULT_HEIGHT;
  int    reconnect_interval_ = RECONNECT_INTERVAL;
  int    frame_since_fail_   = 0;

  std::atomic<bool> camera_opened_{false};

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bc_;
};

// ============================================================================
// main
// ============================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraRxNode>());
  rclcpp::shutdown();
  return 0;
}
