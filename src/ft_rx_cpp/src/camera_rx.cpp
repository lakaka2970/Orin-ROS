// ============================================================================
// camera_rx.cpp — 相机数据采集节点 (空数据, 30 Hz)
// ============================================================================
// 发布极小空图像以维持节点拓扑, 不生成测试图案, 不占用硬盘空间.
// 4×4×3 = 48 bytes/frame, 30 Hz ≈ 1.4 KB/s.
// ============================================================================

#include <string>

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ft_rx_cpp/rx_node_base.hpp"

using Image = sensor_msgs::msg::Image;

namespace { constexpr double FPS = 30.0; }

class CameraRxNode : public ft_rx::RxNodeBase<Image, CameraRxNode>
{
public:
  CameraRxNode()
    : RxNodeBase("camera_rx", "/camera/image_raw", 10, true)
  {
    declare_parameter("fps", FPS);
    declare_parameter("image_width", 4);
    declare_parameter("image_height", 4);
    declare_parameter("fixed_frame", "camera");

    fps_    = get_parameter("fps").as_double();
    width_  = get_parameter("image_width").as_int();
    height_ = get_parameter("image_height").as_int();
    frame_id_ = get_parameter("fixed_frame").as_string();

    // static TF radar → camera (zero stamp = valid for all time in TF2)
    tf_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = builtin_interfaces::msg::Time();
    tf.header.frame_id = "radar";
    tf.child_frame_id  = frame_id_;
    tf.transform.translation.z = 1.2;
    tf.transform.rotation.w    = 1.0;
    tf_bc_->sendTransform(tf);

    // 极小空图像, 不生成测试图案
    img_ = cv::Mat::zeros(height_, width_, CV_8UC3);

    RCLCPP_INFO(get_logger(), "Camera Rx: %.0f Hz, %dx%d (空数据模式)", fps_, width_, height_);
    init_timer(fps_);
  }

  std::string frame_id() const { return frame_id_; }

  bool fill_message(Image & /*msg*/)
  {
    local_count_++;

    auto m = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_).toImageMsg();
    m->header.stamp    = this->now();
    m->header.frame_id = frame_id_;
    pub_->publish(*m);
    return false;  // subclass already published
  }

private:
  cv::Mat      img_;
  int          width_   = 4;
  int          height_  = 4;
  int          local_count_ = 0;
  std::string  frame_id_ = "camera";
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bc_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraRxNode>());
  rclcpp::shutdown();
}
