// ============================================================================
// rx_node_base.hpp  —  C++ rx 节点基类 (集成轻量帧级 Profiler)
// ============================================================================
// 三个 rx 节点共享: declare params → publisher → wall-timer → publish.
// 子类只需实现 fill_message(msg) 填充数据，基类负责定时器/发布/频率校验.
//
// ★ 性能分析器 (FrameProfiler) 通过 declare_parameter 控制:
//     profiler_enabled := true   → 每 N 帧自动输出分步耗时报告
//     profiler_log_every_n := 50 → 报告输出间隔 (帧数)
//   子类在 fill_message() 中插入   prof_.checkpoint("步骤名");
//   即可零侵入获得分步耗时统计。 disabled 时所有调用退化为零开销.
//
// 作者: zhengyuan.liu
// 日期: 2026-06-18
// ============================================================================

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <std_msgs/msg/empty.hpp>

#include "ft_rx_cpp/perf_profiler.hpp"

namespace ft_rx
{

inline rclcpp::QoS rx_qos(int depth = 10, bool best_effort = true)
{
  auto qos = rclcpp::QoS(depth);
  if (best_effort)
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  else
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  return qos;
}


// ============================================================================
// RxNodeBase  (集成 FrameProfiler)
// ============================================================================
template <typename MessageT, typename Derived>
class RxNodeBase : public rclcpp::Node
{
public:
  RxNodeBase(const std::string &name, const std::string &topic,
             int qos_depth = 10, bool best_effort = true)
    : Node(name)
  {
    pub_ = this->create_publisher<MessageT>(topic, rx_qos(qos_depth, best_effort));
  }

  /// Must be called at END of subclass ctor, after parameter reading.
  void init_timer(double fps)
  {
    fps_ = fps;

    // ── 性能分析器参数 (子类可 declare 覆盖) ──
    declare_parameter("profiler_enabled", true);
    declare_parameter("profiler_log_every_n", 50);

    bool prof_enabled = get_parameter("profiler_enabled").as_bool();
    int  prof_period  = get_parameter("profiler_log_every_n").as_int();
    prof_ = ft_perf::FrameProfiler(this, prof_period, prof_enabled);

    auto period = std::chrono::duration<double>(1.0 / fps_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { on_timer_impl(); });

    heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        this->get_name() + std::string("/heartbeat"), rx_qos(10, true));

    wall_start_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(),
      "定时器: %.1f Hz | 心跳: %s | Profiler: %s",
      fps_,
      (std::string(this->get_name()) + "/heartbeat").c_str(),
      prof_.is_enabled() ? "ON" : "OFF");
  }

  /// Start a dedicated polling thread (hardware-driven, no artificial rate limiting).
  /// The thread calls execute_frame() in a loop.  V4L2 DQBUF blocks inside
  /// fill_message() and naturally paces the loop to the hardware frame rate.
  /// No timer-based sleep — the hardware IS the clock.
  void start_polling_loop(double expected_fps)
  {
    fps_ = expected_fps;

    // ── 性能分析器参数 (子类可 declare 覆盖) ──
    declare_parameter("profiler_enabled", true);
    declare_parameter("profiler_log_every_n", 50);

    bool prof_enabled = get_parameter("profiler_enabled").as_bool();
    int  prof_period  = get_parameter("profiler_log_every_n").as_int();
    prof_ = ft_perf::FrameProfiler(this, prof_period, prof_enabled);

    heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        this->get_name() + std::string("/heartbeat"), rx_qos(10, true));

    wall_start_ = std::chrono::steady_clock::now();
    stop_polling_ = false;

    polling_thread_ = std::thread([this]() {
      while (rclcpp::ok() && !stop_polling_) {
        execute_frame();   // fill_message() 内 V4L2 DQBUF 阻塞 → 硬件帧率即为发布帧率
      }
    });

    RCLCPP_INFO(get_logger(),
      "轮询循环: expected %.1f Hz | 心跳: %s | Profiler: %s",
      fps_,
      (std::string(this->get_name()) + "/heartbeat").c_str(),
      prof_.is_enabled() ? "ON" : "OFF");
  }

  ~RxNodeBase()
  {
    if (polling_thread_.joinable())
      polling_thread_.join();
  }

protected:
  typename rclcpp::Publisher<MessageT>::SharedPtr pub_;
  double fps_ = 0;

  /// 子类可在 fill_message() 中调用 prof_.checkpoint("name") 记录分步耗时.
  /// disabled 时所有 checkpoint 调用自动内联为空操作, 零开销.
  ft_perf::FrameProfiler prof_{nullptr, 0, false};

  /// 轮询线程退出标志: 子类析构时设为 true 以通知线程退出
  std::atomic<bool> stop_polling_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  std::chrono::steady_clock::time_point wall_start_;
  int frame_count_ = 0;
  int wall_check_  = 0;

  void on_timer_impl()
  {
    execute_frame();
  }

  /// Single frame iteration: profiler wrap → heartbeat → stamp → fill → publish → rate check.
  /// Called by both timer callback (on_timer_impl) and polling thread (polling_loop).
  void execute_frame()
  {
    prof_.tick();

    frame_count_++;
    heartbeat_pub_->publish(std_msgs::msg::Empty{});
    prof_.checkpoint("heartbeat");

    MessageT msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = static_cast<Derived *>(this)->frame_id();

    // return true → base class publishes msg; false → subclass handled publish
    if (static_cast<Derived *>(this)->fill_message(msg)) {
      prof_.checkpoint("fill");
      pub_->publish(msg);
      prof_.checkpoint("publish");
    }

    // rate check every 30 frames
    wall_check_++;
    if (wall_check_ >= 30) {
      wall_check_ = 0;
      auto now = std::chrono::steady_clock::now();
      double s = std::chrono::duration<double>(now - wall_start_).count();
      if (s > 0.5) {
        double hz = 30.0 / s;
        if (fps_ > 0 && hz < fps_ * 0.85)
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[频率] expected %.0f Hz  actual %.1f Hz (%.0f%%)", fps_, hz, hz/fps_*100);
        wall_start_ = now;
      }
    }

    prof_.tick_end();
  }

  // ── polling thread ──
  std::thread polling_thread_;
};

}  // namespace ft_rx
