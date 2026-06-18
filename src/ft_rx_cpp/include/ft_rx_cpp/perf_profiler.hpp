// ============================================================================
// perf_profiler.hpp — 轻量帧级性能分析器 (header-only, C++17)
// ============================================================================
// 特性:
//   ✓ 零外部依赖，仅使用 C++ 标准库 + rclcpp
//   ✓ tick() / checkpoint(name) / tick_end() API
//   ✓ 每 N 帧自动输出格式化报告
//   ✓ 可选 JSON 报告落盘
//   ✓ enabled=false 时退化为零开销 (inline 空函数)
//   ✓ 自动捕获未计量时间段 (_gap_)
//
// 集成方式 (以 adc_rx 为例):
//   1. #include "ft_rx_cpp/perf_profiler.hpp"
//   2. 类内添加成员: ft_perf::FrameProfiler prof_;
//   3. on_timer_impl() 中: prof_.tick(); ... prof_.checkpoint("step"); ... prof_.tick_end();
//   4. 析构时调用 prof_.finalize()
//
// 作者: zhengyuan.liu
// 日期: 2026-06-18
// ============================================================================

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ft_perf {

// ============================================================================
// 工具: 标准偏差
// ============================================================================
inline double stddev(const std::vector<double> &v) {
  if (v.size() < 2) return 0.0;
  double mean = 0.0;
  for (auto x : v) mean += x;
  mean /= static_cast<double>(v.size());
  double var = 0.0;
  for (auto x : v) var += (x - mean) * (x - mean);
  return std::sqrt(var / static_cast<double>(v.size() - 1));
}

// ============================================================================
// FrameProfiler
// ============================================================================
class FrameProfiler {
public:
  /// @param node         ROS2 节点指针 (用于日志)
  /// @param log_every_n  每隔多少帧输出性能报告 (0 = 不自动输出)
  /// @param enabled      是否启用计时 (false → 零开销)
  FrameProfiler(rclcpp::Node *node, int log_every_n = 50, bool enabled = true)
    : node_(node), enabled_(enabled), log_every_n_(log_every_n)
  {
    // NOTE: rclcpp::Logger::set_level() not available in Foxy;
    // use RCUTILS_LOG_LEVEL env var or ros2 param to control log verbosity.
  }

  // ── 帧控制 ──────────────────────────────────────────────────────────────

  /// 标记新一帧开始 (在 on_timer_impl 第一行调用)
  inline void tick() {
    if (!enabled_) return;
    frame_start_ = clock::now();
    chkpt_time_  = frame_start_;
    steps_this_frame_.clear();
  }

  /// 标记步骤分界点 (记录从上一个 checkpoint 到现在的耗时)
  inline void checkpoint(const char *name) {
    if (!enabled_) return;
    auto now = clock::now();
    double dt = dur_ms(chkpt_time_, now);
    record_step(name, dt);
    chkpt_time_ = now;
  }

  /// 标记帧结束 (在 on_timer_impl 最后一行调用)
  inline void tick_end() {
    if (!enabled_) return;
    auto now = clock::now();
    double frame_ms = dur_ms(frame_start_, now);
    frame_times_.push_back(frame_ms);
    frame_count_++;
    total_frames_++;
    total_time_ms_ += frame_ms;

    // 捕获未计量时间段
    if (chkpt_time_ > frame_start_) {
      double gap = dur_ms(chkpt_time_, now);
      if (gap > 0.005) record_step("_gap_", gap);
    }

    // 周期报告
    if (log_every_n_ > 0 && frame_count_ % log_every_n_ == 0)
      print_report();
  }

  // ── 报告输出 ────────────────────────────────────────────────────────────

  /// 手动输出当前统计窗口报告
  void report() { if (enabled_ && !frame_times_.empty()) print_report(); }

  /// 节点销毁前调用: 输出最终报告 + 可选 JSON 落盘
  void finalize(const std::string &json_path = "") {
    if (!enabled_ || frame_times_.empty()) return;
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
        "[Profiler] 最终: %d 帧, 总耗时 %.1f s, 平均 %.1f ms/帧, 等效 %.1f Hz",
        total_frames_, total_time_ms_ / 1000.0, avg_frame_ms(), effective_hz());
    }
    print_report();
    if (!json_path.empty()) save_json(json_path);
  }

  /// 保存 JSON 报告到文件
  void save_json(const std::string &filepath) {
    std::ofstream f(filepath);
    if (!f) {
      if (node_) RCLCPP_WARN(node_->get_logger(),
        "[Profiler] 无法写入: %s", filepath.c_str());
      return;
    }
    f << build_json();
    if (node_) RCLCPP_INFO(node_->get_logger(),
      "[Profiler] JSON 报告已保存: %s", filepath.c_str());
  }

  // ── 属性 ────────────────────────────────────────────────────────────────

  double avg_frame_ms() const {
    if (frame_times_.empty()) return 0.0;
    double sum = 0.0;
    for (auto x : frame_times_) sum += x;
    return sum / static_cast<double>(frame_times_.size());
  }

  double effective_hz() const {
    double avg = avg_frame_ms();
    return avg > 0.0 ? 1000.0 / avg : 0.0;
  }

  int total_frames() const { return total_frames_; }
  bool is_enabled() const { return enabled_; }
  void set_enabled(bool v) { enabled_ = v; }

  /// 返回统计快照 (供外部/测试使用)
  std::string summary() const {
    std::ostringstream oss;
    oss << total_frames_ << " frames, "
        << std::fixed << std::setprecision(1)
        << avg_frame_ms() << " ms avg, "
        << effective_hz() << " Hz";
    return oss.str();
  }

private:
  using clock = std::chrono::steady_clock;
  using tp = std::chrono::steady_clock::time_point;

  struct StepStats {
    int    count   = 0;
    double total   = 0.0;
    double min_val = 1e18;
    double max_val = 0.0;
    double last    = 0.0;
    std::vector<double> history;
  };

  static double dur_ms(tp a, tp b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  }

  static std::string fmt_ms(double val) {
    if (val < 0.005) return "    ~0";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << std::setw(6) << val;
    return oss.str();
  }

  void record_step(const char *name, double dt_ms) {
    auto &s = steps_[name];
    s.count++;
    s.total   += dt_ms;
    s.last     = dt_ms;
    s.min_val  = std::min(s.min_val, dt_ms);
    s.max_val  = std::max(s.max_val, dt_ms);
    s.history.push_back(dt_ms);
    if (s.history.size() > static_cast<size_t>(log_every_n_ * 3))
      s.history.erase(s.history.begin());

    if (step_order_.empty() || step_order_.back() != name) {
      auto it = std::find(step_order_.begin(), step_order_.end(), name);
      if (it == step_order_.end()) step_order_.push_back(name);
    }
  }

  // ── 格式化报告 ──────────────────────────────────────────────────────────

  void print_report() {
    if (!node_ || frame_times_.empty()) return;
    for (auto &line : build_report_lines())
      RCLCPP_INFO(node_->get_logger(), "%s", line.c_str());
  }

  std::vector<std::string> build_report_lines() const {
    std::vector<std::string> lines;
    if (frame_times_.empty()) { lines.push_back("(无数据)"); return lines; }

    int n = static_cast<int>(frame_times_.size());
    double total = 0.0, vmin = 1e18, vmax = 0.0;
    for (auto x : frame_times_) { total += x; vmin = std::min(vmin, x); vmax = std::max(vmax, x); }
    double avg = total / n;
    double hz  = avg > 0 ? 1000.0 / avg : 0.0;
    double step_sum = 0.0;
    for (auto &kv : steps_) step_sum += kv.second.total;
    double coverage = total > 0 ? (step_sum / total * 100.0) : 100.0;

    lines.push_back("");
    lines.push_back("┌─────────────────────────────────────────────────────────────┐");
    {
      std::ostringstream oss;
      oss << "│  FrameProfiler  │  " << n << " 帧窗口  │  等效 " << std::fixed
          << std::setprecision(2) << hz << " Hz";
      lines.push_back(oss.str());
    }
    lines.push_back("├──────────┬──────────┬──────────┬──────────┬──────────┬──────┤");
    lines.push_back("│ 步骤     │    avg   │    min   │    max   │   last   │ 占比 │");
    lines.push_back("├──────────┼──────────┼──────────┼──────────┼──────────┼──────┤");

    for (auto &name : step_order_) {
      if (name.empty() || name[0] == '_') continue;  // skip internal
      auto &s = steps_.at(name);
      double savg = s.count > 0 ? s.total / s.count : 0.0;
      double pct  = total > 0 ? (s.total / total * 100.0) : 0.0;
      std::ostringstream oss;
      oss << "│ " << std::setw(8) << std::left << name << " │"
          << fmt_ms(savg) << " │" << fmt_ms(s.min_val) << " │"
          << fmt_ms(s.max_val) << " │" << fmt_ms(s.last) << " │"
          << std::setw(5) << std::right << std::fixed << std::setprecision(1) << pct << "% │";
      lines.push_back(oss.str());
    }

    lines.push_back("├──────────┴──────────┴──────────┴──────────┴──────────┴──────┤");
    {
      std::ostringstream oss;
      oss << "│  帧总耗时: " << fmt_ms(avg) << " / " << fmt_ms(vmin)
          << " / " << fmt_ms(vmax) << " ms"
          << "  │  覆盖率: " << std::fixed << std::setprecision(0) << coverage << "%";
      lines.push_back(oss.str());
    }

    // 覆盖率警告
    if (coverage < 90.0) {
      lines.push_back("│  ⚠ checkpoint 覆盖率不足 — 存在大量未计量时间          │");
    }

    // 瓶颈提示
    if (hz < 15.0 && !step_order_.empty()) {
      double worst_avg = 0;
      for (auto &name : step_order_) {
        if (name.empty() || name[0] == '_') continue;
        auto &s = steps_.at(name);
        double savg = s.count > 0 ? s.total / s.count : 0;
        if (savg > worst_avg) worst_avg = savg;
      }
      if (worst_avg > 1.0) {
        std::ostringstream oss;
        oss << "│  → 最大瓶颈: 步骤平均 " << std::fixed << std::setprecision(1)
            << worst_avg << " ms, 占单帧 " << (worst_avg / avg * 100.0) << "%";
        lines.push_back(oss.str());
      }
      if (hz < 1.0) {
        lines.push_back("│  ⚡ 严重低帧率: 检查 WSL 内存/swap (free -h; vmstat 1) │");
      }
    }

    lines.push_back("└─────────────────────────────────────────────────────────────┘");
    return lines;
  }

  // ── JSON 输出 ────────────────────────────────────────────────────────────

  std::string build_json() const {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"total_frames\": " << total_frames_ << ",\n";
    oss << "  \"window_frames\": " << frame_times_.size() << ",\n";
    oss << "  \"avg_ms\": " << std::fixed << std::setprecision(3) << avg_frame_ms() << ",\n";
    oss << "  \"effective_hz\": " << std::fixed << std::setprecision(2) << effective_hz() << ",\n";
    oss << "  \"steps\": {\n";
    bool first = true;
    for (auto &name : step_order_) {
      if (!first) { oss << ",\n"; }
      first = false;
      auto &s = steps_.at(name);
      double savg = s.count > 0 ? s.total / s.count : 0.0;
      oss << "    \"" << name << "\": {"
          << "\"count\":" << s.count
          << ",\"avg_ms\":" << std::fixed << std::setprecision(3) << savg
          << ",\"min_ms\":" << s.min_val
          << ",\"max_ms\":" << s.max_val
          << ",\"std_ms\":" << stddev(s.history)
          << "}";
    }
    oss << "\n  }\n}\n";
    return oss.str();
  }

  // ── 成员 ────────────────────────────────────────────────────────────────

  rclcpp::Node *node_;
  bool enabled_;
  int  log_every_n_;
  int  frame_count_   = 0;
  int  total_frames_  = 0;
  double total_time_ms_ = 0.0;

  tp frame_start_;
  tp chkpt_time_;
  std::vector<double> frame_times_;
  std::map<std::string, StepStats, std::less<>> steps_;
  std::vector<std::string> step_order_;
  std::vector<const char *> steps_this_frame_;  // for tracking order within frame
};

}  // namespace ft_perf
