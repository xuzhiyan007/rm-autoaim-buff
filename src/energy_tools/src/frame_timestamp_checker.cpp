/**
 * @file frame_timestamp_checker.cpp
 * @brief 帧间时间戳调试工具 — 订阅 hik_camera_node 发布的 /image_raw，
 *        计算相邻帧时间戳差异（帧间隔、FPS、抖动），统计并周期输出。
 *
 * 设计为 rclcpp_components 可组合节点，支持放入 camera_full_container
 * 同一进程内零拷贝通信（use_intra_process_comms: true）。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace energy_tools
{

class FrameTimestampChecker : public rclcpp::Node
{
public:
  explicit FrameTimestampChecker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("frame_timestamp_checker", options)
  {
    // 可配置参数
    this->declare_parameter("topic_name", "/image_raw");
    this->declare_parameter("report_interval_frames", 300);
    this->declare_parameter("report_interval_seconds", 5.0);

    topic_name_ = this->get_parameter("topic_name").as_string();
    report_interval_frames_ = this->get_parameter("report_interval_frames").as_int();
    report_interval_seconds_ = this->get_parameter("report_interval_seconds").as_double();

    // QoS 与 hik_camera_node 发布端对齐：SensorDataQoS (BestEffort + Volatile)
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(10);

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name_, qos,
        [this](sensor_msgs::msg::Image::SharedPtr msg) {
          this->imageCallback(msg);
        });

    RCLCPP_INFO(this->get_logger(),
                "FrameTimestampChecker started. Topic: %s, "
                "report every %d frames or %.1f s",
                topic_name_.c_str(), report_interval_frames_, report_interval_seconds_);
  }

private:
  /// 将 header.stamp 转换为纳秒级双精度浮点数
  static double stampToNanos(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<double>(t.sec) * 1e9 + static_cast<double>(t.nanosec);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const double now_ns = stampToNanos(msg->header.stamp);
    total_frames_++;

    if (has_prev_) {
      const double delta_ns = now_ns - prev_stamp_ns_;
      // 丢弃异常点：帧间隔 <= 0 或 > 200ms（海康 165 fps 标称约 6.06ms，留足余量）
      if (delta_ns > 0.0 && delta_ns < 200.0 * 1e6) {
        intervals_ns_.push_back(delta_ns);
      }
    }

    prev_stamp_ns_ = now_ns;
    has_prev_ = true;

    // 定期报告：按帧数阈值 或 按时间阈值
    const auto now_wall = this->now();
    const double wall_sec = (now_wall - last_report_time_).seconds();

    if (static_cast<int>(intervals_ns_.size()) >= report_interval_frames_ ||
        (wall_sec >= report_interval_seconds_ && !intervals_ns_.empty())) {
      printStats();
      intervals_ns_.clear();
      last_report_time_ = now_wall;
    }
  }

  void printStats()
  {
    if (intervals_ns_.empty()) return;

    const size_t N = intervals_ns_.size();

    // 排序以计算中位数和百分位
    std::vector<double> sorted = intervals_ns_;
    std::sort(sorted.begin(), sorted.end());

    const double sum = std::accumulate(intervals_ns_.begin(), intervals_ns_.end(), 0.0);
    const double mean_ns = sum / N;

    // 标准差
    double sq_sum = 0.0;
    for (double v : intervals_ns_) sq_sum += (v - mean_ns) * (v - mean_ns);
    const double std_ns = std::sqrt(sq_sum / N);

    const double min_ns  = sorted.front();
    const double max_ns  = sorted.back();
    const double med_ns  = (N % 2 == 0)
        ? (sorted[N/2 - 1] + sorted[N/2]) * 0.5
        : sorted[N/2];

    // 第1百分位 / 第99百分位
    auto percentile = [&](double p) -> double {
      double idx = p * (N - 1);
      size_t lo = static_cast<size_t>(idx);
      size_t hi = std::min(lo + 1, N - 1);
      double frac = idx - lo;
      return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
    };
    const double p1_ns  = percentile(0.01);
    const double p99_ns = percentile(0.99);

    // 衍生 FPS
    const double fps_mean = 1e9 / mean_ns;
    const double fps_min  = 1e9 / max_ns;
    const double fps_max  = 1e9 / min_ns;

    RCLCPP_INFO(this->get_logger(),
                "━━━━━━━━━━ Frame Timestamp Report ━━━━━━━━━━\n"
                "  Samples        : %zu / %lu total received\n"
                "  Interval (μs)  : mean=%7.1f  std=%7.1f  min=%7.1f  max=%7.1f  median=%7.1f\n"
                "  Interval (ms)  : mean=%7.2f  std=%7.2f  min=%7.2f  max=%7.2f  median=%7.2f\n"
                "  FPS            : mean=%7.1f  min= %6.1f  max= %6.1f\n"
                "  Jitter P1/P99  : %7.1f / %7.1f μs  (range %7.1f μs)\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━",
                N, static_cast<unsigned long>(total_frames_),
                mean_ns * 1e-3, std_ns * 1e-3, min_ns * 1e-3, max_ns * 1e-3, med_ns * 1e-3,
                mean_ns * 1e-6, std_ns * 1e-6, min_ns * 1e-6, max_ns * 1e-6, med_ns * 1e-6,
                fps_mean, fps_min, fps_max,
                p1_ns * 1e-3, p99_ns * 1e-3, (p99_ns - p1_ns) * 1e-3);
  }

  // ── 订阅 ──
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  // ── 参数 ──
  std::string topic_name_;
  int report_interval_frames_;
  double report_interval_seconds_;

  // ── 状态 ──
  bool has_prev_ = false;
  double prev_stamp_ns_ = 0.0;
  uint64_t total_frames_ = 0;
  std::vector<double> intervals_ns_;

  rclcpp::Time last_report_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace energy_tools

// ── rclcpp_components 注册宏 ──
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(energy_tools::FrameTimestampChecker)
