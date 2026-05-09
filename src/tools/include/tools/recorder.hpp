#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "thread_safe_queue.hpp"
namespace tools
{
class Recorder
{
public:
  Recorder(double fps = 30);
  ~Recorder();
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const rclcpp::Time & timestamp);

private:
  struct FrameData
  {
    cv::Mat img;
    Eigen::Quaterniond q;
    rclcpp::Time timestamp;
  };
  bool init_;
  std::atomic<bool> stop_thread_;
  double fps_;
  std::string text_path_;
  std::string video_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;  // 负责保存帧数据的线程
  void init(const cv::Mat & img);
  void save_to_file();

  //rclcpp::Clock steady_clock_{RCL_ROS_TIME};
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP