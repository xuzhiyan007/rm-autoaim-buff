#ifndef TOOLS__ROS_TIME_HPP
#define TOOLS__ROS_TIME_HPP
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace tools
{
    extern rclcpp::Clock g_ros_clock;
    // 计算时间差a - b，单位：s
    double delta_time(
        const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);
  
    double delta_time(
        const rclcpp::Time & a, const rclcpp::Time & b);
}
#endif 