#include "tools/ros_time.hpp"

namespace tools
{
    rclcpp::Clock g_ros_clock{RCL_ROS_TIME};

    double delta_time(
        const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b)
      {
        std::chrono::duration<double> c = a - b;
        return c.count();
      }
      
    double delta_time(
    const rclcpp::Time & a, const rclcpp::Time & b)
    {
    return (a - b).seconds();
    }

}