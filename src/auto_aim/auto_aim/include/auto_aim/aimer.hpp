#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <list>

// #include "io/cboard.hpp"
#include "auto_aim/command.hpp"
#include "auto_aim/target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  Command aim(
    std::list<Target> targets, rclcpp::Time timestamp, double bullet_speed,
    bool to_now = true);

  double yaw_offset_;

  // Command aim(
  //   std::list<Target> targets, rclcpp::Time timestamp, double bullet_speed,
  //   io::ShootMode shoot_mode, bool to_now = true);

private:

  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
  AimPoint choose_aim_point(const Target & target);

  //rclcpp::Clock steady_clock_{RCL_ROS_TIME};
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP