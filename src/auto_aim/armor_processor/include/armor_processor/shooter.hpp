#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>

#include "auto_aim/command.hpp"
#include "auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const auto_aim::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

  bool shoot(
    const auto_aim::Command & command, const bool & valid,
      const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  double last_v2 = 0 ;
  double last__v2 = 0;
  bool auto_fire_;
  double max_distance_tolerance_;
  std::chrono::steady_clock::time_point fire_hold_until_{};
  double fire_hold_duration_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP