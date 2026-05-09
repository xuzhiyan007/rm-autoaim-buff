#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <list>
#include <string>

#include "auto_aim/armor.hpp"
#include "auto_aim/solver.hpp"
#include "auto_aim/target.hpp"
// #include "auto_aim/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path,const rclcpp::Time & t);

  std::string state() const;
  std::string state_, pre_state_;
  std::unique_ptr<Solver> solver_;

  std::list<Armor_point> armors_track_;
  
  std::list<Target> track(
    std::list<Armor_point> & armors, rclcpp::Time t,
    bool use_enemy_color = true);

  // std::tuple<omniperception::DetectionResult, std::list<Target>> track(
  //   const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  //   rclcpp::Time t, bool use_enemy_color = true);

  void set_R_gimbal2world(const Eigen::Quaterniond & q);
  Eigen::Vector3d get_gimbal_pos();

private:
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  Target target_;
  rclcpp::Time last_timestamp_;
  ArmorPriority omni_target_priority_;

  void state_machine(bool found);

  bool set_target(std::list<Armor_point> & armors, rclcpp::Time t);

  bool update_target(std::list<Armor_point> & armors, rclcpp::Time t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP