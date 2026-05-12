#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "auto_aim/armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only
  int found_ = 0;
  
  bool v_id[4];
  bool rebuilt_;
  double z_at_id_[3] = {0.0, 0.0, 0.0};

  Target() = default;
  Target(
    const Armor_point & armor, rclcpp::Time t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(rclcpp::Time t);
  void predict(double dt);
  void update(const Armor_point & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool check_rebuild_optpost();

  void rebuild_outpost(double z_now,int& id);

  bool isinit = false;

  bool checkinit();

  int now_id;

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  rclcpp::Time t_;

  void update_ypda(const Armor_point & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP