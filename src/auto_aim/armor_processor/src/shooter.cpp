#include "armor_processor/shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  // max_distance_tolerance_ = yaml["max_distance_tolerance"].as<double>();  // m
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  fire_hold_duration_ = yaml["fire_hold_time"].IsDefined()
                          ? yaml["fire_hold_time"].as<double>()
                          : 0.04;  // s
}

bool Shooter::shoot(
  const auto_aim::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto v_2 = targets.front().ekf_x()[1]*targets.front().ekf_x()[1] + targets.front().ekf_x()[3]*targets.front().ekf_x()[3];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  tolerance = v_2<0.64  ? tolerance : tolerance/3;
  std::cout<<"v_2: "<<v_2<<std::endl;
  // //tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  // bool change = 0;
  // if(last__v2==0){
  //   last__v2 = last_v2 = v_2;
  // }else{
  //   if(abs(last_v2-v_2)<0.25)
  //   {
  //     last__v2 = (last_v2+v_2)/2;
  //   }else{
  //     if(abs(last__v2-v_2)>0.25)
  //     {
  //       change = 1;
  //     }
  //     last__v2 = last__v2*0.7 + v_2*0.3;
  //   } 
  //   last_v2 = v_2;
  // }
  // if (change==0&&
  if(
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

bool Shooter::shoot(
  const auto_aim::Command & command, const bool & valid,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  // 丢目标 / 不主动瞄准 / 关闭自瞄：立即停火并清空保持窗口
  if (!command.control || targets.empty() || !auto_fire_) {
    fire_hold_until_ = {};
    last_command_ = command;
    return false;
  }

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  double distance = std::sqrt(tools::square(target_x) + tools::square(target_y)) ;
  std::cout<<"shoot2: "<<distance<<std::endl;
  // //tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  bool conditions =
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    // distance < max_distance_tolerance_ &&
    valid;

  last_command_ = command;

  auto now = std::chrono::steady_clock::now();
  if (conditions) {
    // 满足开火条件：续命保持窗口
    fire_hold_until_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(fire_hold_duration_));
    return true;
  }

  // 阈值边界抖动时，保持窗口内继续返回 true，让下位机连发不被打断
  return now < fire_hold_until_;
}

}  // namespace auto_aim