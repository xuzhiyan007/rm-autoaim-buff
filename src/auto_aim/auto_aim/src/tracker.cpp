#include "auto_aim/tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/ros_time.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path,const rclcpp::Time & t)
: solver_(std::make_unique<Solver> (config_path)),
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(t),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
  target_.name = ArmorName::not_armor;
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
  std::list<Armor_point> & armors, rclcpp::Time t, bool use_enemy_color)
{

  armors_track_.clear();

  //armors 没有进行任何筛选判断的
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    //tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });
  armors.remove_if([this](const Armor_point & a) {
    return a.name == ArmorName::not_armor;
  });
      // Fill the distance to image center
  // Change the distance to rank
  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor_point & a, const Armor_point & b) {
    
    return a.distance_to_image_center < b.distance_to_image_center;
  });

  // // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const Armor_point & a, const Armor_point & b) { return a.priority < b.priority; });
  // std::cout<<"name: "<<armors.front().name<<"prio: "<<armors.front().priority<<"\n";

  // if(state_ != "lost" && !armors.empty() && armors.front().priority == ArmorPriority::first && armors.front().name != target_.name)
  // {
  //   state_ == "lost";
  // }

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  std::cout<<"state_  "<<state_<<"\n";

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    //tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    //tools::logger()->debug("[Target] Bad Converge Found!");
    std::cout<<"[Target] Bad Converge Found!\n";
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};  //没有动态资源，拷贝后独立
  return targets;
}

// std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
//   const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
//   rclcpp::Time t, bool use_enemy_color)
// {
//   omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
//   omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
//   if (!detection_queue.empty()) {
//     temp_target = detection_queue.front();
//   }

//   auto dt = tools::delta_time(t, last_timestamp_);
//   last_timestamp_ = t;

//   // 时间间隔过长，说明可能发生了相机离线
//   if (state_ != "lost" && dt > 0.1) {
//     //tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
//     state_ = "lost";
//   }

//   // 优先选择靠近图像中心的装甲板
//   armors.sort([](const Armor & a, const Armor & b) {
//     cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
//     auto distance_1 = cv::norm(a.center - img_center);
//     auto distance_2 = cv::norm(b.center - img_center);
//     return distance_1 < distance_2;
//   });

//   // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
//   armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

//   bool found;
//   if (state_ == "lost") {
//     found = set_target(armors, t);
//   }

//   // 此时主相机画面中出现了优先级更高的装甲板，切换目标
//   else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
//     found = set_target(armors, t);
//     //tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
//   }

//   // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
//   else if (
//     state_ == "tracking" && !temp_target.armors.empty() &&
//     temp_target.armors.front().priority < target_.priority && target_.convergened()) {
//     state_ = "switching";
//     switch_target = omniperception::DetectionResult{
//       temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
//     omni_target_priority_ = temp_target.armors.front().priority;
//     found = false;
//     //tools::logger()->debug("omniperception find higher priority target");
//   }

//   else if (state_ == "switching") {
//     found = !armors.empty() && armors.front().priority == omni_target_priority_;
//   }

//   else if (state_ == "detecting" && pre_state_ == "switching") {
//     found = set_target(armors, t);
//   }

//   else {
//     found = update_target(armors, t);
//   }

//   pre_state_ = state_;
//   // 更新状态机
//   state_machine(found);

//   // 发散检测
//   if (state_ != "lost" && target_.diverged()) {
//     //tools::logger()->debug("[Tracker] Target diverged!");
//     state_ = "lost";
//     return {switch_target, {}};  // 返回switch_target和空的targets
//   }

//   if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

//   std::list<Target> targets = {target_};
//   return {switch_target, targets};
// }

void Tracker::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  solver_->set_R_gimbal2world(q);
}

Eigen::Vector3d Tracker::get_gimbal_pos()
{
  return tools::eulers(solver_->R_gimbal2world(), 2, 1, 0);
}


void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor_point> & armors, rclcpp::Time t)
{
  if (armors.empty()) return false;
  
  auto & armor = armors.front();
  solver_->solve(armor);
  while (armor.xyz_in_world.norm()>6)
  {
    armors.pop_front();
    if (armors.empty()) return false;
    
    armor = armors.front();
    solver_->solve(armor);
  }
  
  armors_track_.push_back(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {      // x  vx  y  vy  z  vz  a    w   r   l   h
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 121, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

bool Tracker::update_target(std::list<Armor_point> & armors, rclcpp::Time t)
{
  target_.predict(t);

  int found_count = 0;   //发现的板数
  // double min_x = 1e10;  // 画面最左侧
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    found_count++;
    
    // solver_->solve(armor);
    
    // armors_track_.push_back(armor);

    // target_.update(armor);
  }
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    solver_->solve(armor);
    if(armor.xyz_in_world.norm()>6.4){
      found_count -= 1;
      continue;
    }
    
    armors_track_.push_back(armor);

    target_.update(armor);
  }
  target_.found_ = found_count;

  if (found_count <= 0) return false;
  return true;
}
  // int Tracker::getTargetID(){
  //   return target_.now_id;
  // }

}  // namespace auto_aim