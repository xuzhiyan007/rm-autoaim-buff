// // Copyright 2022 Chen Jun

// #ifndef ARMOR_PROCESSOR__TRACKER_HPP_
// #define ARMOR_PROCESSOR__TRACKER_HPP_

// // Eigen
// #include <Eigen/Eigen>

// // ROS
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/quaternion.h>
// #include <geometry_msgs/msg/vector3.hpp>

// // STD
// #include <memory>
// #include <string>

// #include "armor_processor/extended_kalman_filter.hpp"
// #include "jlcv_interfaces/msg/armors.hpp"
// #include "jlcv_interfaces/msg/target.hpp"

// namespace auto_aim
// {
// class Tracker
// {
// enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

// public:
//   Tracker(double max_match_distance, double max_match_yaw_diff);
  
//   using Armors = jlcv_interfaces::msg::Armors;
//   using Armor = jlcv_interfaces::msg::Armor;

//   void init(const Armors::SharedPtr & armors_msg);

//   void update(const Armors::SharedPtr & armors_msg);

//   ExtendedKalmanFilter ekf;

//   int tracking_thres;  // frame
//   double lost_thres;   // second
//   int change_thres;

//   enum State {
//     LOST,
//     DETECTING,
//     TRACKING,
//     TEMP_LOST,
//     CHANGE_TARGET,
//   } tracker_state;


//   Armor tracked_armor;
//   std::string tracked_id;
//   ArmorsNum tracked_armors_num;

//   double info_position_diff;
//   double info_yaw_diff;

//   Eigen::VectorXd measurement;
//   Eigen::VectorXd target_state;
//   int task_mode;

//   // To store another pair of armors message
//   // double last_z, last_r;
//   double dz, another_r;

// private:
//   void initEKF(const Armor & a);
  
//   void initChange(const Armor & a);

//   void updateArmorsNum(const Armor & a);

//   void handleArmorJump(const Armor & a);

//   double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

//   Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

//   double max_match_distance_;
//   double max_match_yaw_diff_;

//   // int tracking_threshold_;
//   // int lost_threshold_;

//   int detect_count_;
//   int lost_count_;
//   int change_count_;

//   double last_yaw_;
// };

// }  // namespace auto_aim

// #endif  // ARMOR_PROCESSOR__TRACKER_HPP_