// Copyright 2023 Yang Jiaqi
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__LIFECYCLE_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__LIFECYCLE_DETECTOR_NODE_HPP_

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "armor_detector/solvepnp.hpp"
#include "jlcv_interfaces/msg/armors.hpp"
#include "jlcv_interfaces/msg/game_status.hpp"
#include "jlcv_interfaces/srv/reset_color.hpp"
#include "jlcv_interfaces/srv/set_mode.hpp"


//tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <message_filters/subscriber.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <atomic>

#include "tools/recorder.hpp"
#include "tools/ros_time.hpp"

namespace auto_aim
{
class LifecycleArmorDetector : public rclcpp::Node
{
public:
  explicit LifecycleArmorDetector(const rclcpp::NodeOptions& options);
  ~LifecycleArmorDetector();// = default;

  // Set Mode
  void set_mode_callback(const jlcv_interfaces::srv::SetMode::Request::SharedPtr request,
                               jlcv_interfaces::srv::SetMode::Response::SharedPtr response);

  cv::VideoWriter record;
  cv::VideoWriter record_;

private:
  //rclcpp::Clock steady_clock_{RCL_ROS_TIME};
  
  // Armor Detector
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<PnPSolver> pnp_solver_;
  std::shared_ptr<Camera> camera_;
  jlcv_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<jlcv_interfaces::msg::Armors>::SharedPtr armors_pub_;
  std::unique_ptr<Detector> initDetector(uint8_t outpost_status);
  std::vector<Armor> detectArmors(sensor_msgs::msg::Image::UniquePtr img_msg);

  // Set Mode
  uint8_t Mode = 2;
  rclcpp::Service<jlcv_interfaces::srv::SetMode>::SharedPtr set_mode_service_;

  // /robot_mode 门控（0=SMALL_ENERGY, 1=BIG_ENERGY, 2=ARMOR）
  std::atomic<uint8_t> robot_mode_{2};  // 默认 ARMOR，serial_driver 启动后快速修正
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;

  // Camera center
  cv::Point2f cam_center_;
  // Camera info
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  // Image
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  void ImageCallback(sensor_msgs::msg::Image::UniquePtr img_msg);


  // Game status subscription
  rclcpp::Subscription<jlcv_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;

  // Debug information publishers
  bool debug_;
  bool flip_;
  bool plus_pnp_;
  bool outpost_status_flag = 0;
  bool use_record_ = false;

  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<jlcv_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<jlcv_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr number_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr final_img_pub_;

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  double yaw_o = 0;
  double pitch_o = 0;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointState_status_sub_;

  std::shared_ptr<tools::Recorder>  recorder_;
  Eigen::Quaterniond q_;
  rclcpp::Time t_{0, 0, RCL_ROS_TIME};
};

}  // namespace auto_aim

#endif  // ARMOR_DETECTOR__LIFECYCLE_DETECTOR_NODE_HPP_