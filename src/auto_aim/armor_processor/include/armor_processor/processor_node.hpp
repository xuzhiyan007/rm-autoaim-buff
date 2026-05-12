// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
// #include <message_filters/subscriber.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/create_timer_ros.h>
// #include <tf2_ros/message_filter.h>
// #include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float64.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_processor/tracker.hpp"
#include "jlcv_interfaces/msg/armors.hpp"
#include "jlcv_interfaces/msg/target.hpp"
#include "jlcv_interfaces/msg/measurement.hpp"



#include "auto_aim/aimer.hpp"
#include "armor_processor/shooter.hpp"
#include "auto_aim/tracker.hpp"
#include <geometry_msgs/msg/vector3.hpp>

#include "tools/ros_time.hpp"

// #include <stdexcept>
// #define THROW_ERR(msg) throw LocationException(msg, __FILE__, __LINE__)

namespace auto_aim
{

// struct LocationException : public std::runtime_error
// {
//     std::string file;
//     int line;

//     LocationException(const std::string& msg, const char* file_, int line_)
//         : std::runtime_error(msg), file(file_), line(line_)
//     {}
// };




// using tf2_filter = tf2_ros::MessageFilter<jlcv_interfaces::msg::Armors>;
class ArmorProcessorNode : public rclcpp::Node
{
public:
  explicit ArmorProcessorNode(const rclcpp::NodeOptions & options);

private:

  //rclcpp::Clock steady_clock_{RCL_ROS_TIME};
  uint16_t bullet_num = 0;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr bullet_num_sub_;
  void bulletCallback(const std_msgs::msg::UInt16::SharedPtr msg);




  void armorsCallback(const jlcv_interfaces::msg::Armors::SharedPtr armors_ptr);

  // void publishMarkers(const jlcv_interfaces::msg::Target & target_msg);

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_;
  
  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  // // Subscriber with tf2 message_filter
  std::string target_frame_;
  // std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  // message_filters::Subscriber<jlcv_interfaces::msg::Armors> armors_sub_;
  // std::shared_ptr<tf2_filter> tf2_filter_;

  rclcpp::Subscription<jlcv_interfaces::msg::Armors>::SharedPtr armors_sub_;

  // Measurement publisher
  rclcpp::Publisher<jlcv_interfaces::msg::Measurement>::SharedPtr measure_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub_;

  // Publisher
  // rclcpp::Publisher<jlcv_interfaces::msg::Target>::SharedPtr target_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armors_marker_;
  visualization_msgs::msg::Marker armors_aimed_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_detector;
  void publishMarkers_detector();

    // Armor tracker
  std::unique_ptr<auto_aim::Tracker> tracker_;
  std::unique_ptr<auto_aim::Aimer> aimer_;
  std::unique_ptr<Shooter> shooter_;
  
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_command_pub_;
  geometry_msgs::msg::Vector3 command_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_command_pub_debug;
  geometry_msgs::msg::Vector3 command_debug;

  //debug
  bool show_mark_;
};

}  // namespace auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_