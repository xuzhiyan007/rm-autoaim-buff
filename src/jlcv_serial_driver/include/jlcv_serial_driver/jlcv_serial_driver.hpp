// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef jlcv_serial_driver__jlcv_serial_driver_HPP_
#define jlcv_serial_driver__jlcv_serial_driver_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "serial_driver/serial_driver.hpp"

// ROS2 message publish
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "jlcv_interfaces/msg/game_status.hpp"

// ROS2 message subscribe
#include "jlcv_interfaces/msg/target.hpp"
#include "jlcv_interfaces/srv/reset_color.hpp"
#include "jlcv_interfaces/srv/set_mode.hpp"

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <future>

namespace jlcv_serial_driver
{
class SerialDriver : public rclcpp::Node
{
public:
  explicit SerialDriver(const rclcpp::NodeOptions& options);
  ~SerialDriver() override;

private:
  void getParams();

  void receiveData();

  void AngleSendData(geometry_msgs::msg::Vector3::SharedPtr pitch_yaw);
  
  void reopenPort();

  void resetColor(const rclcpp::Parameter& param);

  void ResetEnergyMode(const rclcpp::Parameter& param);

  void ResetMode();

  //rclcpp::Clock steady_clock_{RCL_ROS_TIME};
  
  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::thread receive_thread_;
  std::thread send_thread_;
  std::mutex msg_mutex;

  double timestamp_offset_ = 0;

  // Service client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
  rclcpp::Client<jlcv_interfaces::srv::SetMode>::SharedPtr set_mode_client_;

  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  bool previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr armor_detector_param_client_;
  ResultFuturePtr set_param_future_;

  // SMALL_ENERGY:0  BIG_ENERGY:1  ARMOR:2
  rclcpp::AsyncParametersClient::SharedPtr energy_processor_param_client_;
  uint8_t last_mode_ ;
  uint8_t curr_mode_;

  // message_pub
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<jlcv_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

  // message_sub
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr energy_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_angle_sub_;

  // debug
  bool debug_;
};
}  // namespace jlcv_serial_driver

#endif  // jlcv_serial_driver__jlcv_serial_driver_HPP_
