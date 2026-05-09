// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "jlcv_serial_driver/jlcv_serial_driver.hpp"

// ROS
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include "serial_driver/serial_driver.hpp"

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "jlcv_serial_driver/crc.hpp"
#include "jlcv_serial_driver/packet.hpp"

#include "tools/ros_time.hpp"

namespace jlcv_serial_driver
{
SerialDriver::SerialDriver(const rclcpp::NodeOptions& options)
  : Node("jlcv_serial_driver", options)
  , owned_ctx_{ new IoContext(2) }
  , serial_driver_{ new drivers::serial_driver::SerialDriver(*owned_ctx_) }
  , last_mode_(2)
  , curr_mode_(2)
{
  RCLCPP_INFO(get_logger(), "Start SerialDriver!");

  // whether or not sentry robot
  debug_ = this->declare_parameter("debug", false);
  getParams();

  // offset time
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);

  // Create Publisher
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
  game_status_pub_ =
      this->create_publisher<jlcv_interfaces::msg::GameStatus>("/game_status", rclcpp::QoS(rclcpp::KeepLast(1)));
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  robot_mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/robot_mode", rclcpp::QoS(rclcpp::KeepLast(1)));

  // Detect parameter client
  armor_detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
  energy_processor_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "energy_processor");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  // Reset mode service client
  set_mode_client_ = this->create_client<jlcv_interfaces::srv::SetMode>("/set_mode");

  try
  {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open())
    {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&SerialDriver::receiveData, this);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  energy_target_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/ene_energy_processor/energy_target", rclcpp::SensorDataQoS(),
      std::bind(&SerialDriver::AngleSendData, this, std::placeholders::_1));

  target_angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
     "/gimbal_command", rclcpp::SensorDataQoS(),
     std::bind(&SerialDriver::AngleSendData, this, std::placeholders::_1));
}

SerialDriver::~SerialDriver()
{
  if (receive_thread_.joinable())
  {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open())
  {
    serial_driver_->port()->close();
  }

  if (owned_ctx_)
  {
    owned_ctx_->waitForExit();
  }
}

void SerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok())
  {
    try
    {
      serial_driver_->port()->receive(header);
      if (header[0] == 0x5A)
      {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        bool crc_ok = crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
        if (crc_ok)
        {

          // std::cout<<"color: "<<+packet.enemy_color<<"\n";
          if (!initial_set_param_ || packet.enemy_color != previous_receive_color_)
          {
            resetColor(rclcpp::Parameter("enemy_color", int(packet.enemy_color)));
            previous_receive_color_ = packet.enemy_color;
          }

          // update mode
          curr_mode_ = packet.mode;
          if (curr_mode_ != last_mode_)
          {
            ResetMode();
            if (curr_mode_ != jlcv_interfaces::srv::SetMode::Request::ARMOR)
            {
              ResetEnergyMode(rclcpp::Parameter("energy_mode", curr_mode_));
            }
            last_mode_ = curr_mode_;
          }

          sensor_msgs::msg::JointState joint_state;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          joint_state.header.stamp = tools::g_ros_clock.now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          joint_state.name.push_back("pitch_joint");
          joint_state.position.push_back(packet.curr_pitch);
          joint_state.name.push_back("yaw_joint");
          joint_state.position.push_back(packet.curr_yaw>M_PI?packet.curr_yaw-2*M_PI:packet.curr_yaw);
          joint_state_pub_->publish(joint_state);
          
          jlcv_interfaces::msg::GameStatus gamestatus;
          gamestatus.enemy_color = packet.enemy_color;
          gamestatus.game_status = packet.game_status;
          gamestatus.outpost_status = packet.outpost_status;
          game_status_pub_->publish(gamestatus);

          std_msgs::msg::UInt8 mode_msg;
          mode_msg.data = packet.mode;
          robot_mode_pub_->publish(mode_msg);
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      }
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void SerialDriver::AngleSendData(geometry_msgs::msg::Vector3::SharedPtr target_angle)
{
  msg_mutex.lock();
  try
  {
    SendPacket sendpacket;
    SendPacketANGLE packet;
    packet.pitch = target_angle->x;
    packet.yaw = target_angle->y;
    packet.shoot_flag = uint8_t(target_angle->z);

    sendpacket.header = SEND_HEADER_ANGLE;
    sendpacket.data.packetangle = packet;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&sendpacket), sizeof(sendpacket));
    std::vector<uint8_t> data = toVector<SendPacket>(sendpacket);
    serial_driver_->port()->send(data);

    if (debug_)
    {
      std::cout << "pitch : " << target_angle->x << std::endl << "yaw : " << target_angle->y << std::endl;
      std::cout << "shoot_flag : " << target_angle->z << std::endl;
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
  msg_mutex.unlock();
}

void SerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try
  {
    device_name_ = declare_parameter<std::string>("device_name", "");
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try
  {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try
  {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none")
    {
      fc = FlowControl::NONE;
    }
    else if (fc_string == "hardware")
    {
      fc = FlowControl::HARDWARE;
    }
    else if (fc_string == "software")
    {
      fc = FlowControl::SOFTWARE;
    }
    else
    {
      throw std::invalid_argument{ "The flow_control parameter must be one of: none, software, or hardware." };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try
  {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none")
    {
      pt = Parity::NONE;
    }
    else if (pt_string == "odd")
    {
      pt = Parity::ODD;
    }
    else if (pt_string == "even")
    {
      pt = Parity::EVEN;
    }
    else
    {
      throw std::invalid_argument{ "The parity parameter must be one of: none, odd, or even." };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try
  {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0")
    {
      sb = StopBits::ONE;
    }
    else if (sb_string == "1.5")
    {
      sb = StopBits::ONE_POINT_FIVE;
    }
    else if (sb_string == "2" || sb_string == "2.0")
    {
      sb = StopBits::TWO;
    }
    else
    {
      throw std::invalid_argument{ "The stop_bits parameter must be one of: 1, 1.5, or 2." };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void SerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try
  {
    if (serial_driver_->port()->is_open())
    {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok())
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void SerialDriver::resetColor(const rclcpp::Parameter& param)
{
  if (!armor_detector_param_client_->service_is_ready())
  {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  // Set armor enemy_color
  if (!set_param_future_.valid() || set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    RCLCPP_INFO(get_logger(), "Setting armor enemy_color to %s...", param.as_int() == 0 ? "RED" : "BLUE");
    armor_detector_param_client_->set_parameters({ param });
    initial_set_param_ = true;
  }
}

void SerialDriver::ResetMode()
{
  if (!set_mode_client_->service_is_ready())
  {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping mode reset");
    return;
  }

  // Set state
  auto request = std::make_shared<jlcv_interfaces::srv::SetMode::Request>();
  request->mode = curr_mode_;
  set_mode_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset mode !!!");
}

void SerialDriver::ResetEnergyMode(const rclcpp::Parameter& param)
{
  if (!energy_processor_param_client_->service_is_ready())
  {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  // Set energy mode
  if (!set_param_future_.valid() || set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    RCLCPP_INFO(get_logger(), "Setting energy_mode to %s...", param.as_int() == 0 ? "SMALL_ENERGY" : "BIG_ENERGY");
    energy_processor_param_client_->set_parameters({ param });
  }
}

}  // namespace jlcv_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(jlcv_serial_driver::SerialDriver)