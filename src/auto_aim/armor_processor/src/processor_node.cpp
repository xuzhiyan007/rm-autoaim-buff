// Copyright 2022 Chen Jun
#include "armor_processor/processor_node.hpp"
#include <std_msgs/msg/u_int16.hpp>
// STD
#include <memory>
#include <vector>
#include <angles/angles.h>
#include "tools/trajectory.hpp"

namespace auto_aim
{
ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions& options) : Node("armor_processor", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting ProcessorNode!");

  // Tracker
  // double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  // double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  // tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  // tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  // lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  std::string config_path = this->declare_parameter("tracker_config_path", "/home/jlcv/infantry/src/vision_bringup/config/standard3.yaml");
  tracker_ = std::make_unique<auto_aim::Tracker>(config_path, tools::g_ros_clock.now());
  aimer_ = std::make_unique<auto_aim::Aimer> (config_path);
  shooter_ = std::make_unique<Shooter>(config_path);

  // // Subscriber with tf2 message_filter
  // // tf2 relevant
  // tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // // Create the timer interface before call to waitForTransform,
  // // to avoid a tf2_ros::CreateTimerInterfaceException exception
  // auto timer_interface =
  //     std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  // tf2_buffer_->setCreateTimerInterface(timer_interface);
  // tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // // subscriber and filter
  // armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "world");
  // tf2_filter_ =
  //     std::make_shared<tf2_filter>(armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
  //                                  this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  // tf2_filter_->registerCallback(&ArmorProcessorNode::armorsCallback, this);

  armors_sub_ = this->create_subscription<jlcv_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS(),
    std::bind(&ArmorProcessorNode::armorsCallback, this, std::placeholders::_1));

  bullet_num_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
    "/permitted_pill_number", rclcpp::SensorDataQoS(),
    std::bind(&ArmorProcessorNode::bulletCallback, this, std::placeholders::_1));

  gimbal_command_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    "/gimbal_command", rclcpp::SensorDataQoS());
  gimbal_command_pub_debug = create_publisher<geometry_msgs::msg::Vector3>(
      "/gimbal_command_debug", rclcpp::SensorDataQoS());
  

  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/reset",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->state_ = "lost";
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Tracker reset!");
      return;
    });  

  show_mark_ = this->declare_parameter("show_mark", true);
  if(show_mark_)
  {
    // Measurement publisher (for debug usage)
    measure_pub_ = this->create_publisher<jlcv_interfaces::msg::Measurement>("/tracker/measurement", 10);

    // Publisher
    // target_pub_ = this->create_publisher<jlcv_interfaces::msg::Target>("/processor/target", rclcpp::SensorDataQoS());

    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;

    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;

    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;

    armors_marker_.ns = "armors";
    armors_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    armors_marker_.scale.x = armors_marker_.scale.y = armors_marker_.scale.z = 0.1;
    armors_marker_.color.a = 1.0;
    armors_marker_.color.b = 1.0;

    armors_aimed_marker_.ns = "armor_aimed";
    armors_aimed_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    armors_aimed_marker_.scale.x = armors_aimed_marker_.scale.y = armors_aimed_marker_.scale.z = 0.12;
    armors_aimed_marker_.color.a = 1.0;
    armors_aimed_marker_.color.r = 1.0;
    armors_aimed_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);


    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/processor/marker", 10);

    marker_pub_detector = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);
      // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    armor_marker_.ns = "armors_detecter";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.y = 0.15;
    armor_marker_.scale.z = 0.12;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 1.0;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    position_marker_.header.frame_id = target_frame_;
    linear_v_marker_.header.frame_id = target_frame_;
    angular_v_marker_.header.frame_id = target_frame_;
    armors_marker_.header.frame_id = target_frame_;
    armors_aimed_marker_.header.frame_id = target_frame_;

  }

}
void ArmorProcessorNode::bulletCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  bullet_num = msg->data;
}
void ArmorProcessorNode::armorsCallback(const jlcv_interfaces::msg::Armors::SharedPtr armors_msg)
{
  try {
    Eigen::Quaterniond q(
      armors_msg->q.w,
      armors_msg->q.x,
      armors_msg->q.y,
      armors_msg->q.z
  );
    tracker_->set_R_gimbal2world(q);

    // // Filter abnormal armors
    // armors_msg->armors.erase(
    //     std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
    //                    [this](const jlcv_interfaces::msg::Armor& armor) {
    //                      return abs(armor.pose.position.z) > 1.2 ||
    //                             Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
    //                                 max_armor_distance_;
    //                    }),
    //     armors_msg->armors.end());

    std::list<auto_aim::Armor_point> armors_point{};
    for(auto armor : armors_msg->armors)
    {
      std::vector<cv::Point2f> points;
      if(armor.points_x.size()!=4 || armor.points_y.size()!=4) 
      {
        RCLCPP_WARN(this->get_logger(), "跳过点数不足的装甲板，点数：%ld", points.size());
        continue;
      }else{
        for(uint8_t i=0;i<4;i++)
        {
          points.emplace_back(cv::Point2f(armor.points_x[i],armor.points_y[i]));
        }
      }
      // if(points.size() != 4){
      //   continue;
      // }

      //std::cout<<points.size() <<"\n";

      
      armors_point.push_back(auto_aim::Armor_point(armor.type,armor.number,points));
    }
    auto targets = tracker_->track(armors_point, armors_msg->header.stamp);
    
    // int id = tracker_->getTargetID();
    // id = id+1;
    // std::cout << "id: " <<id;
    // std_msgs::msg::Float64 debug_msg;
    // debug_msg.data = id;
    // debug_pub_->publish(debug_msg); 

    //std::cout<<"num: "<<bullet_num<<std::endl;
    auto command = aimer_->aim(targets, armors_msg->header.stamp, 22.1);

    auto gimbal_pos = tracker_->get_gimbal_pos();

    // if (!targets.empty() && command.control) {
    //   constexpr double high_spin_threshold = 5.0;
    //   constexpr double high_spin_delay = 0.035;
    //   constexpr double bullet_speed = 23.0;
    //   constexpr double fixed_head_fire_window = 2.5 / 57.3;
    //   constexpr double fixed_head_lock_tolerance = 2.5 / 57.3;

    //   auto fixed_head_target = targets.front();
    //   auto target_state = fixed_head_target.ekf_x();
    //   const bool high_spin = std::abs(target_state[7]) > high_spin_threshold;

    //   if (high_spin) {
    //     double predict_dt = tools::delta_time(tools::g_ros_clock.now(), armors_msg->header.stamp) + high_spin_delay;
    //     const double center_distance = std::hypot(target_state[0], target_state[2]);
    //     predict_dt += center_distance / bullet_speed;
    //     if (predict_dt < 0) predict_dt = 0;

    //     fixed_head_target.predict(predict_dt);
    //     target_stabte = fixed_head_target.ekf_x();

    //     const double center_yaw = std::atan2(target_state[2], target_state[0]);
    //     command.yaw = center_yaw + aimer_->yaw_offset_;

    //     bool armor_in_fire_window = false;
    //     for (const auto & armor_xyza : fixed_head_target.armor_xyza_list()) {
    //       const double armor_yaw = std::atan2(armor_xyza[1], armor_xyza[0]);
    //       const double yaw_error = angles::shortest_angular_distance(center_yaw, armor_yaw);
    //       if (std::abs(yaw_error) < fixed_head_fire_window) {
    //         armor_in_fire_window = true;
    //         break;
    //       }
    //     }
        
    //     tools::Trajectory temp = tools::Trajectory(bullet_speed, center_distance-target_state[8], target_state[10] / 2 + target_state[4]);
    //     if(!temp.unsolvable){
    //       command.pitch = -temp.pitch-1.09 / 57.3;
    //     } 
        
    //     const double lock_error = angles::shortest_angular_distance(gimbal_pos[0], command.yaw);
    //     const bool gimbal_locked_center = std::abs(lock_error) < fixed_head_lock_tolerance;
    //     command.shoot = armor_in_fire_window && gimbal_locked_center;
    //   } else {
    //     command.shoot =  shooter_->shoot(command, aimer_->debug_aim_point.valid, targets, gimbal_pos);
    //   }
    // } else {
    //   command.shoot = false;
    // }
    command.shoot =  shooter_->shoot(command, aimer_->debug_aim_point.valid, targets, gimbal_pos);
    
    command_debug.x = gimbal_pos[1];
    command_debug.y = command.pitch;
    command_debug.z = command.shoot == 1 ? 2 : command.control;
    //shoot_flag发3是攻击模式，弹频高，发2是防御模式，弹频低

    command_.y = command.yaw;

    gimbal_command_pub_debug->publish(command_debug);

    if(show_mark_)
    {
      armor_marker_.header = text_marker_.header = armors_msg->header; 

      position_marker_.header.stamp = armors_msg->header.stamp;
      linear_v_marker_.header.stamp = armors_msg->header.stamp;
      angular_v_marker_.header.stamp = armors_msg->header.stamp;
      armors_marker_.header.stamp = armors_msg->header.stamp;
      armors_aimed_marker_.header.stamp = armors_msg->header.stamp;

      if (!targets.empty()) {
        auto target = targets.front();
    
        std::vector<Eigen::Vector4d> armor_xyza_list;



        armors_marker_.action = visualization_msgs::msg::Marker::ADD;
        armors_marker_.points.clear();
    
        geometry_msgs::msg::Point p_a;

        // 当前帧target更新后
        armor_xyza_list = target.armor_xyza_list();
        
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {

          p_a.x = xyza[0];
          p_a.y = xyza[1];
          p_a.z = xyza[2];
          armors_marker_.points.emplace_back(p_a);
          
          // if(0)
          // {
          //   auto image_points =
          //   tracker_->solver_->reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          //   tools::draw_points(img, image_points, {0, 255, 0});
          // }

        }

        // 0 1  2 3  4 5  6 7 8 9 10
        // x vx y vy z vz a w r l h
        // a: angle
        // w: angular velocity
        // l: r2 - r1
        // h: z2 - z1
        Eigen::VectorXd x_ = target.ekf_x();
        std::cout << "v_z:" << x_[5] << std::endl;
        

        position_marker_.action = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = x_[0];
        position_marker_.pose.position.y = x_[2];
        position_marker_.pose.position.z = x_[4] + x_[10] / 2;

        linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += x_[1];
        arrow_end.y += x_[3];
        arrow_end.z += x_[5];
        linear_v_marker_.points.emplace_back(arrow_end);

        angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += x_[7] / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);
    
        // aimer瞄准位置
        auto aim_point = aimer_->debug_aim_point;
        std::cout<<"aim_point.valid: "<<aim_point.valid<<"\n";
        if(aim_point.valid)
        {
          armors_aimed_marker_.action = visualization_msgs::msg::Marker::ADD;
          armors_aimed_marker_.points.clear();

          Eigen::Vector4d aim_xyza = aim_point.xyza;

          p_a.x = aim_xyza[0];
          p_a.y = aim_xyza[1];
          p_a.z = aim_xyza[2];
          armors_aimed_marker_.points.emplace_back(p_a);

          // auto center_x = aim_xyza[0] + x_[8] * std::cos(aim_xyza[3]);
          // auto center_y = aim_xyza[1] + x_[8] * std::sin(aim_xyza[3]);
          // // auto center_z = aim_xyza[2];
          // command_.y = std::atan2(center_y, center_x) + aimer_->yaw_offset_;

          auto distance_ = aim_xyza[0]*aim_xyza[0] + aim_xyza[1]*aim_xyza[1] + aim_xyza[2]*aim_xyza[2];
          
          std::cout<<"distance : "<<distance_<<"\n";

          // if((distance_>31.5 && distance_<33.9) || distance_>41)
          // {
          //   command.shoot = 0;
          // }
          // if(0)
          // {
          //   auto image_points =
          //     solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
          //   tools::draw_points(img, image_points, {0, 0, 255});
          // }

        }else{
          armors_aimed_marker_.action = visualization_msgs::msg::Marker::DELETE;
        }

      }else{

        position_marker_.action = visualization_msgs::msg::Marker::DELETE;
        linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
        armors_aimed_marker_.action = visualization_msgs::msg::Marker::DELETE;
      
      }
    
      visualization_msgs::msg::MarkerArray marker_array;

      marker_array.markers.emplace_back(position_marker_);
      marker_array.markers.emplace_back(linear_v_marker_);
      marker_array.markers.emplace_back(angular_v_marker_);
      marker_array.markers.emplace_back(armors_marker_);
      marker_array.markers.emplace_back(armors_aimed_marker_);
      marker_pub_->publish(marker_array);

      publishMarkers_detector();
    
    }  
    
    command_.x = command.pitch;
    command_.z = command.shoot == 1 ? 2 : command.control;
    // if(bullet_num==0) {command_.z = 0;std::cout<<"000000000000000"<<std::endl;}
    gimbal_command_pub_->publish(command_);

  }catch (const std::exception& e)
    {
      std::cerr << "异常信息: " << e.what() << std::endl;
      // std::cerr << "位置: " << e.file << ":" << e.line << std::endl;
    }
    catch (...)
    {
      std::cerr << "未知异常" << std::endl;
    }
}

// void ArmorProcessorNode::publishMarkers(const jlcv_interfaces::msg::Target& target)
// {


// }

void ArmorProcessorNode::publishMarkers_detector()
{
  marker_array_.markers.clear();
  armor_marker_.id = 0;
  text_marker_.id = 0;
  
  for(auto & armor : tracker_->armors_track_)
  {

        // Fill the markers
    armor_marker_.pose.orientation.x = armor.q_in_camera.x();
    armor_marker_.pose.orientation.y = armor.q_in_camera.y();
    armor_marker_.pose.orientation.z = armor.q_in_camera.z();
    armor_marker_.pose.orientation.w = armor.q_in_camera.w();

    armor_marker_.pose.position.x = armor.xyz_in_camera.x();
    armor_marker_.pose.position.y = armor.xyz_in_camera.y();
    armor_marker_.pose.position.z = armor.xyz_in_camera.z();

    text_marker_.pose.position = armor_marker_.pose.position;
    text_marker_.pose.position.y -= 0.1;
    text_marker_.text = auto_aim::ARMOR_NAMES[armor.name];

    // marker_array_.markers.emplace_back(armor_marker_);
    marker_array_.markers.emplace_back(text_marker_);

    armor_marker_.header.frame_id = "pitch_link";
    armor_marker_.pose.orientation.x = armor.q_in_gimbal.x();
    armor_marker_.pose.orientation.y = armor.q_in_gimbal.y();
    armor_marker_.pose.orientation.z = armor.q_in_gimbal.z();
    armor_marker_.pose.orientation.w = armor.q_in_gimbal.w();

    armor_marker_.pose.position.x = armor.xyz_in_gimbal.x();
    armor_marker_.pose.position.y = armor.xyz_in_gimbal.y();
    armor_marker_.pose.position.z = armor.xyz_in_gimbal.z();

    marker_array_.markers.emplace_back(armor_marker_);

    armor_marker_.id++;
    text_marker_.id++;


  }

  if(marker_array_.markers.empty()) return ;

  text_marker_.pose.position.z += 0.3;
  text_marker_.text = tracker_->state();
  marker_array_.markers.emplace_back(text_marker_);

  marker_pub_detector->publish(marker_array_);
}

}  // namespace auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(auto_aim::ArmorProcessorNode)
