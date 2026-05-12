// Copyright 2023 Yang Jiaqi
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.hpp>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>

// Eigen
#include <Eigen/Eigen>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <sstream>
#include <iomanip>
#include <ctime>

#include "armor_detector/armor.hpp"
#include "armor_detector/lifecycle_armor_detector.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
LifecycleArmorDetector::LifecycleArmorDetector(const rclcpp::NodeOptions& options): Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting LifecycleArmorDetectorNode!");

  // Detector
  detector_ = initDetector(0);

  // rclcpp::Time time = this ->now ();
  // time_t sec = time.seconds();
  // struct tm *timeinfo = localtime(&sec);
  // std::stringstream ss;
  // ss<<std::put_time(timeinfo,"%Y-%m-%d %H-%M-:%S");
  // int64_t nans = time.nanoseconds()%1000000;
  // int mils = nans/1000000;
  // ss<<"."<<mils;
  // record.open("/home/jlcv21/video/"+ss.str()+".avi", cv::VideoWriter::fourcc('M','J','P','G'), 60.0,cv::Size(1440.0,1080.0));
  // record_.open("/home/jlcv21/video/"+ss.str()+"_.avi", cv::VideoWriter::fourcc('M','J','P','G'), 60.0,cv::Size(1440.0,1080.0));

  // record.open("/home/jlcv/Documents/name.avi", cv::VideoWriter::fourcc('M','J','P','G'), 100.0,cv::Size(1440.0,1080.0));

  //Armors Publisher
  armors_pub_ = this->create_publisher<jlcv_interfaces::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

  //Armors Subscription
  set_mode_service_ = this->create_service<jlcv_interfaces::srv::SetMode>(
    "/set_mode", std::bind(&LifecycleArmorDetector::set_mode_callback, this, std::placeholders::_1, std::placeholders::_2));

  game_status_sub_ = this->create_subscription<jlcv_interfaces::msg::GameStatus>(
    "/game_status", rclcpp::SensorDataQoS(),[this](jlcv_interfaces::msg::GameStatus gamestatus){
      if (!outpost_status_flag){
        if (gamestatus.outpost_status){
          RCLCPP_WARN(get_logger(), "Destory the outpost of enemy!");
          // detector_ = initDetector(gamestatus.outpost_status);
          outpost_status_flag = 1;
        }
      }
    });
  cam_center_ = cv::Point2f(709.300455, 576.638489);
  // cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
  //     "/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
  //       cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
  //       cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
  //       pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
  //       camera_ = std::make_shared<Camera>(camera_info->k, camera_info->d);
  //       cam_info_sub_.reset();
  //     });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&LifecycleArmorDetector::ImageCallback, this, std::placeholders::_1));



  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  target_frame_ = this->declare_parameter("target_frame", "world");


  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  flip_ = this->declare_parameter("flip", false);
  plus_pnp_ = this->declare_parameter("plus_pnp", false);
  use_record_ = this->declare_parameter("use_record", false);

  if(debug_)
  {
    // lights_data_pub_ = this->create_publisher<jlcv_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
    // armors_data_pub_ = this->create_publisher<jlcv_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

    binary_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/binary_img", 10);
    number_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/number_img", 10);
    final_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/final_img", 10);
  }
  if(use_record_)recorder_ = std::make_shared<tools::Recorder>();

  // /robot_mode 门控订阅
  mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/robot_mode", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        robot_mode_.store(msg->data);
      });
  // jointState_status_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
  // [this](sensor_msgs::msg::JointState::SharedPtr msg) { pitch_o = msg->position[0];yaw_o = msg->position[1];});
}
LifecycleArmorDetector::~LifecycleArmorDetector()
{
  // record.release();
  // record_.release();
}

void LifecycleArmorDetector::set_mode_callback(const jlcv_interfaces::srv::SetMode::Request::SharedPtr request,
                                                     jlcv_interfaces::srv::SetMode::Response::SharedPtr response)
{
  Mode = request->mode;
  if (Mode == jlcv_interfaces::srv::SetMode::Request::ARMOR)
  {
    RCLCPP_INFO(this->get_logger(), "Set armor mode !");
    response->success = true;
  }else{
    RCLCPP_WARN(this->get_logger(), "Set energy mode !");
  }
}

void LifecycleArmorDetector::ImageCallback(sensor_msgs::msg::Image::UniquePtr img_msg)
{
  if(robot_mode_.load() != 2)  // 仅在 ARMOR 模式（2）下处理
  {
    return;
  }
    t_ = img_msg->header.stamp;
    armors_msg_.header = img_msg->header;
    armors_msg_.armors.clear();
    try {
      geometry_msgs::msg::TransformStamped transform = tf2_buffer_->lookupTransform(
        target_frame_, "pitch_link", img_msg->header.stamp,rclcpp::Duration::from_nanoseconds(100000000));

        q_.x() = armors_msg_.q.x = transform.transform.rotation.x;
        q_.y() = armors_msg_.q.y = transform.transform.rotation.y;
        q_.z() = armors_msg_.q.z = transform.transform.rotation.z;
        q_.w() = armors_msg_.q.w = transform.transform.rotation.w;
      
        // // 构建4×4矩阵
        // tf2::Quaternion tf_q  = tf2::Quaternion(transform.transform.rotation.x,
        //   transform.transform.rotation.y,
        //   transform.transform.rotation.z,
        //   transform.transform.rotation.w);
        // double roll, pitch, yaw;
      // tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
      // yaw_ = yaw;

      // tf2::Matrix3x3 rotation_matrix;
      // rotation_matrix.setRotation(tf_q);
      // for (int i = 0; i < 3; ++i) {
      //     for (int j = 0; j < 3; ++j) {
      //       camera_->Rotate_pnp2world(i,j) = camera_->Trans_pnp2world(i,j) = rotation_matrix[i][j];
      //     }
      // }
      // camera_->Trans_pnp2world(0, 3) = transform.transform.translation.x;
      // camera_->Trans_pnp2world(1, 3) = transform.transform.translation.y;
      // camera_->Trans_pnp2world(2, 3) = transform.transform.translation.z;
      // camera_->Trans_pnp2world(3,0) = camera_->Trans_pnp2world(3,1) = camera_->Trans_pnp2world(3,2) = 0;
      // camera_->Trans_pnp2world(3,3) = 1;

    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }

    auto armors = detectArmors(std::move(img_msg));


    for (const auto& armor : armors)
    {
      jlcv_interfaces::msg::Armor armor_msg;
      armor_msg.type = armor.type == ArmorType::SMALL  ? 1 : 0;
      armor_msg.number = armor.label_id;
      armor_msg.points_x.emplace_back(armor.left_light.bottom.x);
      armor_msg.points_x.emplace_back(armor.left_light.top.x);
      armor_msg.points_x.emplace_back(armor.right_light.top.x);
      armor_msg.points_x.emplace_back(armor.right_light.bottom.x);

      armor_msg.points_y.emplace_back(armor.left_light.bottom.y);
      armor_msg.points_y.emplace_back(armor.left_light.top.y);
      armor_msg.points_y.emplace_back(armor.right_light.top.y);
      armor_msg.points_y.emplace_back(armor.right_light.bottom.y);  

      armors_msg_.armors.emplace_back(armor_msg);
    }
    // Publishing detected armors
    armors_pub_->publish(armors_msg_);
}

std::unique_ptr<Detector> LifecycleArmorDetector::initDetector(uint8_t outpost_status)
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int min_lightness = declare_parameter("binary_thres", 80, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;

  int enemy_color = declare_parameter("enemy_color", BLUE, param_desc);

  Detector::LightParams l_params = { .min_ratio = declare_parameter("light.min_ratio", 0.08),
                                     .max_ratio = declare_parameter("light.max_ratio", 0.40),
                                     .max_angle = declare_parameter("light.max_angle", 40.0) };

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.6),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.6),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.5),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.5),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)
  };

  auto detector = std::make_unique<Detector>(min_lightness, enemy_color, l_params, a_params);

  // Init classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp_f.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes;
  // if(!outpost_status){
  //   ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string>
  //                                                                     {"negative", "guard"});
  // }
  // else{
  //   ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string>
  //                                                                     {"negative"});
  // }
      ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string>
                                                                      {"negative"});
  detector->classifier = std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

std::vector<Armor> LifecycleArmorDetector::detectArmors(sensor_msgs::msg::Image::UniquePtr img_msg)
{
  // Convert ROS img to cv::Mat
  cv::Mat img(
    img_msg->height,
    img_msg->width,
    CV_8UC3,            // 图像格式
    img_msg->data.data(), // 直接用指针，零拷贝！
    img_msg->step       // 行宽
);
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

  if (flip_)
  {
    cv::flip(img, img, -1);
  }

  if(use_record_)recorder_->record(img.clone(), q_, t_);
  // cv::imshow("img_",img);

  // Update params
  detector_->binary_thres = get_parameter("binary_thres").as_int();
  detector_->enemy_color = get_parameter("enemy_color").as_int();
  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  auto armors = detector_->detect(img);
  auto final_time = tools::g_ros_clock.now();
  auto latency = tools::delta_time(final_time, img_msg->header.stamp)*1000;//(final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  // Publish debug info

  if (debug_)
  {
    // record.write(img);
    binary_img_pub_->publish(*cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
              [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; });
    std::sort(detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
              [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; });

    // lights_data_pub_->publish(detector_->debug_lights);
    // armors_data_pub_->publish(detector_->debug_armors);
    auto all_num_img = detector_->getAllNumbersImage();
    number_img_pub_->publish(*cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    detector_->drawResults(img);
    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    final_img_pub_->publish(*cv_bridge::CvImage(img_msg->header, "bgr8", img).toImageMsg());
    // record_.write(img);
  }
  // cv::imshow("img",img);
  // cv::waitKey(1);

  return armors;
}

}  // namespace auto_aim

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(auto_aim::LifecycleArmorDetector)