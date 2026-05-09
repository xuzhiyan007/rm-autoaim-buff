// src/capture.cpp
// 功能：订阅ROS2话题（相机图像和关节状态），实时采集标定数据

#include <fmt/core.h>           // 格式化输出库

#include <filesystem>           // 文件系统操作
#include <fstream>              // 文件流操作
#include <memory>               // 智能指针
#include <opencv2/opencv.hpp>   // OpenCV核心功能
#include <yaml-cpp/yaml.h>      // YAML解析库

#include <rclcpp/rclcpp.hpp>              // ROS2客户端库
#include <cv_bridge/cv_bridge.h>          // ROS图像与OpenCV图像转换
#include <sensor_msgs/msg/image.hpp>      // ROS图像消息类型
#include <sensor_msgs/msg/joint_state.hpp> // ROS关节状态消息类型

#include "tools/img_tools.hpp"   // 图像处理工具函数
#include "tools/logger.hpp"      // 日志工具
#include "tools/math_tools.hpp"  // 数学工具函数

/**
 * @brief 数据采集节点类
 * 
 * 【修改说明】完全重写，将原来的CAN总线接口改为ROS2话题订阅
 */
class DataCaptureNode : public rclcpp::Node
{
public:
  DataCaptureNode() : Node("data_capture_node"), count_(0)
  {
    // 声明ROS2参数（可以从launch文件传入）
    this->declare_parameter("output_folder", "src/handeye_calibration/assets/img_with_q");
    this->declare_parameter("pattern_cols", 11);
    this->declare_parameter("pattern_rows", 8);
    this->declare_parameter("camera_topic", "/image_raw");
    this->declare_parameter("joint_state_topic", "/joint_states");
    

    // 获取参数值
    output_folder_ = this->get_parameter("output_folder").as_string();
    pattern_cols_ = this->get_parameter("pattern_cols").as_int();
    pattern_rows_ = this->get_parameter("pattern_rows").as_int();
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string joint_state_topic = this->get_parameter("joint_state_topic").as_string();

    // 清空旧数据
    if (std::filesystem::exists(output_folder_)) {
      RCLCPP_INFO(this->get_logger(), "检测到已有文件夹: %s", output_folder_.c_str());
      RCLCPP_INFO(this->get_logger(), "正在清空旧数据...");
      
      try {
          // 方法1：完全删除文件夹后重建
          std::filesystem::remove_all(output_folder_);
      } catch (const std::filesystem::filesystem_error& e) {
          RCLCPP_WARN(this->get_logger(), "清空文件夹失败: %s", e.what());
      }
    }

    // 创建输出文件夹
    std::filesystem::create_directories(output_folder_);
    RCLCPP_INFO(this->get_logger(), "输出文件夹: %s", output_folder_.c_str());
    RCLCPP_INFO(this->get_logger(), "标定板尺寸: %d列 x %d行", pattern_cols_, pattern_rows_);
    
    // 订阅相机图像话题
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 
        rclcpp::SensorDataQoS(),  // 使用传感器数据专用QoS
        std::bind(&DataCaptureNode::image_callback, this, std::placeholders::_1));
    
    // 订阅关节状态话题（包含yaw和pitch）
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 10,
      std::bind(&DataCaptureNode::joint_state_callback, this, std::placeholders::_1));
    
    // 创建显示窗口
    cv::namedWindow("Press 's' to save, 'q' to quit", cv::WINDOW_NORMAL);
    
    // 启动定时器，用于处理显示和按键（因为ROS2没有waitKey循环）
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),  // 约30fps的显示频率
      std::bind(&DataCaptureNode::display_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "节点已启动，等待图像和关节状态数据...");
    RCLCPP_INFO(this->get_logger(), "操作说明：按 's' 键保存当前帧，按 'q' 键退出");
  }
  
  ~DataCaptureNode()
  {
    cv::destroyAllWindows();  // 关闭所有OpenCV窗口
  }

private:
  /**
   * @brief 相机图像回调函数
   * @param msg ROS图像消息
   * 
   * 【新增】订阅相机图像，存储最新帧
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // std::cout << "!!!!!!!!!!!!!!!" << std::endl;
    try {
      // 将ROS图像消息转换为OpenCV Mat格式
      current_img_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
      image_received_ = true;  // 标记已接收到图像
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
    }
  }
  
  /**
   * @brief 关节状态回调函数
   * @param msg ROS关节状态消息
   * 
   * 【修改】从/joint_states话题获取yaw和pitch，roll固定为0
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 假设关节状态中存储了yaw和pitch
    // 需要根据实际的消息结构来解析，这里假设：
    // - 第一个关节是yaw
    // - 第二个关节是pitch
    // 或者通过关节名称来查找
    
    double yaw = 0.0, pitch = 0.0;
    
    // 方法1：通过关节名称查找（推荐）
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "pitch_joint") {
        pitch = msg->position[i] * 180.0 / M_PI;  // 弧度转度数
      } else if (msg->name[i] == "yaw_joint") {
        yaw = msg->position[i] * 180.0 / M_PI;
      }
    }
    
    // 方法2：如果关节顺序固定且已知
    // if (msg->name.size() >= 2) {
    //   yaw = msg->position[0] * 180.0 / M_PI;
    //   pitch = msg->position[1] * 180.0 / M_PI;
    // }
    
    // 保存最新的yaw和pitch（度数），roll固定为0
    current_yaw_ = yaw;
    current_pitch_ = pitch;
    current_roll_ = 0.0;  // roll固定为0
    joint_state_received_ = true;
    
    // 调试输出（可选，注释掉以避免刷屏）
    // RCLCPP_DEBUG(this->get_logger(), "Yaw: %.2f, Pitch: %.2f", yaw, pitch);
  }
  
  /**
   * @brief 将欧拉角转换为四元数
   * @param yaw 偏航角（度数）
   * @param pitch 俯仰角（度数）
   * @param roll 滚转角（度数）
   * @return Eigen四元数
   * 
   * 【新增】将接收到的欧拉角转换为四元数，用于保存
   */
  Eigen::Quaterniond euler_to_quaternion(double yaw, double pitch, double roll)
  {
    // 度数转弧度
    yaw = yaw * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    roll = roll * M_PI / 180.0;
    
    // 按Z-Y-X顺序（yaw-pitch-roll）构建四元数
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    
    return q;
  }
  
  /**
   * @brief 保存当前帧的图像和四元数
   * 
   * 【修改】将原来的write_q函数集成到类方法中
   */
  void save_data()
  {
    if (!image_received_) {
      RCLCPP_WARN(this->get_logger(), "未接收到图像数据，无法保存");
      return;
    }
    
    if (!joint_state_received_) {
      RCLCPP_WARN(this->get_logger(), "未接收到关节状态数据，无法保存");
      return;
    }
    
    count_++;
    
    // 保存图像
    std::string img_path = fmt::format("{}/{}.jpg", output_folder_, count_);
    cv::imwrite(img_path, current_img_);
    
    // 将欧拉角转换为四元数
    Eigen::Quaterniond q = euler_to_quaternion(current_yaw_, current_pitch_, current_roll_);
    
    // 保存四元数（w x y z格式）
    std::string q_path = fmt::format("{}/{}.txt", output_folder_, count_);
    std::ofstream q_file(q_path);
    q_file << fmt::format("{} {} {} {}", q.w(), q.x(), q.y(), q.z());
    q_file.close();
    
    RCLCPP_INFO(this->get_logger(), "[%d] 已保存: %s 和 %s", 
                count_, img_path.c_str(), q_path.c_str());
    RCLCPP_INFO(this->get_logger(), "    姿态: yaw=%.2f, pitch=%.2f, roll=%.2f", 
                current_yaw_, current_pitch_, current_roll_);
  }
  
  /**
   * @brief 显示回调函数，处理图像显示和按键
   * 
   * 【新增】定时器回调，用于实时显示图像和标定板识别结果
   */
  void display_callback()
  {
    if (!image_received_) return;  // 还没有图像数据
    
    // 复制当前图像用于显示
    cv::Mat display_img = current_img_.clone();
    
    // std::cout << "??????????????????" <<std::endl;

    // 在图像上显示欧拉角
    if (joint_state_received_) {
      tools::draw_text(display_img, fmt::format("Yaw: {:.2f}", current_yaw_), 
                       {40, 40}, {0, 0, 255});
      tools::draw_text(display_img, fmt::format("Pitch: {:.2f}", current_pitch_), 
                       {40, 80}, {0, 0, 255});
      tools::draw_text(display_img, fmt::format("Roll: {:.2f} (fixed)", current_roll_), 
                       {40, 120}, {0, 0, 255});
    } else {
      tools::draw_text(display_img, "Waiting for joint state...", 
                       {40, 40}, {0, 0, 255});
    }
    
    // 识别标定板圆点网格
    cv::Size pattern_size(pattern_cols_, pattern_rows_);
    std::vector<cv::Point2f> centers_2d;
    bool success = cv::findChessboardCorners(current_img_, pattern_size, centers_2d);
    
    // 在图像上绘制识别结果
    cv::drawChessboardCorners(display_img, pattern_size, centers_2d, success);
    
    // 在图像上显示保存计数
    tools::draw_text(display_img, fmt::format("Saved: {}", count_), 
                     {40, display_img.rows - 40}, {0, 255, 0});
    
    // 显示操作提示
    tools::draw_text(display_img, "Press 's' to save, 'q' to quit", 
                     {40, display_img.rows - 80}, {255, 255, 255});
    
    // 缩小图像便于显示
    cv::Mat resized_img;
    cv::resize(display_img, resized_img, {}, 0.6, 0.6);
    cv::imshow("Press 's' to save, 'q' to quit", resized_img);
    
    // 处理按键（OpenCV的waitKey在ROS2中需要特殊处理）
    int key = cv::waitKey(1);
    if (key == 's') {
      save_data();
    } else if (key == 'q') {
      RCLCPP_INFO(this->get_logger(), "用户请求退出");
      rclcpp::shutdown();
    }
  }
  void load_camera_info()
  {
    std::string camera_info_path = this->get_parameter("camera_info_path").as_string();
    
    try {
      YAML::Node yaml = YAML::LoadFile(camera_info_path);
      auto camera_matrix_data = yaml["camera_matrix"]["data"].as<std::vector<double>>();
      // 可以用于图像矫正等
    } catch (const YAML::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "无法加载相机内参: %s", e.what());
    }
  }
  // 成员变量
  int count_;                              // 保存的图片计数
  std::string output_folder_;              // 输出文件夹路径
  int pattern_cols_;                       // 标定板列数
  int pattern_rows_;                       // 标定板行数
  
  cv::Mat current_img_;                    // 当前图像
  double current_yaw_;                     // 当前yaw角（度数）
  double current_pitch_;                   // 当前pitch角（度数）
  double current_roll_;                    // 当前roll角（度数，固定为0）
  
  bool image_received_ = false;            // 是否已接收到图像
  bool joint_state_received_ = false;      // 是否已接收到关节状态
  
  // ROS2订阅器和定时器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief 主函数
 * 
 * 【修改说明】改为ROS2节点入口
 */
int main(int argc, char * argv[])
{
  // 初始化ROS2
  rclcpp::init(argc, argv);
  
  // 创建节点
  auto node = std::make_shared<DataCaptureNode>();
  
  RCLCPP_INFO(node->get_logger(), "数据采集节点已启动");
  RCLCPP_INFO(node->get_logger(), "注意：roll角固定为0度");
  
  // 运行节点（阻塞直到shutdown）
  rclcpp::spin(node);
  
  // 清理
  rclcpp::shutdown();
  return 0;
}